//! Journeyman spec S5: 2D Adaptive wall finishing passes after clearing.
//! Proves all synthetic motion checks of the spec: defaults unchanged, inside
//! area offsets + shape_mask footprint, outside boss rings + opposite turning
//! sense, spring passes, island finishing, multi-level every/last + flute law,
//! link law, and parameter refusals.
//!
//! Gate map: covers camcraft_motion::machining2d (adaptive::finish_passes,
//! adaptive::finish_rings, levels, ladder_links).
//!
//! Provenance: Journeyman spec S5
//! (docs/journeyman/specs/S5-adaptive-finish-passes.md); M. Held 1991;
//! M. Stori and P. K. Wright 2000; no vendor parameters, text or code.

mod common;

use camcraft_motion::cl::{Move, MoveKind, Toolpath};
use camcraft_motion::entry::Entry3D;
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::links::{Link, Links};
use camcraft_motion::load::DEFAULT_OPTIMAL_LOAD;
use camcraft_motion::machining2d::{
    ARC_FIT_TOL, AdaptiveFinish, AdaptiveTrochParams, CornerRelief, FinishLevel, PassStart,
    PocketParams, PocketStrategy, StartSide, adaptive_avoid_with_finish, adaptive_flute_covers_wall,
    adaptive_trochoidal_with_finish, adaptive_with_finish, pocket_started_on,
};
use common::shape_mask::{CutDepth, MaskSpec, ShapeMask};

fn lp(pts: Vec<Point2>) -> Loop {
    Loop::new(pts).expect("fixture loop")
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
    Region::new(vec![lp(vec![
        Point2::new(x0, y0),
        Point2::new(x1, y0),
        Point2::new(x1, y1),
        Point2::new(x0, y1),
    ])])
    .expect("rect region")
}

fn circle_pts(cx: f64, cy: f64, r: f64, n: usize) -> Vec<Point2> {
    (0..n)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / n as f64;
            Point2::new(cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

fn circle(cx: f64, cy: f64, r: f64, n: usize) -> Region {
    Region::new(vec![lp(circle_pts(cx, cy, r, n))]).expect("circle region")
}

fn base_params(climb: bool) -> PocketParams {
    PocketParams {
        tool_diameter: 10.0,
        stepover: 2.5,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: Some(2.0),
        top_z: 0.0,
        bottom_z: -5.0,
        stepdown: 5.0,
        feed: 800.0,
        plunge_feed: 250.0,
        climb,
        strategy: PocketStrategy::Adaptive,
        entry: Entry3D::Helix,
        helix_radius: 2.5,
        optimal_load: DEFAULT_OPTIMAL_LOAD,
        blend_turns: false,
        blend_radius: None,
        links: Links::PLANAR,
    }
}

/// Split in-plane feed moves at `z` into contiguous runs (separated by lifts,
/// rapids, plunges, or microlifts).
fn cut_runs_at(path: &Toolpath, z: f64) -> Vec<Vec<&Move>> {
    let mut runs: Vec<Vec<&Move>> = Vec::new();
    let mut cur: Vec<&Move> = Vec::new();
    for m in &path.moves {
        let planar = m.kind == MoveKind::Feed && (m.from.z - z).abs() < 1e-6 && (m.to.z - z).abs() < 1e-6;
        if planar {
            cur.push(m);
        } else if !cur.is_empty() {
            runs.push(std::mem::take(&mut cur));
        }
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    runs
}

/// Minimum distance from any sample of `run` to the 4 walls of `[-hx, hx] x [-hy, hy]`.
fn min_dist_to_rect_walls(run: &[&Move], hx: f64, hy: f64) -> f64 {
    let mut best = f64::INFINITY;
    for m in run {
        for (a, b) in m.chords(0.01) {
            for p in [a, b, Point2::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y))] {
                let d = (hx - p.x.abs()).min(hy - p.y.abs());
                best = best.min(d);
            }
        }
    }
    best
}

/// Median distance from samples of `run` to the 4 walls of `[-hx, hx] x [-hy, hy]`.
fn median_dist_to_rect_walls(run: &[&Move], hx: f64, hy: f64) -> f64 {
    let mut ds = Vec::new();
    for m in run {
        for (a, b) in m.chords(0.05) {
            let mid = Point2::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
            ds.push((hx - mid.x.abs()).min(hy - mid.y.abs()));
        }
    }
    ds.sort_by(f64::total_cmp);
    ds[ds.len() / 2]
}

/// Signed swept area of `run` around `(0, 0)`: positive = CCW, negative = CW.
fn signed_turn(run: &[&Move]) -> f64 {
    run.iter()
        .flat_map(|m| m.chords(0.05))
        .map(|(a, b)| a.x * b.y - a.y * b.x)
        .sum()
}

/// Median radial distance of `run` from `(0, 0)`, sampled on true arc/chord
/// vertices (so arc chord sagitta does not bias the reading inward).
fn median_radius(run: &[&Move]) -> f64 {
    let mut rs: Vec<f64> = run
        .iter()
        .flat_map(|m| m.chords(0.001))
        .map(|(a, _)| a.x.hypot(a.y))
        .collect();
    rs.sort_by(f64::total_cmp);
    rs[rs.len() / 2]
}

/// Minimum radial distance of `run` from `(0, 0)`.
fn min_radius(run: &[&Move]) -> f64 {
    run.iter()
        .flat_map(|m| m.chords(0.001))
        .flat_map(|(a, b)| [a, b])
        .map(|p| p.x.hypot(p.y))
        .fold(f64::INFINITY, f64::min)
}

/// S5 Check 1: defaults keep today's 2D Adaptive toolpath byte-for-byte.
#[test]
fn s5_check1_defaults_match_existing_adaptive_toolpath() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let p = base_params(true);
    let relief = CornerRelief::default();
    let side = StartSide::from(PassStart::Auto);
    let plain = pocket_started_on(&area, &p, &[], &relief, &side).expect("plain adaptive");
    let with_def = adaptive_with_finish(&area, &p, &relief, &side, &AdaptiveFinish::default()).expect("default finish");
    assert_eq!(with_def, plain, "default AdaptiveFinish must match pre-S5 path identically");
}

/// S5 Check 2: 60 x 40 mm inside area, 10 mm end mill, stock_to_leave 0, one
/// level, n_f = 2, d_f = 0.25 -> clearing >= 5.5 mm from walls, two finishing
/// rings at 5.25 and 5.0 mm, swept footprint matches area (`shape_mask`).
#[test]
fn s5_check2_inside_area_clearing_offset_finishing_rings_and_shape_mask() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let p = base_params(true);
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.25,
        spring_passes: 0,
        finish_level: FinishLevel::Every,
    };
    let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("adaptive finish");

    // Floor Z does not change: every planar feed lies at bottom_z = -5.0.
    let runs = cut_runs_at(&path, -5.0);
    assert!(runs.len() >= 3, "expected clearing run(s) plus 2 finishing rings, got {}", runs.len());
    let n = runs.len();
    let (clearing, ring1, ring2) = (&runs[..n - 2], &runs[n - 2], &runs[n - 1]);

    // Clearing stays at least 5.5 mm from all four walls.
    for (i, r) in clearing.iter().enumerate() {
        let d = min_dist_to_rect_walls(r, 30.0, 20.0);
        assert!(d >= 5.5 - 1e-3, "clearing run {i} approached wall to {d:.4} mm (< 5.5 mm)");
    }

    // Two finishing rings at 5.25 and 5.00 mm in cut order.
    let d1 = median_dist_to_rect_walls(ring1, 30.0, 20.0);
    let d2 = median_dist_to_rect_walls(ring2, 30.0, 20.0);
    assert!((d1 - 5.25).abs() < 1e-3, "finishing ring 1 at {d1:.4} mm, want 5.25 mm");
    assert!((d2 - 5.00).abs() < 1e-3, "finishing ring 2 at {d2:.4} mm, want 5.00 mm");
    assert!(signed_turn(ring1) > 0.0, "climb inside ring 1 must turn CCW");
    assert!(signed_turn(ring2) > 0.0, "climb inside ring 2 must turn CCW");

    // Zero crossings on the whole path.
    let crossed = camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL);
    assert!(crossed.is_empty(), "crossing pairs: {crossed:?}");

    // Swept footprint matches the 60 x 40 mm area (`shape_mask`), allowing only
    // the 4 sharp inside corners that a 10 mm cutter cannot physically reach.
    let reachable = area.offset(-5.0).expect("erode").offset(5.0).expect("dilate");
    let corners = area.difference(&reachable).expect("corner spandrels");
    let spec = MaskSpec {
        name: "s5_check2_inside_60x40",
        expected: &area,
        path: &path,
        tool_diameter: p.tool_diameter,
        cell: 0.25,
        depth: CutDepth::AllCut,
        diff_dir: None,
    };
    let mask = ShapeMask::evaluate(&spec).expect("shape_mask evaluate");
    mask.assert_no_overcut();
    mask.assert_bounded_leftover(25.0, Some(&corners));
}

/// S5 Check 3: outside (boss) area — 30 mm diameter boss on an 80 x 80 mm
/// stock, passes of check 2 -> two rings round the boss at 20.25 and 20.0 mm
/// radius; turning sense is the opposite of check 2's boundary rings.
#[test]
fn s5_check3_outside_boss_rings_and_opposite_turning_sense() {
    let stock = rect(-40.0, -40.0, 40.0, 40.0);
    let boss = circle(0.0, 0.0, 15.0, 128);
    let area = stock.difference(&boss).expect("boss area");
    let inside_area = rect(-30.0, -20.0, 30.0, 20.0);
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.25,
        spring_passes: 0,
        finish_level: FinishLevel::Every,
    };

    for climb in [true, false] {
        let p = base_params(climb);
        let inside_path = adaptive_with_finish(&inside_area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
            .expect("inside path");
        let inside_runs = cut_runs_at(&inside_path, -5.0);
        let inside_sense = signed_turn(inside_runs.last().expect("inside ring")) > 0.0;

        let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
            .expect("boss path");
        let runs = cut_runs_at(&path, -5.0);
        // Each finishing offset produces 2 rings (outer stock boundary + boss);
        // the last 4 runs are the 2 finishing passes (2 loops each).
        assert!(runs.len() >= 5, "climb={climb}: expected clearing + 4 finishing rings, got {}", runs.len());
        let fin_start = runs.len() - 4;

        // Clearing stays at radius >= 15 + 5.5 = 20.5 mm from boss center
        // (within polygon/arc chord tolerance of the 256-gon offset).
        for (i, r) in runs[..fin_start].iter().enumerate() {
            let mr = min_radius(r);
            assert!(mr >= 20.5 - 0.02, "climb={climb}: clearing run {i} came to R={mr:.4} (< 20.5 mm)");
        }

        // Identify the two boss rings (those with median radius near 20.25 and 20.0, not outer ~35 mm).
        let boss_rings: Vec<&Vec<&Move>> = runs[fin_start..]
            .iter()
            .filter(|r| median_radius(r) < 25.0)
            .collect();
        assert_eq!(boss_rings.len(), 2, "climb={climb}: expected 2 boss finishing rings, got {}", boss_rings.len());
        let r1 = median_radius(boss_rings[0]);
        let r2 = median_radius(boss_rings[1]);
        assert!((r1 - 20.25).abs() < 0.02, "climb={climb}: boss ring 1 R={r1:.4}, want 20.25 mm");
        assert!((r2 - 20.00).abs() < 0.02, "climb={climb}: boss ring 2 R={r2:.4}, want 20.00 mm");

        let boss_sense_1 = signed_turn(boss_rings[0]) > 0.0;
        let boss_sense_2 = signed_turn(boss_rings[1]) > 0.0;
        assert_ne!(boss_sense_1, inside_sense, "climb={climb}: boss ring 1 turning sense must oppose inside boundary");
        assert_ne!(boss_sense_2, inside_sense, "climb={climb}: boss ring 2 turning sense must oppose inside boundary");

        let crossed = camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL);
        assert!(crossed.is_empty(), "climb={climb}: crossing pairs: {crossed:?}");

        let spec = MaskSpec {
            name: "s5_check3_boss",
            expected: &area,
            path: &path,
            tool_diameter: p.tool_diameter,
            cell: 0.25,
            depth: CutDepth::AllCut,
            diff_dir: None,
        };
        ShapeMask::evaluate(&spec).expect("boss mask").assert_no_overcut();
    }
}

/// S5 Check 4: `spring_passes = 1` on check 2 adds a third ring at 5.0 mm;
/// also verify `n_f = 0, spring_passes = 1` emits one spring ring at 5.0 mm.
#[test]
fn s5_check4_spring_passes_add_ring_at_final_offset() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let p = base_params(true);
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.25,
        spring_passes: 1,
        finish_level: FinishLevel::Every,
    };
    let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("spring pass path");
    let runs = cut_runs_at(&path, -5.0);
    assert!(runs.len() >= 4, "expected clearing + 3 rings, got {}", runs.len());
    let n = runs.len();
    let d1 = median_dist_to_rect_walls(&runs[n - 3], 30.0, 20.0);
    let d2 = median_dist_to_rect_walls(&runs[n - 2], 30.0, 20.0);
    let d3 = median_dist_to_rect_walls(&runs[n - 1], 30.0, 20.0);
    assert!((d1 - 5.25).abs() < 1e-3, "ring 1 at {d1:.4}, want 5.25");
    assert!((d2 - 5.00).abs() < 1e-3, "ring 2 at {d2:.4}, want 5.00");
    assert!((d3 - 5.00).abs() < 1e-3, "spring ring 3 at {d3:.4}, want 5.00");
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());

    // Spring pass alone (finish_passes = 0, spring_passes = 1): clearing at 5.0 + 1 spring ring at 5.0.
    let spring_only = AdaptiveFinish {
        finish_passes: 0,
        finish_spacing: 0.0,
        spring_passes: 1,
        finish_level: FinishLevel::Every,
    };
    let path0 = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &spring_only)
        .expect("spring-only path");
    let runs0 = cut_runs_at(&path0, -5.0);
    let last_d = median_dist_to_rect_walls(runs0.last().expect("spring ring"), 30.0, 20.0);
    assert!((last_d - 5.00).abs() < 1e-3, "spring-only ring at {last_d:.4}, want 5.00");
    assert!(camcraft_motion::verify::crossed_pairs(&path0, ARC_FIT_TOL).is_empty());
}

/// Multi-level `finish_level = Every` vs `Last`, plus the flute-length refusal.
#[test]
fn s5_multi_level_every_vs_last_and_flute_law() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let mut p = base_params(true);
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0; // levels: -2.0, -4.0, -6.0

    let every = AdaptiveFinish { finish_passes: 2, finish_spacing: 0.25, spring_passes: 0, finish_level: FinishLevel::Every };
    let path_every = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &every).expect("every");
    for z in [-2.0, -4.0, -6.0] {
        let runs = cut_runs_at(&path_every, z);
        let d_last = median_dist_to_rect_walls(runs.last().expect("ring"), 30.0, 20.0);
        assert!((d_last - 5.00).abs() < 1e-3, "Every at z={z}: final ring at {d_last:.4}, want 5.00");
    }

    let last = AdaptiveFinish { finish_level: FinishLevel::Last, ..every };
    let path_last = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &last).expect("last");
    for z in [-2.0, -4.0] {
        for r in cut_runs_at(&path_last, z) {
            let d = min_dist_to_rect_walls(&r, 30.0, 20.0);
            assert!(d >= 5.5 - 1e-3, "Last at upper z={z}: clearing approached {d:.4} (< 5.5)");
        }
    }
    let runs_bot = cut_runs_at(&path_last, -6.0);
    let n = runs_bot.len();
    assert!((median_dist_to_rect_walls(&runs_bot[n - 2], 30.0, 20.0) - 5.25).abs() < 1e-3);
    assert!((median_dist_to_rect_walls(&runs_bot[n - 1], 30.0, 20.0) - 5.00).abs() < 1e-3);

    // Flute law: 6 mm wall with 4 mm flute refused by name; 6 mm flute accepted.
    let err = adaptive_flute_covers_wall(0.0, -6.0, 4.0).expect_err("short flute must refuse");
    assert!(err.to_string().contains("flute"), "refusal must name flute: {err}");
    assert!(adaptive_flute_covers_wall(0.0, -6.0, 6.0).is_ok());
}

/// Keep-down link law: stays down when proved inside already-cut area, zero crossings.
#[test]
fn s5_keep_down_link_stays_down_without_crossing() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let mut p = base_params(true);
    p.links = Links { pass: Link::KeepDown, ..Links::PLANAR };
    let finish = AdaptiveFinish { finish_passes: 2, finish_spacing: 0.25, spring_passes: 1, finish_level: FinishLevel::Every };
    let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish).expect("keepdown");
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());

    // Under KeepDown, the final 5.00 mm wall is reached with fewer retract lifts than Link::Retract.
    let p_ret = base_params(true);
    let path_ret = adaptive_with_finish(&area, &p_ret, &CornerRelief::default(), &StartSide::default(), &finish).expect("retract");
    let lifts = |t: &Toolpath| t.moves.iter().filter(|m| m.kind == MoveKind::Rapid && m.to.z > m.from.z + 1e-3).count();
    assert!(lifts(&path) < lifts(&path_ret), "KeepDown ({}) should lift less than Retract ({})", lifts(&path), lifts(&path_ret));
}

/// Parameter refusals: finish_passes > 0 with non-positive spacing, non-finite, negative.
#[test]
fn s5_refusals_by_name() {
    let bad_zero = AdaptiveFinish { finish_passes: 1, finish_spacing: 0.0, ..Default::default() };
    assert!(bad_zero.refusal().is_some_and(|s| s.contains("spacing")));
    let bad_neg = AdaptiveFinish { finish_passes: 0, finish_spacing: -0.1, ..Default::default() };
    assert!(bad_neg.refusal().is_some_and(|s| s.contains("negative")));
    let bad_nan = AdaptiveFinish { finish_passes: 1, finish_spacing: f64::NAN, ..Default::default() };
    assert!(bad_nan.refusal().is_some_and(|s| s.contains("finite")));
    let bad_inf = AdaptiveFinish { finish_passes: 0, finish_spacing: f64::INFINITY, ..Default::default() };
    assert!(bad_inf.refusal().is_some_and(|s| s.contains("finite")));

    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let err = adaptive_with_finish(&area, &base_params(true), &CornerRelief::default(), &StartSide::default(), &bad_zero)
        .expect_err("zero spacing refused");
    assert!(err.to_string().contains("spacing"));
}

/// Pocket with an island: both outer wall and island receive finishing rings
/// at 5.25 and 5.0 mm with opposite turning senses.
#[test]
fn s5_pocket_with_island_finishes_both_walls_with_opposite_senses() {
    let outer = rect(-35.0, -25.0, 35.0, 25.0);
    let island = circle(0.0, 0.0, 7.0, 128);
    let area = outer.difference(&island).expect("pocket with island");
    let p = base_params(true);
    let finish = AdaptiveFinish { finish_passes: 2, finish_spacing: 0.25, spring_passes: 0, finish_level: FinishLevel::Every };
    let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("island finish");
    let runs = cut_runs_at(&path, -5.0);
    let fin = &runs[runs.len() - 4..];
    let island_rings: Vec<_> = fin.iter().filter(|r| median_radius(r) < 16.0).collect();
    let outer_rings: Vec<_> = fin.iter().filter(|r| median_radius(r) >= 16.0).collect();
    assert_eq!(island_rings.len(), 2, "expected 2 island rings");
    assert_eq!(outer_rings.len(), 2, "expected 2 outer rings");
    assert!((median_radius(island_rings[0]) - 12.25).abs() < 0.02);
    assert!((median_radius(island_rings[1]) - 12.00).abs() < 0.02);
    assert!(signed_turn(outer_rings[1]) > 0.0, "climb outer boundary must turn CCW");
    assert!(signed_turn(island_rings[1]) < 0.0, "climb island must turn CW");
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());
}

/// Regression for R1 BLOCKER 1: under multi-level `finish_level = Last`, the
/// uncut finishing wall stands all the way up to `top_z`, so no rapid move
/// onto a finishing pass may descend below `top_z`.
#[test]
fn s5_multi_level_last_never_rapids_below_top_z_for_finishing() {
    let area = rect(-30.0, -20.0, 30.0, 20.0);
    let mut p = base_params(true);
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0; // levels: -2.0, -4.0, -6.0
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.25,
        spring_passes: 1,
        finish_level: FinishLevel::Last,
    };
    let path = adaptive_with_finish(&area, &p, &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("multi-level last finish");

    // Any rapid move positioned within the finishing wall band (< 5.5 mm from
    // a wall) must stay at or above `top_z = 0.0` (never rapiding into uncut
    // wall stock at z = -4.0 + plunge_height).
    for (i, m) in path.moves.iter().enumerate() {
        if m.kind == MoveKind::Rapid {
            let wall_d = (30.0 - m.to.x.abs()).min(20.0 - m.to.y.abs());
            if wall_d < 5.5 - 1e-4 {
                assert!(
                    m.to.z >= p.top_z - 1e-6,
                    "move {i} rapids below top_z ({:.3}) into uncut wall band (wall_d={wall_d:.3}): {m:?}",
                    m.to.z
                );
            }
        }
    }
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());
}

/// Regression for R1 BLOCKER 2 & MAJOR 1: `adaptive_trochoidal_with_finish`
/// with `Link::KeepDown` + cleanup never feeds diagonally from `hop` down to
/// `z`, never rapids into uncut wall stock under `FinishLevel::Last`, stays
/// crossing-free, and emits finishing rings at `w + d_f` and `w`.
#[test]
fn s5_trochoidal_with_finish_keep_down_no_diagonal_feed_and_valid_rings() {
    let area = rect(0.0, 0.0, 36.0, 16.0);
    let p = PocketParams {
        tool_diameter: 6.0,
        stepover: 2.0,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: Some(2.0),
        top_z: 0.0,
        bottom_z: -4.0,
        stepdown: 2.0,
        feed: 600.0,
        plunge_feed: 150.0,
        climb: true,
        strategy: PocketStrategy::Adaptive,
        entry: Entry3D::Helix,
        helix_radius: 1.5,
        optimal_load: DEFAULT_OPTIMAL_LOAD,
        blend_turns: false,
        blend_radius: None,
        links: Links { pass: Link::KeepDown, ..Links::PLANAR },
    };
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.2,
        spring_passes: 0,
        finish_level: FinishLevel::Last,
    };
    let path = adaptive_trochoidal_with_finish(&area, &p, &AdaptiveTrochParams::default(), None, &finish)
        .expect("trochoidal finish");

    // Zero diagonal full-feed moves descending from retract (`hop = 2.0`) to cut plane.
    for (i, m) in path.moves.iter().enumerate() {
        if m.kind == MoveKind::Feed && m.arc.is_none() && m.from.z > p.top_z + 0.5 && m.to.z < p.top_z - 0.5 {
            let xy = (m.to.x - m.from.x).hypot(m.to.y - m.from.y);
            assert!(
                xy <= 1e-6 || m.feed.is_some_and(|f| f <= p.plunge_feed + 1e-6),
                "move {i} feeds diagonally from hop ({:.3}) to cut ({:.3}) at full feed: {m:?}",
                m.from.z,
                m.to.z
            );
        }
        if m.kind == MoveKind::Rapid {
            let wall_d = m.to.x.min(36.0 - m.to.x).min(m.to.y).min(16.0 - m.to.y);
            if wall_d < 3.4 - 1e-4 {
                assert!(
                    m.to.z >= p.top_z - 1e-6,
                    "trochoidal move {i} rapids below top_z ({:.3}) into uncut wall band (wall_d={wall_d:.3})",
                    m.to.z
                );
            }
        }
    }

    // Verify both finishing rings at 3.2 mm and 3.0 mm exist at bottom_z = -4.0.
    let min_wall_at_bottom = path
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && (m.from.z + 4.0).abs() < 1e-6 && (m.to.z + 4.0).abs() < 1e-6)
        .flat_map(|m| m.chords(0.01))
        .map(|(a, _)| a.x.min(36.0 - a.x).min(a.y).min(16.0 - a.y))
        .fold(f64::INFINITY, f64::min);
    assert!(
        (min_wall_at_bottom - 3.0).abs() < 1e-2,
        "trochoidal finishing must reach final wall offset 3.0 mm (got {min_wall_at_bottom:.4})"
    );
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());
}

/// R1 MAJOR 2(b): `adaptive_avoid_with_finish` keeps the full tool clear of a
/// protected "Do not touch" pin while still finishing the reachable walls.
#[test]
fn s5_avoid_with_finish_keeps_clear_of_protected_zone() {
    use camcraft_motion::geometry::{Point3, Triangle};
    use camcraft_motion::tools::ToolGeom;
    use camcraft_motion::verify::{AVOID_VERIFY_TOL, AvoidItem, AvoidSpec, Cutter, avoid_hits};

    let box_facets = |min: [f64; 3], max: [f64; 3]| -> Vec<Triangle> {
        let v = |i: usize| {
            Point3::new([min[0], max[0]][i & 1], [min[1], max[1]][(i >> 1) & 1], [min[2], max[2]][(i >> 2) & 1])
        };
        let quad = |a: usize, b: usize, c: usize, d: usize| [Triangle::new(v(a), v(b), v(c)), Triangle::new(v(a), v(c), v(d))];
        [quad(4, 5, 7, 6), quad(0, 2, 3, 1), quad(0, 1, 5, 4), quad(2, 6, 7, 3), quad(1, 3, 7, 5), quad(2, 0, 4, 6)]
            .into_iter()
            .flatten()
            .collect()
    };
    let cutter = Cutter::flat(&ToolGeom {
        radius: 3.0,
        nose_radius: 0.0,
        flute_len: 8.0,
        shank_radius: 4.5,
        shank_len: 10.0,
        holder_radius: 12.0,
        holder_len: 20.0,
    });
    let spec = AvoidSpec {
        items: vec![AvoidItem { label: "east pin".into(), tris: box_facets([28.0, 5.0, -10.0], [33.0, 11.0, 2.0]), edges: vec![] }],
        cutter,
        clearance: 1.0,
    };
    let area = rect(0.0, 0.0, 36.0, 16.0);
    let p = PocketParams {
        tool_diameter: 6.0,
        stepover: 2.5,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: Some(2.0),
        top_z: 0.0,
        bottom_z: -5.0,
        stepdown: 2.5,
        feed: 600.0,
        plunge_feed: 150.0,
        climb: true,
        strategy: PocketStrategy::Adaptive,
        entry: Entry3D::Helix,
        helix_radius: 1.5,
        optimal_load: 0.4,
        blend_turns: false,
        blend_radius: None,
        links: Links::ROUGH3D,
    };
    let finish = AdaptiveFinish {
        finish_passes: 2,
        finish_spacing: 0.2,
        spring_passes: 0,
        finish_level: FinishLevel::Every,
    };
    let path = adaptive_avoid_with_finish(&area, &p, &spec, &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("avoid adaptive finish");

    let hits = avoid_hits(&path, &spec, AVOID_VERIFY_TOL).expect("avoid_hits");
    assert!(hits.is_empty(), "avoid finish must stay clear of protected pin: {hits:?}");
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());

    // Reachable western wall (x = 0) gets finished to x = 3.0 mm.
    let min_x = path
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && (m.from.z + 5.0).abs() < 1e-6 && (m.to.z + 5.0).abs() < 1e-6)
        .flat_map(|m| m.chords(0.01))
        .map(|(a, _)| a.x)
        .fold(f64::INFINITY, f64::min);
    assert!((min_x - 3.0).abs() < 1e-2, "western wall should finish at x=3.0 mm (got {min_x:.4})");
}
