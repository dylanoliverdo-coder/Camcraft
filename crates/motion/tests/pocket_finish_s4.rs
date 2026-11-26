//! Journeyman spec S4: 2D Pocket wall finishing passes with their own direction.
//! Proves all synthetic motion checks of the spec:
//! 1. Defaults invariance across strategies (byte-for-byte identical to pre-S4 pocket).
//! 2. 50 x 40 mm pocket with 10 x 10 mm island, 6 mm end mill, one level, n_f = 2,
//!    d_f = 0.1, finish_direction = same: clearing >= 3.2 mm from every wall;
//!    two rings round outer boundary and two round island at 3.1 and 3.0 mm;
//!    swept footprint matches pocket area (`shape_mask`: no overcut, no leftover).
//! 3. Same with climb = true and finish_direction = conventional: finishing rings
//!    reverse turning sense round each wall while clearing passes stay unchanged.
//! 4. Circular pocket of radius 20 mm, climb = true, n_f = 1: wall ring is all
//!    arcs of one sense (G3 under CW spindle); conventional reverses to all G2.
//! 5. spring_passes = 1: one more ring at 3.0 mm round each wall (both with n_f = 2
//!    and standalone n_f = 0).
//!
//! Also covers multi-level Every vs Last, flute-length refusal, KeepDown links,
//! AvoidSpec keep-outs, and parameter refusals.
//!
//! Gate map: covers camcraft_motion::machining2d (pocket_finish, pocket_finish_avoid,
//! pocket, levels, ladder_links).
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991;
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007; no vendor
//! parameters, text or code.

mod common;

use camcraft_motion::cl::{Move, MoveKind, Toolpath};
use camcraft_motion::entry::Entry3D;
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::links::{Link, Links};
use camcraft_motion::load::DEFAULT_OPTIMAL_LOAD;
use camcraft_motion::machining2d::{
    ARC_FIT_TOL, CornerRelief, FinishDirection, FinishLevel, MAX_FINISH_SPACING, MAX_WALL_PASSES,
    PassStart, PocketFinish, PocketParams, PocketStrategy, StartSide, pocket_avoid_with_finish,
    pocket_flute_covers_wall, pocket_started_on, pocket_with_finish,
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

fn circle(cx: f64, cy: f64, r: f64, n: usize) -> Region {
    let pts = (0..n)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / n as f64;
            Point2::new(cx + r * a.cos(), cy + r * a.sin())
        })
        .collect();
    Region::new(vec![lp(pts)]).expect("circle region")
}

/// Spec S4 Check 2/3/5 geometry: 50 x 40 mm pocket with 10 x 10 mm island in the middle.
fn pocket_with_island() -> Region {
    let outer = rect(-25.0, -20.0, 25.0, 20.0);
    let island = rect(-5.0, -5.0, 5.0, 5.0);
    outer.difference(&island).expect("pocket minus island")
}

fn params(strategy: PocketStrategy, climb: bool) -> PocketParams {
    PocketParams {
        tool_diameter: 6.0,
        stepover: 2.0,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: Some(2.0),
        top_z: 0.0,
        bottom_z: -5.0,
        stepdown: 5.0,
        feed: 900.0,
        plunge_feed: 250.0,
        climb,
        strategy,
        entry: Entry3D::Helix,
        helix_radius: 1.5,
        optimal_load: DEFAULT_OPTIMAL_LOAD,
        blend_turns: false,
        blend_radius: None,
        links: Links::PLANAR,
    }
}

/// Split in-plane feed moves at `z` into contiguous runs.
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

/// Distance from `p` to the nearest wall of the 50 x 40 outer / 10 x 10 island pocket.
fn dist_to_pocket_island_walls(p: Point2) -> f64 {
    let d_outer = (25.0 - p.x.abs()).min(20.0 - p.y.abs());
    let dx = (p.x.abs() - 5.0).max(0.0);
    let dy = (p.y.abs() - 5.0).max(0.0);
    let d_island = dx.hypot(dy);
    d_outer.min(d_island)
}

fn dist_to_outer_walls(p: Point2) -> f64 {
    (25.0 - p.x.abs()).min(20.0 - p.y.abs())
}

fn dist_to_island_walls(p: Point2) -> f64 {
    let dx = (p.x.abs() - 5.0).max(0.0);
    let dy = (p.y.abs() - 5.0).max(0.0);
    dx.hypot(dy)
}

/// Minimum distance from true curve points (`chords(0.001)` endpoints, avoiding
/// chord-flattening sagitta bias on curved corners) to any wall.
fn min_wall_dist(run: &[&Move]) -> f64 {
    let mut best = f64::INFINITY;
    for m in run {
        for (a, b) in m.chords(0.001) {
            for p in [a, b] {
                best = best.min(dist_to_pocket_island_walls(p));
            }
        }
    }
    best
}

/// Median distance from true curve points (`chords(0.001)` endpoints) to `f`.
fn median_of(run: &[&Move], f: impl Fn(Point2) -> f64) -> f64 {
    let mut ds: Vec<f64> = run
        .iter()
        .flat_map(|m| m.chords(0.001))
        .map(|(a, _)| f(a))
        .collect();
    ds.sort_by(f64::total_cmp);
    ds[ds.len() / 2]
}

fn signed_turn(run: &[&Move]) -> f64 {
    run.iter()
        .flat_map(|m| m.chords(0.05))
        .map(|(a, b)| a.x * b.y - a.y * b.x)
        .sum()
}

fn max_abs_coord(run: &[&Move]) -> f64 {
    run.iter()
        .flat_map(|m| m.chords(0.05))
        .flat_map(|(a, b)| [a, b])
        .map(|p| p.x.abs().max(p.y.abs()))
        .fold(0.0_f64, f64::max)
}

/// S4 Check 1: defaults keep every pocket strategy's toolpath byte-for-byte identical.
#[test]
fn s4_check1_defaults_match_existing_pocket_toolpaths() {
    let area = pocket_with_island();
    let relief = CornerRelief::default();
    let side = StartSide::from(PassStart::Auto);
    for strat in [
        PocketStrategy::Zigzag,
        PocketStrategy::Waterline,
        PocketStrategy::Auto,
        PocketStrategy::OffsetRings,
    ] {
        for climb in [true, false] {
            let p = params(strat, climb);
            let plain = pocket_started_on(&area, &p, &[], &relief, &side).expect("plain pocket");
            let with_def =
                pocket_with_finish(&area, &p, &[], &relief, &side, &PocketFinish::default()).expect("default finish");
            assert_eq!(with_def, plain, "default PocketFinish must match pre-S4 path for {strat:?} climb={climb}");
        }
    }
}

/// S4 Check 2: 50 x 40 mm pocket with 10 x 10 mm island, 6 mm end mill, one level,
/// n_f = 2, d_f = 0.1, finish_direction = same:
/// - clearing stays >= 3.2 mm from every wall;
/// - two rings round outer boundary and two round island at 3.1 and 3.0 mm;
/// - swept footprint matches pocket area (`shape_mask`: no overcut, no leftover).
#[test]
fn s4_check2_pocket_with_island_offsets_and_shape_mask() {
    let area = pocket_with_island();
    let finish = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };

    for strat in [PocketStrategy::Zigzag, PocketStrategy::Waterline, PocketStrategy::OffsetRings] {
        let p = params(strat, true);
        let path = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &finish)
            .expect("pocket finish");

        let runs = cut_runs_at(&path, -5.0);
        assert!(runs.len() >= 5, "{strat:?}: expected clearing + 4 finishing rings, got {}", runs.len());
        let fin_start = runs.len() - 4;
        let (clearing, fin) = (&runs[..fin_start], &runs[fin_start..]);

        // Clearing passes stay >= 3.2 mm from every wall (outer + island;
        // within Clipper's 5 um arc-chord tolerance on the 4 rounded island corners).
        for (i, r) in clearing.iter().enumerate() {
            let d = min_wall_dist(r);
            assert!(d >= 3.2 - 0.006, "{strat:?}: clearing run {i} approached wall to {d:.4} mm (< 3.2 mm)");
        }

        // Split finishing rings into outer boundary rings vs island rings.
        let outer_rings: Vec<_> = fin.iter().filter(|r| max_abs_coord(r) > 15.0).collect();
        let island_rings: Vec<_> = fin.iter().filter(|r| max_abs_coord(r) <= 15.0).collect();
        assert_eq!(outer_rings.len(), 2, "{strat:?}: expected 2 outer finishing rings");
        assert_eq!(island_rings.len(), 2, "{strat:?}: expected 2 island finishing rings");

        let o1 = median_of(outer_rings[0], dist_to_outer_walls);
        let o2 = median_of(outer_rings[1], dist_to_outer_walls);
        let i1 = median_of(island_rings[0], dist_to_island_walls);
        let i2 = median_of(island_rings[1], dist_to_island_walls);
        assert!((o1 - 3.1).abs() < 1e-3, "{strat:?}: outer ring 1 at {o1:.4}, want 3.1");
        assert!((o2 - 3.0).abs() < 1e-3, "{strat:?}: outer ring 2 at {o2:.4}, want 3.0");
        assert!((i1 - 3.1).abs() < 1e-3, "{strat:?}: island ring 1 at {i1:.4}, want 3.1");
        assert!((i2 - 3.0).abs() < 1e-3, "{strat:?}: island ring 2 at {i2:.4}, want 3.0");

        // Climb milling: outer wall CCW (> 0), island CW (< 0).
        assert!(signed_turn(outer_rings[0]) > 0.0, "{strat:?}: climb outer ring 1 must be CCW");
        assert!(signed_turn(outer_rings[1]) > 0.0, "{strat:?}: climb outer ring 2 must be CCW");
        assert!(signed_turn(island_rings[0]) < 0.0, "{strat:?}: climb island ring 1 must be CW");
        assert!(signed_turn(island_rings[1]) < 0.0, "{strat:?}: climb island ring 2 must be CW");

        assert!(
            camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty(),
            "{strat:?}: crossing pairs found"
        );

        // Swept footprint matches the 50 x 40 pocket with 10 x 10 island (`shape_mask`),
        // allowing only the 4 sharp inside outer corners a 6 mm cutter cannot physically reach.
        let reachable = area.offset(-3.0).expect("erode").offset(3.0).expect("dilate");
        let corners = area.difference(&reachable).expect("corner spandrels");
        let spec = MaskSpec {
            name: "s4_check2_50x40_island",
            expected: &area,
            path: &path,
            tool_diameter: p.tool_diameter,
            cell: 0.25,
            depth: CutDepth::AllCut,
            diff_dir: None,
        };
        let mask = ShapeMask::evaluate(&spec).expect("shape_mask evaluate");
        mask.assert_no_overcut();
        mask.assert_bounded_leftover(12.0, Some(&corners));
    }
}

/// S4 Check 3: climb = true with finish_direction = conventional reverses every
/// finishing ring's turning sense while clearing passes remain identical.
#[test]
fn s4_check3_conventional_finish_direction_reverses_only_finishing_rings() {
    let area = pocket_with_island();
    let p = params(PocketStrategy::Waterline, true);
    let fin_same = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    let fin_conv = PocketFinish { finish_direction: FinishDirection::Conventional, ..fin_same };

    let path_same = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &fin_same)
        .expect("same");
    let path_conv = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &fin_conv)
        .expect("conventional");

    let runs_same = cut_runs_at(&path_same, -5.0);
    let runs_conv = cut_runs_at(&path_conv, -5.0);
    let fin_s = runs_same.len() - 4;
    let fin_c = runs_conv.len() - 4;
    assert_eq!(fin_s, fin_c, "clearing run count must not change");

    // Clearing passes are identical move-for-move.
    for i in 0..fin_s {
        assert_eq!(runs_same[i].len(), runs_conv[i].len(), "clearing run {i} length changed");
        for (a, b) in runs_same[i].iter().zip(runs_conv[i].iter()) {
            assert_eq!(a.from, b.from, "clearing run {i} move start changed");
            assert_eq!(a.to, b.to, "clearing run {i} move end changed");
            assert_eq!(a.arc, b.arc, "clearing run {i} arc changed");
        }
    }

    // Finishing rings reverse sense on BOTH outer wall (CW) and island (CCW).
    let outer_conv: Vec<_> = runs_conv[fin_c..].iter().filter(|r| max_abs_coord(r) > 15.0).collect();
    let island_conv: Vec<_> = runs_conv[fin_c..].iter().filter(|r| max_abs_coord(r) <= 15.0).collect();
    assert_eq!(outer_conv.len(), 2);
    assert_eq!(island_conv.len(), 2);
    assert!(signed_turn(outer_conv[0]) < 0.0, "conventional outer ring 1 must be CW");
    assert!(signed_turn(outer_conv[1]) < 0.0, "conventional outer ring 2 must be CW");
    assert!(signed_turn(island_conv[0]) > 0.0, "conventional island ring 1 must be CCW");
    assert!(signed_turn(island_conv[1]) > 0.0, "conventional island ring 2 must be CCW");

    // Symmetric check: climb = false with finish_direction = Climb -> outer CCW, island CW.
    let p_conv = params(PocketStrategy::Waterline, false);
    let fin_climb = PocketFinish { finish_direction: FinishDirection::Climb, ..fin_same };
    let path_climb = pocket_with_finish(&area, &p_conv, &[], &CornerRelief::default(), &StartSide::default(), &fin_climb)
        .expect("climb override");
    let runs_cl = cut_runs_at(&path_climb, -5.0);
    let outer_cl: Vec<_> = runs_cl[runs_cl.len() - 4..].iter().filter(|r| max_abs_coord(r) > 15.0).collect();
    let island_cl: Vec<_> = runs_cl[runs_cl.len() - 4..].iter().filter(|r| max_abs_coord(r) <= 15.0).collect();
    assert!(signed_turn(outer_cl[1]) > 0.0, "Climb override outer ring must be CCW");
    assert!(signed_turn(island_cl[1]) < 0.0, "Climb override island ring must be CW");
    assert!(camcraft_motion::verify::crossed_pairs(&path_conv, ARC_FIT_TOL).is_empty());
    assert!(camcraft_motion::verify::crossed_pairs(&path_climb, ARC_FIT_TOL).is_empty());
}

/// S4 Check 4: circular pocket of radius 20 mm, climb = true, n_f = 1:
/// the wall ring is all arcs of one sense (G3 / ccw = true under CW spindle);
/// finish_direction = conventional produces all G2 / ccw = false arcs.
#[test]
fn s4_check4_circular_pocket_wall_ring_is_all_g3_arcs() {
    let disk = circle(0.0, 0.0, 20.0, 128);
    let p = params(PocketStrategy::Auto, true);
    let fin_g3 = PocketFinish {
        finish_passes: 1,
        finish_spacing: 0.2,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    let path_g3 = pocket_with_finish(&disk, &p, &[], &CornerRelief::default(), &StartSide::default(), &fin_g3)
        .expect("circular pocket climb");
    let runs_g3 = cut_runs_at(&path_g3, -5.0);
    let wall_ring = runs_g3.last().expect("wall ring");
    assert!(!wall_ring.is_empty(), "wall ring must have moves");
    for (i, m) in wall_ring.iter().enumerate() {
        let arc = m.arc.unwrap_or_else(|| panic!("wall ring move {i} must be an arc, got line: {m:?}"));
        assert!(arc.ccw, "climb circular wall ring move {i} must be G3 (ccw=true): {m:?}");
        let r = (m.to.x - arc.center.x).hypot(m.to.y - arc.center.y);
        assert!((r - 17.0).abs() < 0.02, "wall ring move {i} radius {r:.4}, want 17.0 mm");
    }
    assert!(camcraft_motion::verify::crossed_pairs(&path_g3, ARC_FIT_TOL).is_empty());

    // Conventional finish direction on the same circular pocket: all G2 (ccw = false) arcs.
    let fin_g2 = PocketFinish { finish_direction: FinishDirection::Conventional, ..fin_g3 };
    let path_g2 = pocket_with_finish(&disk, &p, &[], &CornerRelief::default(), &StartSide::default(), &fin_g2)
        .expect("circular pocket conventional");
    let runs_g2 = cut_runs_at(&path_g2, -5.0);
    let wall_g2 = runs_g2.last().expect("conventional wall ring");
    assert!(!wall_g2.is_empty());
    for (i, m) in wall_g2.iter().enumerate() {
        let arc = m.arc.unwrap_or_else(|| panic!("conventional wall ring move {i} must be an arc: {m:?}"));
        assert!(!arc.ccw, "conventional circular wall ring move {i} must be G2 (ccw=false): {m:?}");
    }
    assert!(camcraft_motion::verify::crossed_pairs(&path_g2, ARC_FIT_TOL).is_empty());
}

/// S4 Check 5: spring_passes = 1 adds one more ring at 3.0 mm round each wall
/// (both after n_f = 2 finishing passes and standalone n_f = 0, spring_passes = 1).
#[test]
fn s4_check5_spring_passes_add_ring_at_final_offset() {
    let area = pocket_with_island();
    let p = params(PocketStrategy::Waterline, true);
    let finish = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 1,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    let path = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("spring passes");
    let runs = cut_runs_at(&path, -5.0);
    // 2 finishing passes * 2 walls + 1 spring pass * 2 walls = 6 wall rings.
    assert!(runs.len() >= 7, "expected clearing + 6 wall rings, got {}", runs.len());
    let fin = &runs[runs.len() - 6..];
    let outer: Vec<_> = fin.iter().filter(|r| max_abs_coord(r) > 15.0).collect();
    let island: Vec<_> = fin.iter().filter(|r| max_abs_coord(r) <= 15.0).collect();
    assert_eq!(outer.len(), 3, "2 finishing + 1 spring outer rings");
    assert_eq!(island.len(), 3, "2 finishing + 1 spring island rings");
    assert!((median_of(outer[0], dist_to_outer_walls) - 3.1).abs() < 1e-3);
    assert!((median_of(outer[1], dist_to_outer_walls) - 3.0).abs() < 1e-3);
    assert!((median_of(outer[2], dist_to_outer_walls) - 3.0).abs() < 1e-3);
    assert!((median_of(island[0], dist_to_island_walls) - 3.1).abs() < 1e-3);
    assert!((median_of(island[1], dist_to_island_walls) - 3.0).abs() < 1e-3);
    assert!((median_of(island[2], dist_to_island_walls) - 3.0).abs() < 1e-3);
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());

    // Standalone spring pass (finish_passes = 0, spring_passes = 1) with conventional override.
    let spring_only = PocketFinish {
        finish_passes: 0,
        finish_spacing: 0.0,
        spring_passes: 1,
        finish_direction: FinishDirection::Conventional,
        finish_level: FinishLevel::Every,
    };
    let path0 = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &spring_only)
        .expect("spring-only");
    let runs0 = cut_runs_at(&path0, -5.0);
    let fin0 = &runs0[runs0.len() - 2..];
    let outer0: Vec<_> = fin0.iter().filter(|r| max_abs_coord(r) > 15.0).collect();
    let island0: Vec<_> = fin0.iter().filter(|r| max_abs_coord(r) <= 15.0).collect();
    assert_eq!(outer0.len(), 1);
    assert_eq!(island0.len(), 1);
    assert!((median_of(outer0[0], dist_to_outer_walls) - 3.0).abs() < 1e-3);
    assert!((median_of(island0[0], dist_to_island_walls) - 3.0).abs() < 1e-3);
    assert!(signed_turn(outer0[0]) < 0.0, "conventional spring outer ring must be CW");
    assert!(signed_turn(island0[0]) > 0.0, "conventional spring island ring must be CCW");
    assert!(camcraft_motion::verify::crossed_pairs(&path0, ARC_FIT_TOL).is_empty());
}

/// Multi-level `finish_level = Every` vs `Last`, rapid safety above `top_z`, and flute law.
#[test]
fn s4_multi_level_every_vs_last_rapid_safety_and_flute_law() {
    let area = pocket_with_island();
    let mut p = params(PocketStrategy::Waterline, true);
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0; // levels: -2.0, -4.0, -6.0

    let every = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    let path_every = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &every)
        .expect("every");
    for z in [-2.0, -4.0, -6.0] {
        let runs = cut_runs_at(&path_every, z);
        let d_min = runs.iter().map(|r| min_wall_dist(r)).fold(f64::INFINITY, f64::min);
        assert!((d_min - 3.0).abs() < 0.006, "Every at z={z}: reached {d_min:.4}, want 3.0");
    }

    let last = PocketFinish { finish_level: FinishLevel::Last, spring_passes: 1, ..every };
    let path_last = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &last)
        .expect("last");
    for z in [-2.0, -4.0] {
        for r in cut_runs_at(&path_last, z) {
            let d = min_wall_dist(&r);
            assert!(d >= 3.2 - 0.006, "Last at upper z={z}: approached wall to {d:.4} (< 3.2)");
        }
    }
    let bot_runs = cut_runs_at(&path_last, -6.0);
    let d_bot = bot_runs.iter().map(|r| min_wall_dist(r)).fold(f64::INFINITY, f64::min);
    assert!((d_bot - 3.0).abs() < 0.006, "Last at bottom: reached {d_bot:.4}, want 3.0");

    // Rapid moves inside the uncut finishing wall band (< 3.2 mm from any wall) stay >= top_z.
    for (i, m) in path_last.moves.iter().enumerate() {
        if m.kind == MoveKind::Rapid {
            let wd = dist_to_pocket_island_walls(Point2::new(m.to.x, m.to.y));
            if wd < 3.2 - 1e-4 {
                assert!(
                    m.to.z >= p.top_z - 1e-6,
                    "move {i} rapids below top_z ({:.3}) in uncut wall band (wd={wd:.3}): {m:?}",
                    m.to.z
                );
            }
        }
    }
    assert!(camcraft_motion::verify::crossed_pairs(&path_last, ARC_FIT_TOL).is_empty());

    // Flute law: 6 mm wall with 4 mm flute refused by name; 6 mm flute accepted.
    let err = pocket_flute_covers_wall(0.0, -6.0, 4.0).expect_err("short flute refused");
    assert!(err.to_string().contains("flute"), "refusal must name flute: {err}");
    assert!(pocket_flute_covers_wall(0.0, -6.0, 6.0).is_ok());
}

/// KeepDown links stay down where proved inside cleared area, lifting less than Retract.
#[test]
fn s4_keep_down_links_stay_down_without_crossing() {
    let area = rect(-25.0, -20.0, 25.0, 20.0);
    let mut p_kd = params(PocketStrategy::Waterline, true);
    p_kd.links = Links { pass: Link::KeepDown, ..Links::PLANAR };
    let p_ret = params(PocketStrategy::Waterline, true);
    let finish = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 1,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    let path_kd = pocket_with_finish(&area, &p_kd, &[], &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("keepdown");
    let path_ret = pocket_with_finish(&area, &p_ret, &[], &CornerRelief::default(), &StartSide::default(), &finish)
        .expect("retract");
    let lifts = |t: &Toolpath| t.moves.iter().filter(|m| m.kind == MoveKind::Rapid && m.to.z > m.from.z + 1e-3).count();
    assert!(lifts(&path_kd) < lifts(&path_ret), "KeepDown ({}) should lift less than Retract ({})", lifts(&path_kd), lifts(&path_ret));
    assert!(camcraft_motion::verify::crossed_pairs(&path_kd, ARC_FIT_TOL).is_empty());
}

/// Parameter refusals by name: finish_passes > 0 with spacing <= 0, negative, non-finite, > 10000, > MAX_WALL_PASSES.
#[test]
fn s4_refusals_by_name() {
    let cases = [
        (PocketFinish { finish_passes: 1, finish_spacing: 0.0, ..Default::default() }, "spacing"),
        (PocketFinish { finish_passes: 0, finish_spacing: -0.1, ..Default::default() }, "negative"),
        (PocketFinish { finish_passes: 1, finish_spacing: f64::NAN, ..Default::default() }, "finite"),
        (PocketFinish { finish_passes: 0, finish_spacing: f64::INFINITY, ..Default::default() }, "finite"),
        (PocketFinish { finish_passes: 1, finish_spacing: MAX_FINISH_SPACING + 1.0, ..Default::default() }, "limit"),
        (PocketFinish { finish_passes: MAX_WALL_PASSES + 1, finish_spacing: 0.1, ..Default::default() }, "supported"),
        (PocketFinish { spring_passes: MAX_WALL_PASSES + 1, ..Default::default() }, "supported"),
    ];
    let area = pocket_with_island();
    let p = params(PocketStrategy::Zigzag, true);
    for (fin, needle) in cases {
        let msg = fin.refusal().unwrap_or_else(|| panic!("expected refusal for {fin:?}"));
        assert!(msg.contains(needle), "refusal '{msg}' must mention '{needle}'");
        let err = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &fin)
            .expect_err("generator must refuse");
        assert!(err.to_string().contains(needle), "gen error '{err}' must mention '{needle}'");
    }
}

/// `pocket_avoid_with_finish` stays clear of a protected "Do not touch" pin while finishing walls.
#[test]
fn s4_avoid_with_finish_keeps_clear_of_protected_zone() {
    use camcraft_motion::geometry::{Point3, Triangle};
    use camcraft_motion::tools::ToolGeom;
    use camcraft_motion::verify::{AVOID_VERIFY_TOL, AvoidItem, AvoidSpec, Cutter, avoid_hits};

    let box_facets = |min: [f64; 3], max: [f64; 3]| -> Vec<Triangle> {
        let v = |i: usize| {
            Point3::new([min[0], max[0]][i & 1], [min[1], max[1]][(i >> 1) & 1], [min[2], max[2]][(i >> 2) & 1])
        };
        let q = |a, b, c, d| [Triangle::new(v(a), v(b), v(c)), Triangle::new(v(a), v(c), v(d))];
        [q(4, 5, 7, 6), q(0, 2, 3, 1), q(0, 1, 5, 4), q(2, 6, 7, 3), q(1, 3, 7, 5), q(2, 0, 4, 6)]
            .into_iter()
            .flatten()
            .collect()
    };

    let area = rect(-25.0, -20.0, 25.0, 20.0);
    let p = params(PocketStrategy::Waterline, true);
    let finish = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Conventional,
        finish_level: FinishLevel::Every,
    };
    let pin = box_facets([-3.0, -3.0, -6.0], [3.0, 3.0, 4.0]);
    let avoid = AvoidSpec {
        items: vec![AvoidItem { label: "Pin".into(), tris: pin, edges: Vec::new() }],
        clearance: 1.0,
        cutter: Cutter::flat(&ToolGeom {
            radius: p.tool_diameter * 0.5,
            nose_radius: 0.0,
            flute_len: 15.0,
            shank_radius: p.tool_diameter * 0.5,
            shank_len: 20.0,
            holder_radius: 12.0,
            holder_len: 20.0,
        }),
    };
    let path = pocket_avoid_with_finish(
        &area,
        &p,
        &[],
        &avoid,
        &CornerRelief::default(),
        &StartSide::default(),
        &finish,
    )
    .expect("pocket avoid finish");
    let hits = avoid_hits(&path, &avoid, AVOID_VERIFY_TOL).expect("avoid_hits");
    assert!(hits.is_empty(), "avoid hits: {hits:?}");
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());
}

/// R1 MAJOR 1 regression: `blend_turns = true` with active `PocketFinish` uses
/// `post_blended_prefix_crossing_free` (preserving `#2317` `arc_touch` exemptions
/// on clearing corner blends), producing 0 crossings and 0 spurious arc proof notes.
#[test]
fn s4_r1_blend_turns_with_finish_preserves_blends_and_zero_crossings() {
    let area = pocket_with_island();
    let finish = PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };
    for strat in [PocketStrategy::Zigzag, PocketStrategy::Waterline] {
        let mut p = params(strat, true);
        p.blend_turns = true;
        let path = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &finish)
            .expect("blended pocket with finish");
        assert!(
            camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty(),
            "{strat:?}: blended pocket + finish must have zero crossings"
        );
        let arc_moves = path
            .moves
            .iter()
            .filter(|m| m.kind == MoveKind::Feed && m.arc.is_some() && (m.from.z + 5.0).abs() < 1e-6)
            .count();
        assert!(arc_moves > 0, "{strat:?}: blend_turns must retain corner arcs alongside finishing passes");
        let spurious_proof = path.notes.iter().any(|n| n.contains("arc output:"));
        assert!(!spurious_proof, "{strat:?}: unexpected arc proof refusal note: {:?}", path.notes);
    }
}

/// R1 MAJOR 2 regression: `finish_spacing > tool_diameter` is refused by name
/// in both `pocket_with_finish` and `pocket_avoid_with_finish` (naming both
/// `finish_spacing` and `tool_diameter`).
#[test]
fn s4_r1_finish_spacing_exceeding_tool_diameter_is_refused() {
    use camcraft_motion::geometry::{Point3, Triangle};
    use camcraft_motion::tools::ToolGeom;
    use camcraft_motion::verify::{AvoidItem, AvoidSpec, Cutter};

    let area = rect(-30.0, -30.0, 30.0, 30.0);
    let p = params(PocketStrategy::Waterline, true); // tool_diameter = 6.0
    let too_wide = PocketFinish {
        finish_passes: 1,
        finish_spacing: 6.5,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Every,
    };

    let err_plain = pocket_with_finish(&area, &p, &[], &CornerRelief::default(), &StartSide::default(), &too_wide)
        .expect_err("plain pocket must refuse finish_spacing > tool_diameter");
    let msg_plain = err_plain.to_string();
    assert!(
        msg_plain.contains("finish_spacing") && msg_plain.contains("tool_diameter"),
        "refusal must name both finish_spacing and tool_diameter: {msg_plain}"
    );

    let v = |i: usize| {
        Point3::new([-1.0, 1.0][i & 1], [-1.0, 1.0][(i >> 1) & 1], [-6.0, 4.0][(i >> 2) & 1])
    };
    let q = |a, b, c, d| [Triangle::new(v(a), v(b), v(c)), Triangle::new(v(a), v(c), v(d))];
    let pin: Vec<Triangle> =
        [q(4, 5, 7, 6), q(0, 2, 3, 1), q(0, 1, 5, 4), q(2, 6, 7, 3), q(1, 3, 7, 5), q(2, 0, 4, 6)]
            .into_iter()
            .flatten()
            .collect();
    let avoid = AvoidSpec {
        items: vec![AvoidItem { label: "Pin".into(), tris: pin, edges: Vec::new() }],
        clearance: 1.0,
        cutter: Cutter::flat(&ToolGeom {
            radius: 3.0,
            nose_radius: 0.0,
            flute_len: 15.0,
            shank_radius: 3.0,
            shank_len: 20.0,
            holder_radius: 12.0,
            holder_len: 20.0,
        }),
    };
    let err_avoid = pocket_avoid_with_finish(
        &area,
        &p,
        &[],
        &avoid,
        &CornerRelief::default(),
        &StartSide::default(),
        &too_wide,
    )
    .expect_err("avoid pocket must refuse finish_spacing > tool_diameter");
    let msg_avoid = err_avoid.to_string();
    assert!(
        msg_avoid.contains("finish_spacing") && msg_avoid.contains("tool_diameter"),
        "avoid refusal must name both finish_spacing and tool_diameter: {msg_avoid}"
    );
}

/// R1 MINOR 1 regression: when `finish_level == Last` and the bottom level is
/// completely blocked by an `avoid` obstacle, finishing runs at the lowest clear
/// level and emits an explicit note (never silently dropped).
#[test]
fn s4_r1_avoid_blocked_bottom_level_runs_finishing_on_lowest_clear_level_with_note() {
    use camcraft_motion::geometry::{Point3, Triangle};
    use camcraft_motion::tools::ToolGeom;
    use camcraft_motion::verify::{AVOID_VERIFY_TOL, AvoidItem, AvoidSpec, Cutter, avoid_hits};

    let area = rect(-20.0, -15.0, 20.0, 15.0);
    let mut p = params(PocketStrategy::Waterline, true);
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0; // levels: -2.0, -4.0, -6.0
    let finish = PocketFinish {
        finish_passes: 1,
        finish_spacing: 0.1,
        spring_passes: 0,
        finish_direction: FinishDirection::Same,
        finish_level: FinishLevel::Last,
    };

    // Floor plate blocking ONLY the bottom level z = -6.0 (sits from z = -8.0 to z = -5.5).
    let v = |i: usize| {
        Point3::new([-25.0, 25.0][i & 1], [-20.0, 20.0][(i >> 1) & 1], [-8.0, -5.5][(i >> 2) & 1])
    };
    let q = |a, b, c, d| [Triangle::new(v(a), v(b), v(c)), Triangle::new(v(a), v(c), v(d))];
    let slab: Vec<Triangle> =
        [q(4, 5, 7, 6), q(0, 2, 3, 1), q(0, 1, 5, 4), q(2, 6, 7, 3), q(1, 3, 7, 5), q(2, 0, 4, 6)]
            .into_iter()
            .flatten()
            .collect();
    let avoid = AvoidSpec {
        items: vec![AvoidItem { label: "Subplate".into(), tris: slab, edges: Vec::new() }],
        clearance: 0.5,
        cutter: Cutter::flat(&ToolGeom {
            radius: 3.0,
            nose_radius: 0.0,
            flute_len: 15.0,
            shank_radius: 3.0,
            shank_len: 20.0,
            holder_radius: 12.0,
            holder_len: 20.0,
        }),
    };
    let path = pocket_avoid_with_finish(
        &area,
        &p,
        &[],
        &avoid,
        &CornerRelief::default(),
        &StartSide::default(),
        &finish,
    )
    .expect("avoid finish with blocked bottom");
    assert!(avoid_hits(&path, &avoid, AVOID_VERIFY_TOL).expect("hits").is_empty());
    assert!(camcraft_motion::verify::crossed_pairs(&path, ARC_FIT_TOL).is_empty());
    // z = -6.0 is blocked; z = -4.0 is the lowest clear level and receives the finishing ring at 3.0 mm.
    assert!(cut_runs_at(&path, -6.0).is_empty(), "bottom level -6.0 should be blocked");
    let runs_m4 = cut_runs_at(&path, -4.0);
    assert!(!runs_m4.is_empty(), "level -4.0 must cut");
    let wall_d_m4 = runs_m4
        .iter()
        .flat_map(|r| r.iter())
        .flat_map(|m| m.chords(0.001))
        .map(|(a, _)| (20.0 - a.x.abs()).min(15.0 - a.y.abs()))
        .fold(f64::INFINITY, f64::min);
    assert!((wall_d_m4 - 3.0).abs() < 1e-3, "lowest clear level -4.0 must finish to 3.0 mm (got {wall_d_m4:.4})");
    assert!(
        path.notes.iter().any(|n| n.contains("last-level finishing ran at the lowest clear level")),
        "expected explicit note naming lowest clear finishing level: {:?}",
        path.notes
    );
}
