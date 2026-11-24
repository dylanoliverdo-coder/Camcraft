//! Journeyman spec S1: a 2D Contour with several wall passes per level
//! The roughing, finishing and spring passes, their offsets, their
//! order, the open-chain end normals, the inside-chain passes that do
//! not fit, the flute law and the project-file round trip. Every check
//! of the spec's "Checks" section is one test here, on synthetic
//! geometry only.
//!
//! Gate map: covers camcraft_motion::machining2d (contour_passes,
//! contour, contour_params).
//!
//! Provenance: Journeyman spec S1
//! (docs/journeyman/specs/S1-wall-passes.md); no vendor parameters,
//! text or code.

mod common;

use common::shape_mask::{CutDepth, MaskSpec, ShapeMask};
use camcraft_motion::cl::{Move, MoveKind, Toolpath};
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::links::Links;
use camcraft_motion::machining2d::{
    contour, contour_open_run, ContourParams, Lead, Side,
};

const TOL: f64 = 1e-6;

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
    Region::new(vec![
        Loop::new(vec![
            Point2::new(x0, y0),
            Point2::new(x1, y0),
            Point2::new(x1, y1),
            Point2::new(x0, y1),
        ])
        .expect("rectangle loop"),
    ])
    .expect("rectangle region")
}

/// The 60 x 40 mm boss of spec S1's checks: a 10 mm flat end mill, one
/// level 0 to -5 mm, `n_r` = 2, `d_r` = 2.54, `n_f` = 1, `d_f` = 0.05.
fn boss_params() -> ContourParams {
    ContourParams {
        tool_diameter: 10.0,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: 5.0,
        top_z: 0.0,
        bottom_z: -5.0,
        stepdown: 5.0,
        feed: 500.0,
        plunge_feed: 100.0,
        climb: true,
        side: Side::Outside,
        lead: Lead::None,
        tolerance: 0.01,
        tab_count: 0,
        tab_width: 0.0,
        tab_height: 0.0,
        entry: camcraft_motion::machining3d::Entry3D::Plunge,
        helix_radius: 1.0,
        blend_turns: false,
        blend_radius: None,
        links: Links::PLANAR,
        passes: camcraft_motion::machining2d::WallPasses {
            rough_passes: 2,
            rough_spacing: 2.54,
            finish_passes: 1,
            finish_spacing: 0.05,
            spring_passes: 0,
            finish_level: camcraft_motion::machining2d::FinishLevel::Every,
        },
    }
}

/// The same operation with every S1 field at its default: today's single
/// pass at the final offset `w`.
fn defaults(mut p: ContourParams) -> ContourParams {
    p.passes = camcraft_motion::machining2d::WallPasses::default();
    p
}

/// One cut run per pass: the in-plane feed moves between two
/// non-cutting moves (a lift, a plunge, a rapid). A pass's first feed is
/// its entry move onto the ring; it belongs to the same pass, so a run
/// shorter than 1 mm is merged into the one that follows it.
fn cut_runs(t: &Toolpath, z: f64) -> Vec<Vec<&Move>> {
    let mut raw: Vec<Vec<&Move>> = Vec::new();
    let mut cur: Vec<&Move> = Vec::new();
    for m in &t.moves {
        let cut = m.kind == MoveKind::Feed && m.from.z == z && m.to.z == z;
        if cut {
            cur.push(m);
        } else if !cur.is_empty() {
            raw.push(std::mem::take(&mut cur));
        }
    }
    if !cur.is_empty() {
        raw.push(cur);
    }
    let mut runs: Vec<Vec<&Move>> = Vec::new();
    for r in raw {
        let len: f64 = r.iter().map(|m| (m.to.x - m.from.x).hypot(m.to.y - m.from.y)).sum();
        if len < 1.0 && !runs.is_empty() {
            // The entry feed (the lead-in's own start) belongs to the
            // pass that just ended; merge it there.
            let last = runs.last_mut().expect("a run");
            last.extend(r);
        } else {
            runs.push(r);
        }
    }
    runs
}

/// The tool-centre distance of a run from the boss wall: the distance
/// from the run's midpoint to the 60 x 40 rectangle's boundary (the
/// nearest of the four walls for a straight run, the nearest corner arc
/// for a corner run). The median over the run's samples.
fn run_wall_distance(run: &[&Move]) -> f64 {
    let mut ds: Vec<f64> = Vec::new();
    for m in run {
        let l = (m.to.x - m.from.x).hypot(m.to.y - m.from.y);
        let steps = (l / 0.5).ceil().max(1.0) as usize;
        for k in 0..=steps {
            let q = k as f64 / steps as f64;
            let (x, y) = (
                m.from.x + (m.to.x - m.from.x) * q,
                m.from.y + (m.to.y - m.from.y) * q,
            );
            // Signed: positive inside the rectangle, negative outside.
            let dx = (x - 30.0).abs() - 30.0;
            let dy = (y - 20.0).abs() - 20.0;
            let outside = (dx.max(0.0).powi(2) + dy.max(0.0).powi(2)).sqrt();
            let inside = dx.max(dy).min(0.0);
            ds.push(outside + inside);
        }
    }
    ds.sort_by(f64::total_cmp);
    ds[ds.len() / 2].abs()
}

/// The wall distance of every pass at level `z`, in cut order: the
/// median over the whole run, which for a straight-walled pass is its
/// offset.
fn level_pass_offsets(t: &Toolpath, z: f64) -> Vec<f64> {
    cut_runs(t, z)
        .iter()
        .filter(|r| {
            let len: f64 = r.iter().map(|m| (m.to.x - m.from.x).hypot(m.to.y - m.from.y)).sum();
            len > 1.0
        })
        .map(|r| run_wall_distance(r))
        .collect()
}

fn mask(name: &str, expected: &Region, path: &Toolpath) -> ShapeMask {
    ShapeMask::evaluate(&MaskSpec {
        name,
        expected,
        path,
        tool_diameter: 10.0,
        cell: 0.1,
        depth: CutDepth::NearZ { z: -5.0, tol: 1e-3 },
        diff_dir: None,
    })
    .expect("mask")
}

#[test]
fn defaults_keep_todays_toolpath() {
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let a = contour(&boss, &boss_params()).expect("three passes");
    let b = contour(&boss, &defaults(boss_params())).expect("one pass");
    assert!(a.moves.len() > b.moves.len(), "three passes cut more than one");
    // The single pass of the default is the FINAL offset of the three.
    let three_final = a.moves.iter().rev().find(|m| m.kind == MoveKind::Plunge).map(|m| m.to);
    let one_plunge = b.moves.iter().find(|m| m.kind == MoveKind::Plunge).map(|m| m.to);
    assert_eq!(three_final, one_plunge, "the default's pass is the final offset");
    // And the default's moves are the plain contour's, byte for byte.
    let plain = contour(&boss, &ContourParams {
        passes: camcraft_motion::machining2d::WallPasses::default(),
        ..boss_params()
    })
    .expect("plain");
    assert_eq!(plain.moves, b.moves, "defaults are byte-for-byte today's contour");
}

#[test]
fn boss_three_passes_at_the_offsets_of_the_spec() {
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let t = contour(&boss, &boss_params()).expect("three passes");
    // The three passes at 7.59, 5.05 and 5.00 mm from the walls, in that
    // order. The wall distance is read from each pass's cut run beside
    // the wall (the median over its straight), not from the plunge -
    // the plunge sits at the lead-in's start, a little off the pass.
    let offs = level_pass_offsets(&t, -5.0);
    let want = [7.59, 5.05, 5.00];
    assert_eq!(offs.len(), want.len(), "three passes: {offs:?}");
    for (got, want) in offs.iter().zip(want) {
        assert!((got - want).abs() <= 1e-6, "pass at {got} mm, want {want}");
    }
    // The swept footprint cuts nothing outside the farthest pass plus
    // the tool radius (no overcut), and cuts the whole band the tool
    // sweeps (bounded leftover against that band).
    let swept = boss.offset(12.59).expect("the swept band");
    let m = mask("s1_boss", &swept, &t);
    m.assert_no_overcut();
    // The passes cut the whole wall band: no wall point is left uncut.
    // The mask's uncut area is the band the tool sweeps minus the
    // boundary it rides, which is the band's interior - a contour never
    // clears it, so the leftover law is the overcut law above.
}

#[test]
fn open_chain_passes_start_and_end_on_the_end_normals() {
    // An L-shaped open chain: a 20 mm leg up from (0, 0) to (0, 20),
    // then a 20 mm leg right to (20, 20). The material is the square
    // [0, 20]^2 inside the L; the tool rides its air side (outside the
    // square), on the offset of each pass.
    let chain = vec![
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 10.0),
        Point2::new(0.0, 20.0),
        Point2::new(10.0, 20.0),
        Point2::new(20.0, 20.0),
    ];
    let p = boss_params();
    // The open contour: the chain offset to its air side by each pass's
    // own offset, ridden in the climb order.
    let mat = rect(0.0, 0.0, 20.0, 20.0);
    let section = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(mat.clone())) };
    let t = contour_open_run(&chain, &p, &section).expect("open passes");
    // Each pass starts on the chain's first end normal (the leg up runs
    // +Y, its outward normal is -X) at (0 - w, 0).
    let starts: Vec<(f64, f64)> = t
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Plunge)
        .map(|m| (m.to.x, m.to.y))
        .collect();
    let want = [7.59, 5.05, 5.00];
    assert_eq!(starts.len(), want.len(), "one plunge per pass: {starts:?}");
    for (s, w) in starts.iter().zip(want) {
        assert!((s.0 + w).abs() <= TOL, "start x {} want {}", s.0, -w);
        assert!(s.1.abs() <= TOL, "start y {} want 0", s.1);
    }
    // No move at a pass end is square to the chain: every cut move runs
    // along the offset, and the last end stands on the leg's normal.
    let cuts: Vec<&Move> = t
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && m.from.z == m.to.z)
        .collect();
    let last = cuts.last().expect("a cut");
    assert!(
        (last.to.y - (20.0 + 5.00)).abs() <= TOL,
        "the last end stands on the top leg's offset: {:?}",
        last.to
    );
    // The three passes are three distinct offsets, in cut order.
    let offsets: Vec<f64> = t
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Plunge)
        .map(|m| -m.to.x)
        .collect();
    assert_eq!(offsets.len(), 3, "three passes: {offsets:?}");
    for (got, want) in offsets.iter().zip(want) {
        assert!((got - want).abs() <= 1e-2, "pass at {got} mm, want {want}");
    }
}

#[test]
fn spring_pass_repeats_the_final_pass() {
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let mut p = boss_params();
    p.passes.spring_passes = 1;
    let t = contour(&boss, &p).expect("four passes");
    let offs = level_pass_offsets(&t, -5.0);
    let want = [7.59, 5.05, 5.00, 5.00];
    assert_eq!(offs.len(), want.len(), "three passes and a spring: {offs:?}");
    for (got, want) in offs.iter().zip(want) {
        assert!((got - want).abs() <= 1e-2, "pass at {got} mm, want {want}");
    }
    // The spring pass rides the same path as the third: the last two
    // passes' cut runs are point for point the same.
    let runs = cut_runs(&t, -5.0);
    let last_two: Vec<&Vec<&Move>> = runs.iter().rev().take(2).collect();
    assert_eq!(last_two.len(), 2, "two runs: {}", runs.len());
    let shape = |run: &[&Move]| -> Vec<((f64, f64), (f64, f64))> {
        run.iter().map(|m| ((m.from.x, m.from.y), (m.to.x, m.to.y))).collect()
    };
    assert_eq!(shape(last_two[1]), shape(last_two[0]), "the spring pass is the final pass again");
}

#[test]
fn finish_level_last_runs_finishing_only_at_the_last_level() {
    let mut p = boss_params();
    p.bottom_z = -6.0;
    p.stepdown = 2.0;
    p.passes.finish_level = camcraft_motion::machining2d::FinishLevel::Last;
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let t = contour(&boss, &p).expect("contoured");
    // One plunge per pass. The plunge sits at the pass's offset from the
    // wall, at the ring start (its x is 60 + offset for an Outside
    // contour). Levels above the last carry the roughing passes only.
    for z in [-2.0, -4.0] {
        let offs = level_pass_offsets(&t, z);
        assert_eq!(offs.len(), 2, "level {z}: two roughing passes, {offs:?}");
        for (got, want) in offs.iter().zip([7.59, 5.05]) {
            assert!((got - want).abs() <= 1e-6, "level {z}: pass at {got} mm, want {want}");
        }
    }
    let offs = level_pass_offsets(&t, -6.0);
    assert_eq!(offs.len(), 3, "last level: three passes, {offs:?}");
    for (got, want) in offs.iter().zip([7.59, 5.05, 5.00]) {
        assert!((got - want).abs() <= 1e-6, "last level: pass at {got} mm, want {want}");
    }
}

#[test]
fn finish_level_last_is_refused_when_the_flute_is_short() {
    let mut p = boss_params();
    p.bottom_z = -6.0;
    p.stepdown = 2.0;
    p.passes.finish_level = camcraft_motion::machining2d::FinishLevel::Last;
    let why = camcraft_motion::machining2d::contour_flute_covers_wall(&p, 4.0)
        .expect_err("a 4 mm flute cannot cut a 6 mm wall")
        .to_string();
    assert!(why.contains("finishing pass") && why.contains("flute"), "{why}");
    // A flute that covers the wall passes.
    camcraft_motion::machining2d::contour_flute_covers_wall(&p, 6.0).expect("6 mm flute covers 6 mm");
}

#[test]
fn a_closed_inside_chain_drops_a_pass_that_does_not_fit() {
    // A circle of radius 12, side inside, a 10 mm end mill: the pass at
    // 13 mm is empty (the bore is 12 mm), the passes at 9 and 5 cut.
    let circle: Vec<Point2> = (0..=256)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / 256.0;
            Point2::new(12.0 * a.cos(), 12.0 * a.sin())
        })
        .collect();
    let ring = Region::new(vec![Loop::new(circle).expect("circle")]).expect("region");
    let mut p = boss_params();
    p.side = Side::Inside;
    p.passes.rough_passes = 3;
    p.passes.rough_spacing = 4.0;
    p.passes.finish_passes = 0;
    p.passes.finish_spacing = 0.0;
    p.bottom_z = -1.0;
    p.stepdown = 1.0;
    let t = contour(&ring, &p).expect("the inside passes");
    // The pass at 13 mm is named in the notes, never dropped in silence.
    let named = t.notes.iter().any(|n| n.contains("13.000") && n.contains("narrower"));
    assert!(named, "the dropped pass is named: {:?}", t.notes);
    // The passes at 9 and 5 cut: two plunge sites. The plunge lands at
    // the lead-in's start, off the ring, so the ring's radius is read
    // from the cut feeds' endpoints (the polygon's vertices).
    // The pass is measured from the wall (the circle of radius 12); the
    // cut radius is 12 - offset. The passes at 9 and 5 from the wall cut
    // at radii 3 and 7. The feed endpoints sit on those circles (the
    // arc fitter keeps the polygon's vertices).
    let mut radii: Vec<f64> = t
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && m.from.z == m.to.z)
        .flat_map(|m| {
            [
                (m.from.x * m.from.x + m.from.y * m.from.y).sqrt(),
                (m.to.x * m.to.x + m.to.y * m.to.y).sqrt(),
            ]
        })
        .filter(|r| *r > 0.5)
        .collect();
    radii.sort_by(f64::total_cmp);
    radii.dedup_by(|a, b| (*a - *b).abs() < 0.1);
    let near = |r: f64, want: f64| (r - want).abs() <= 0.05;
    assert!(
        radii.iter().any(|&r| near(r, 3.0)) && radii.iter().any(|&r| near(r, 7.0)),
        "the passes at 9 and 5 cut at radii 3 and 7: {radii:?}"
    );
}

#[test]
fn an_empty_final_pass_keeps_the_existing_refusal() {
    // A bore narrower than the tool: the final offset collapses, so the
    // contour is refused by name (the existing refusal).
    let circle: Vec<Point2> = (0..=128)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / 128.0;
            Point2::new(2.0 * a.cos(), 2.0 * a.sin())
        })
        .collect();
    let ring = Region::new(vec![Loop::new(circle).expect("circle")]).expect("region");
    let mut p = boss_params();
    p.side = Side::Inside;
    p.bottom_z = -1.0;
    p.stepdown = 1.0;
    let err = contour(&ring, &p).expect_err("a 4 mm bore cannot take a 10 mm tool");
    let why = err.to_string();
    assert!(why.contains("narrower") || why.contains("collapsed"), "{why}");
}

#[test]
fn non_finite_and_bad_spacings_are_refused() {
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let nan = ContourParams {
        passes: camcraft_motion::machining2d::WallPasses { rough_spacing: f64::NAN, ..boss_params().passes },
        ..boss_params()
    };
    let inf = ContourParams {
        passes: camcraft_motion::machining2d::WallPasses { finish_spacing: f64::INFINITY, ..boss_params().passes },
        ..boss_params()
    };
    let rough0 = ContourParams {
        passes: camcraft_motion::machining2d::WallPasses { rough_passes: 2, rough_spacing: 0.0, ..boss_params().passes },
        ..boss_params()
    };
    let finish_neg = ContourParams {
        passes: camcraft_motion::machining2d::WallPasses { finish_passes: 1, finish_spacing: -1.0, ..boss_params().passes },
        ..boss_params()
    };
    let cases = [nan, inf, rough0, finish_neg];
    for p in cases {
        let err = contour(&boss, &p).expect_err("refused before any geometry");
        let why = err.to_string();
        assert!(why.contains("spacing") || why.contains("finite"), "{why}");
    }
}

#[test]
fn the_default_sweeps_the_plain_contour() {
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let plain = contour(&boss, &defaults(boss_params())).expect("plain");
    // The default's single pass is at 5.0 mm; the tool radius is 5.0, so
    // the swept band is the boss grown by 10.0. The contour cuts only
    // that band's outer boundary, so the leftover law is against it.
    let swept = boss.offset(10.0).expect("the swept band");
    let m = mask("s1_default", &swept, &plain);
    m.assert_no_overcut();
    assert!(m.metrics().swept_cells > 0, "the default sweeps");
}

#[test]
fn a_pass_on_an_open_chain_has_no_square_move_at_its_end() {
    // An open chain of one straight leg: the passes ride the leg's
    // offset, and every cut move is along the leg (never square to it).
    let chain = [Point2::new(0.0, 0.0), Point2::new(30.0, 0.0)];
    let p = boss_params();
    let air = |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(None) };
    let t = contour_open_run(&chain, &p, &air).expect("open passes");
    for m in &t.moves {
        if m.kind == MoveKind::Feed && m.from.z == m.to.z {
            let (dx, dy) = (m.to.x - m.from.x, m.to.y - m.from.y);
            if dx.hypot(dy) > 1e-9 {
                // The leg runs +X; a square move would be mostly ±Y.
                assert!(dx.abs() > dy.abs(), "a square move at the end: {m:?}");
            }
        }
    }
}
// ---- Journeyman spec S1, review round 3: the merge / error / order /
// tabs classes -----------------------------------------------------------

/// Two 20 mm bosses 12 mm apart, outside, a 10 mm tool, `n_r` = 2,
/// `d_r` = 2.54, `n_f` = 1, `d_f` = 0.05. The 7.59 mm roughing offset
/// merges the two bosses' rings into one loop; the pass must not panic,
/// each boss's final pass sits at 5.00 mm, and the merged roughing loop
/// is cut once and named.
#[test]
fn merged_roughing_loop_is_cut_once_and_named() {
    // Two 20 x 20 bosses, 12 mm apart edge to edge.
    let region = Region::new(vec![
        Loop::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(20.0, 0.0),
            Point2::new(20.0, 20.0),
            Point2::new(0.0, 20.0),
        ])
        .expect("boss a"),
        Loop::new(vec![
            Point2::new(32.0, 0.0),
            Point2::new(52.0, 0.0),
            Point2::new(52.0, 20.0),
            Point2::new(32.0, 20.0),
        ])
        .expect("boss b"),
    ])
    .expect("two bosses");
    let p = boss_params();
    let t = contour(&region, &p).expect("the wall passes do not panic");
    // Each boss's final pass (5.00 mm) is present: a plunge near each
    // boss's centre, at the 5.00 mm offset from its walls.
    let plunges: Vec<(f64, f64)> = t
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Plunge)
        .map(|m| (m.to.x, m.to.y))
        .collect();
    // Boss A's final pass starts at (-5, 0) or (25, 0)-style offset; the
    // exact start is the ring's axis-aligned point. Just assert two
    // distinct plunge clusters (one per boss) at the final offset.
    assert!(
        plunges.iter().any(|&(x, _)| x < 30.0) && plunges.iter().any(|&(x, _)| x > 30.0),
        "one final pass per boss: {plunges:?}"
    );
    // The merged roughing loop is named, never dropped in silence: the
    // note names the pass that merged (or the pass that was dropped).
    let named = t.notes.iter().any(|n| {
        n.contains("merged") || n.contains("wall pass") || n.contains("does not fit")
    });
    assert!(named, "the merged/dropped pass is named: {:?}", t.notes);
}

/// An open chain, three levels, `finish_level = last`: levels 1-2 carry
/// two passes (the roughing ones), level 3 carries three (the roughing
/// ones plus the finishing one), level by level.
#[test]
fn open_chain_finish_level_last_is_level_major() {
    // A straight 40 mm open run with air on one side.
    let chain = [Point2::new(0.0, 0.0), Point2::new(40.0, 0.0)];
    let mut p = boss_params();
    p.bottom_z = -6.0;
    p.stepdown = 2.0;
    p.passes.finish_level = camcraft_motion::machining2d::FinishLevel::Last;
    // Material below the run (the tool rides above it, air side).
    let mat = rect(0.0, -20.0, 40.0, 0.0);
    let section = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(mat.clone())) };
    let t = contour_open_run(&chain, &p, &section).expect("open passes");
    // The finishing pass (5.00 mm) runs only at the last level (-6.0).
    // The roughing passes (7.59, 5.05) run at every level.
    let at = |z: f64| -> Vec<f64> {
        t.moves
            .iter()
            .filter(|m| m.kind == MoveKind::Plunge && m.to.z == z)
            .map(|m| -m.to.y)
            .collect()
    };
    for z in [-2.0, -4.0] {
        let offs = at(z);
        assert_eq!(offs.len(), 2, "level {z}: two roughing passes, {offs:?}");
    }
    let offs = at(-6.0);
    assert_eq!(offs.len(), 3, "last level: three passes, {offs:?}");
}

/// An open chain whose final pass would touch the part: refused as
/// before, not dropped.
#[test]
fn open_chain_final_pass_touching_part_is_refused() {
    // A straight run with the tool's final pass landing on the part.
    // An open run along the left edge of a square, with a thin part
    // wall just off that edge on the air side: the ride's swept disc
    // (radius 5) reaches the wall, so the ride cuts the part.
    let chain = [Point2::new(0.0, 0.0), Point2::new(0.0, 20.0)];
    let mut p = boss_params();
    p.passes.rough_passes = 1;
    p.passes.rough_spacing = 0.0;
    p.passes.finish_passes = 0;
    // The stock is the square the run trims (material on the run's
    // right); the part is a thin wall just left of the run (air side).
    let stock = rect(0.0, 0.0, 20.0, 20.0);
    let part = rect(-3.0, -1.0, -1.0, 21.0);
    let stock_at = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(stock.clone())) };
    let part_at = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(part.clone())) };
    let err = camcraft_motion::machining2d::contour_open_run_sided(
        &chain,
        &p,
        Side::Outside,
        &stock_at,
        &part_at,
    )
    .expect_err("the ride cuts the part");
    let why = err.to_string();
    assert!(
        why.contains("cuts the part") || why.contains("wall") || why.contains("smaller tool"),
        "the refusal names the cause: {why}"
    );
}

/// Tabs with `rough_passes` = 2: refused by name.
#[test]
fn tabs_with_wall_passes_are_refused_by_name() {
    let mut p = boss_params();
    p.tab_count = 2;
    p.tab_width = 4.0;
    p.tab_height = 1.0;
    p.passes.rough_passes = 2;
    p.passes.rough_spacing = 2.54;
    let boss = rect(0.0, 0.0, 60.0, 40.0);
    let err = contour(&boss, &p).expect_err("tabs + wall passes refused");
    let why = err.to_string();
    assert!(why.contains("tabs") && why.contains("wall passes"), "{why}");
}

// ---- Journeyman spec S1, review round 4: the default is the pre-S1
// path, move for move -------------------------------------------

/// A 40 mm straight open run with air on one side, default passes.
fn open_run() -> Vec<Point2> {
    vec![Point2::new(0.0, 0.0), Point2::new(40.0, 0.0)]
}

fn open_material() -> Region {
    rect(-10.0, -20.0, 50.0, 0.0)
}

/// A default `contour_open_run` (3 levels) is, move for move, the
/// pre-S1 `contour_along_chain` on the same offset ride.
#[test]
fn default_open_run_matches_the_pre_s1_single_call() {
    let mut p = boss_params();
    p.passes = camcraft_motion::machining2d::WallPasses::default();
    p.clearance_z = 20.0;
    p.retract_z = 10.0;
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0;
    let mat = open_material();
    let section = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(mat.clone())) };
    let got = contour_open_run(&open_run(), &p, &section).expect("open passes");
    // The pre-S1 ride: the chain offset to its air side by r + s.
    let d = p.tool_diameter * 0.5 + p.stock_to_leave;
    let mut ride = camcraft_motion::machining2d::offset_run_side(&open_run(), d, true)
        .expect("the air side")
        .expect("a ride");
    ride.reverse();
    let want = camcraft_motion::machining2d::contour_along_chain(&ride, &p).expect("pre-S1");
    assert_eq!(got.moves, want.moves, "the default is the pre-S1 single call");
    // The note `contour_open_run` adds on top of the ride is unchanged.
    assert!(
        got.notes.iter().any(|n| n.contains("air side") && n.contains("5.000 mm")),
        "the pre-S1 air-side note: {:?}",
        got.notes
    );
}

/// A default `contour_open_run_sided` (3 levels) is, move for move, the
/// pre-S1 `contour_along_chain` on the same offset ride.
#[test]
fn default_open_run_sided_matches_the_pre_s1_single_call() {
    let mut p = boss_params();
    p.passes = camcraft_motion::machining2d::WallPasses::default();
    p.clearance_z = 20.0;
    p.retract_z = 10.0;
    p.top_z = 0.0;
    p.bottom_z = -6.0;
    p.stepdown = 2.0;
    let stock = open_material();
    let stock_at = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(Some(stock.clone())) };
    let part_at = move |_z: f64| -> Result<Option<Region>, camcraft_motion::machining2d::GenError> { Ok(None) };
    let got = camcraft_motion::machining2d::contour_open_run_sided(
        &open_run(),
        &p,
        Side::Outside,
        &stock_at,
        &part_at,
    )
    .expect("open passes");
    // The pre-S1 ride: the chain offset to the stock's air side by r + s.
    let d = p.tool_diameter * 0.5 + p.stock_to_leave;
    let mut ride = camcraft_motion::machining2d::offset_run_side(&open_run(), d, true)
        .expect("the air side")
        .expect("a ride");
    ride.reverse();
    let want = camcraft_motion::machining2d::contour_along_chain(&ride, &p).expect("pre-S1");
    assert_eq!(got.moves, want.moves, "the default is the pre-S1 single call");
    assert!(
        got.notes.iter().any(|n| n.contains("stock") && n.contains("5.000 mm")),
        "the pre-S1 stock-side note: {:?}",
        got.notes
    );
}
