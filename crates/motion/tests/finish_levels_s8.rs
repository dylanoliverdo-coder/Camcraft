//! Journeyman spec S8 motion checks (Checks 1-8 + repeated-level safety):
//! finishing depth levels at a smaller stepdown for Contour, Pocket, and Face.
//!
//! Gate map: covers camcraft_motion::machining2d (finish_levels, equal_steps,
//! levels, contour, contour_link, contour_depth, pocket, face),
//! camcraft_motion::approach.
//!
//! Provenance: Journeyman spec S8 (`docs/journeyman/specs/S8-finish-levels.md`);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007;
//! M. P. Groover, *Fundamentals of Modern Manufacturing*, Wiley.
//! No vendor parameters, text or code.

use camcraft_motion::cl::{Move, MoveKind, Toolpath};
use camcraft_motion::entry::Entry3D;
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::geometry::{Point3, Triangle};
use camcraft_motion::links::{Approach, Link, Links};
use camcraft_motion::machining2d::{
    ContourParams, FaceHoles, FaceParams, Lead, PocketParams, PocketStrategy, Side, WallPasses,
    contour, contour_whole_bodies, cut_levels_for, face, pocket, pocket_avoid,
};
use camcraft_motion::tools::ToolGeom;
use camcraft_motion::verify::{AvoidItem, AvoidSpec, Cutter};

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
    let lp = Loop::new(vec![
        Point2::new(x0, y0),
        Point2::new(x1, y0),
        Point2::new(x1, y1),
        Point2::new(x0, y1),
    ])
    .expect("rect loop");
    Region::new(vec![lp]).expect("rect region")
}

/// Ordered sequence of planar cutting Z levels separated by lifts to
/// `clearance_z` (preserving repeated Z levels while grouping passes within
/// a single level that hop only to `retract_z`).
fn planar_level_runs(path: &Toolpath, clearance_z: f64) -> Vec<f64> {
    let mut runs: Vec<f64> = Vec::new();
    let mut in_level: Option<f64> = None;
    for m in &path.moves {
        let planar_cut = m.kind == MoveKind::Feed && (m.from.z - m.to.z).abs() < 1e-9;
        if planar_cut {
            if in_level.is_none_or(|z| (z - m.to.z).abs() > 1e-8) {
                runs.push(m.to.z);
                in_level = Some(m.to.z);
            }
        } else if m.to.z >= clearance_z - 1e-6 {
            in_level = None;
        }
    }
    runs
}

/// Distinct planar cutting Z levels in order of first appearance.
fn distinct_levels(path: &Toolpath) -> Vec<f64> {
    let mut out = Vec::new();
    for m in &path.moves {
        if m.kind == MoveKind::Feed
            && (m.from.z - m.to.z).abs() < 1e-9
            && !out.iter().any(|&z: &f64| (z - m.to.z).abs() < 1e-8)
        {
            out.push(m.to.z);
        }
    }
    out
}

/// All planar Feed moves lying at cut height `z`.
fn moves_at_level(path: &Toolpath, z: f64) -> Vec<&Move> {
    path.moves
        .iter()
        .filter(|m| {
            m.kind == MoveKind::Feed
                && (m.from.z - z).abs() < 1e-8
                && (m.to.z - z).abs() < 1e-8
        })
        .collect()
}

fn assert_levels_eq(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "actual={actual:?} expected={expected:?}");
    for (&a, &e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < 1e-9, "actual={actual:?} expected={expected:?}");
    }
}

fn assert_level_shifted(path: &Toolpath, z_upper: f64, z_lower: f64, dz: f64) {
    let upper = moves_at_level(path, z_upper);
    let lower = moves_at_level(path, z_lower);
    assert!(!upper.is_empty(), "expected moves at Z={z_upper}");
    assert_eq!(upper.len(), lower.len(), "move count at {z_upper} vs {z_lower}");
    for (i, (u, l)) in upper.iter().zip(lower.iter()).enumerate() {
        assert_eq!(u.kind, l.kind, "move {i} kind");
        assert_eq!(u.feed, l.feed, "move {i} feed");
        assert!((u.from.x - l.from.x).abs() < 1e-9, "move {i} from.x");
        assert!((u.from.y - l.from.y).abs() < 1e-9, "move {i} from.y");
        assert!((u.to.x - l.to.x).abs() < 1e-9, "move {i} to.x");
        assert!((u.to.y - l.to.y).abs() < 1e-9, "move {i} to.y");
        assert!(((u.from.z - l.from.z) - dz).abs() < 1e-9, "move {i} from.z shift");
        assert!(((u.to.z - l.to.z) - dz).abs() < 1e-9, "move {i} to.z shift");
        assert_eq!(u.arc.is_some(), l.arc.is_some(), "move {i} arc presence");
        if let (Some(ua), Some(la)) = (u.arc, l.arc) {
            assert_eq!(ua.ccw, la.ccw, "move {i} arc ccw");
            assert_eq!(ua.plane, la.plane, "move {i} arc plane");
            assert!((ua.center.x - la.center.x).abs() < 1e-9, "move {i} arc cx");
            assert!((ua.center.y - la.center.y).abs() < 1e-9, "move {i} arc cy");
        }
    }
}

fn contour_boss_params(
    top_z: f64,
    bottom_z: f64,
    stepdown: f64,
    equal_steps: bool,
    finish_levels: u32,
    finish_stepdown: f64,
) -> ContourParams {
    ContourParams {
        tool_diameter: 6.0,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        retract_z: 5.0,
        top_z,
        bottom_z,
        stepdown,
        feed: 800.0,
        plunge_feed: 200.0,
        climb: true,
        side: Side::Outside,
        lead: Lead::Arc { radius: 3.0 },
        tolerance: 0.01,
        tab_count: 0,
        tab_width: 0.0,
        tab_height: 0.0,
        entry: Entry3D::Plunge,
        helix_radius: 1.5,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            approach: Approach {
                equal_steps,
                finish_levels,
                finish_stepdown,
                ..Approach::DEFAULT
            },
            ..Links::PLANAR
        },
        passes: WallPasses::default(),
    }
}

fn pocket_params(
    top_z: f64,
    bottom_z: f64,
    stepdown: f64,
    equal_steps: bool,
    finish_levels: u32,
    finish_stepdown: f64,
) -> PocketParams {
    PocketParams {
        tool_diameter: 6.0,
        stepover: 3.0,
        stock_to_leave: 0.0,
        clearance_z: 10.0,
        top_z,
        bottom_z,
        stepdown,
        feed: 800.0,
        plunge_feed: 200.0,
        climb: true,
        strategy: PocketStrategy::Zigzag,
        retract_z: Some(5.0),
        entry: Entry3D::Plunge,
        helix_radius: 1.5,
        optimal_load: 0.25,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            approach: Approach {
                equal_steps,
                finish_levels,
                finish_stepdown,
                ..Approach::DEFAULT
            },
            ..Links::PLANAR
        },
    }
}

fn face_params(
    top_z: Option<f64>,
    cut_z: f64,
    stepdown: Option<f64>,
    finish_levels: u32,
    finish_stepdown: f64,
) -> FaceParams {
    FaceParams {
        tool_diameter: 12.0,
        stepover: 8.0,
        clearance_z: 10.0,
        cut_z,
        top_z,
        stepdown,
        feed: 1000.0,
        plunge_feed: 250.0,
        climb: true,
        blend_turns: false,
        blend_radius: None,
        holes: FaceHoles::Lift,
        retract_feed: 0.0,
        finish_levels,
        finish_stepdown,
        stock_top: None,
    }
}

#[test]
fn s8_check_1_defaults_leave_paths_byte_identical() {
    assert_eq!(Approach::DEFAULT.finish_levels, 0);
    assert_eq!(Approach::DEFAULT.finish_stepdown, 0.0);

    let boss = rect(-15.0, -10.0, 15.0, 10.0);
    let c0 = contour(&boss, &contour_boss_params(0.0, -3.81, 1.905, false, 0, 0.0)).expect("c0");
    let mut cp = contour_boss_params(0.0, -3.81, 1.905, false, 0, 0.0);
    cp.links = Links::PLANAR;
    let c_plain = contour(&boss, &cp).expect("c_plain");
    assert_eq!(c0.moves, c_plain.moves);

    let pock = rect(0.0, 0.0, 40.0, 30.0);
    let p0 = pocket(&pock, &pocket_params(0.0, -6.0, 2.0, false, 0, 0.0)).expect("p0");
    let mut pp = pocket_params(0.0, -6.0, 2.0, false, 0, 0.0);
    pp.links = Links::PLANAR;
    let p_plain = pocket(&pock, &pp).expect("p_plain");
    assert_eq!(p0.moves, p_plain.moves);

    let stock = rect(0.0, 0.0, 80.0, 60.0);
    let f0 = face(&stock, &face_params(Some(0.0), -1.0, Some(0.6), 0, 0.0)).expect("f0");
    assert_levels_eq(&distinct_levels(&f0), &[-0.6, -1.0]);
}

#[test]
fn s8_checks_2_3_4_5_6_cut_levels_for_law() {
    // Check 2: equal steps on
    let a2 = Approach { equal_steps: true, finish_levels: 1, finish_stepdown: 0.508, ..Approach::DEFAULT };
    assert_levels_eq(&cut_levels_for(0.0, &a2, -3.81, 1.905).expect("c2"), &[-1.651, -3.302, -3.81]);

    // Check 3: equal steps off
    let a3 = Approach { equal_steps: false, finish_levels: 1, finish_stepdown: 0.508, ..Approach::DEFAULT };
    assert_levels_eq(&cut_levels_for(0.0, &a3, -3.81, 1.905).expect("c3"), &[-1.905, -3.302, -3.81]);

    // Check 4: n = 3, s = 0.25
    let a4 = Approach { equal_steps: false, finish_levels: 3, finish_stepdown: 0.25, ..Approach::DEFAULT };
    assert_levels_eq(
        &cut_levels_for(0.0, &a4, -6.0, 2.0).expect("c4"),
        &[-2.0, -4.0, -5.25, -5.5, -5.75, -6.0],
    );

    // Check 5: n = 1, s = 0 repeats the last level
    let a5 = Approach { equal_steps: false, finish_levels: 1, finish_stepdown: 0.0, ..Approach::DEFAULT };
    assert_levels_eq(&cut_levels_for(0.0, &a5, -4.0, 2.0).expect("c5"), &[-2.0, -4.0, -4.0]);

    // Check 6: n = 2, s = 2, top = 0, z_b = -3 -> refused naming both 4.000 and 3.000
    let a6 = Approach { equal_steps: false, finish_levels: 2, finish_stepdown: 2.0, ..Approach::DEFAULT };
    let err = cut_levels_for(0.0, &a6, -3.0, 2.0).expect_err("c6 refused").to_string();
    assert!(err.contains("finish_levels") && err.contains("finish_stepdown"), "{err}");
    assert!(err.contains("4.000") && err.contains("3.000"), "{err}");

    // Refusals also trigger through Contour, Pocket, and Face generators.
    let r = rect(0.0, 0.0, 40.0, 30.0);
    for gen_err in [
        contour(&r, &contour_boss_params(0.0, -3.0, 2.0, false, 2, 2.0)).expect_err("contour").to_string(),
        pocket(&r, &pocket_params(0.0, -3.0, 2.0, false, 2, 2.0)).expect_err("pocket").to_string(),
        face(&r, &face_params(Some(0.0), -3.0, Some(2.0), 2, 2.0)).expect_err("face").to_string(),
    ] {
        assert!(gen_err.contains("4.000") && gen_err.contains("3.000"), "{gen_err}");
    }
}

#[test]
fn s8_check_7_contour_30x20_boss_finishing_level_is_shifted_move_for_move() {
    let boss = rect(-15.0, -10.0, 15.0, 10.0);
    let p = contour_boss_params(0.0, -3.81, 1.905, true, 1, 0.508);
    let path = contour(&boss, &p).expect("contour boss");

    assert_levels_eq(&distinct_levels(&path), &[-1.651, -3.302, -3.81]);
    assert_level_shifted(&path, -3.302, -3.81, 0.508);

    // Same invariance with S1 wall passes enabled on every level.
    let mut p_wall = p;
    p_wall.passes = WallPasses {
        rough_passes: 2,
        rough_spacing: 0.5,
        finish_passes: 1,
        finish_spacing: 0.2,
        spring_passes: 0,
        ..WallPasses::default()
    };
    let path_wall = contour(&boss, &p_wall).expect("contour boss with wall passes");
    assert_levels_eq(&distinct_levels(&path_wall), &[-1.651, -3.302, -3.81]);
    assert_level_shifted(&path_wall, -3.302, -3.81, 0.508);
}

#[test]
fn s8_check_8_face_80x60_stock_levels_and_passes() {
    let stock = rect(0.0, 0.0, 80.0, 60.0);
    let p = face_params(Some(0.0), -1.0, Some(0.6), 1, 0.3);
    let path = face(&stock, &p).expect("face");

    assert_levels_eq(&distinct_levels(&path), &[-0.6, -0.7, -1.0]);
    assert_level_shifted(&path, -0.7, -1.0, 0.3);
}

#[test]
fn s8_check_5_zero_finish_stepdown_repeats_final_level_on_contour_pocket_and_face() {
    // Contour (standard Clearance link + KeepDown link)
    let boss = rect(-15.0, -10.0, 15.0, 10.0);
    let c_base = contour(&boss, &contour_boss_params(0.0, -4.0, 2.0, false, 0, 0.0)).expect("c base");
    let c_rep = contour(&boss, &contour_boss_params(0.0, -4.0, 2.0, false, 1, 0.0)).expect("c rep");
    assert_levels_eq(&planar_level_runs(&c_rep, 10.0), &[-2.0, -4.0, -4.0]);
    assert_eq!(moves_at_level(&c_rep, -4.0).len(), moves_at_level(&c_base, -4.0).len() * 2);

    // Contour with KeepDown level link also repeats the full -4.0 ring cleanly
    // without extra ramp laps: the first -4.0 pass stays down at the seam (skipping
    // lead-out) and the repeat pass walks the full ring + lead-out.
    let mut c_keep_base_p = contour_boss_params(0.0, -4.0, 2.0, false, 0, 0.0);
    c_keep_base_p.links.level = Link::KeepDown;
    let c_keep_base = contour(&boss, &c_keep_base_p).expect("c keepdown base");
    let mut c_keep = contour_boss_params(0.0, -4.0, 2.0, false, 1, 0.0);
    c_keep.links.level = Link::KeepDown;
    let c_keep_path = contour(&boss, &c_keep).expect("c keepdown rep");
    let base_keep_moves = moves_at_level(&c_keep_base, -4.0);
    let rep_keep_moves = moves_at_level(&c_keep_path, -4.0);
    // KeepDown omits the 1 lead-out move between the two -4.0 passes, so repeating
    // the ring adds (base_keep_moves - 1) moves and doubles the ring walk moves.
    assert_eq!(
        rep_keep_moves.len(),
        base_keep_moves.len() * 2 - 1,
        "KeepDown repeat must walk the full ring twice at -4.0"
    );

    // Pocket across Plunge, Helix, Ramp, Arc entries
    let pock = rect(0.0, 0.0, 40.0, 30.0);
    for entry in [Entry3D::Plunge, Entry3D::Helix, Entry3D::Ramp, Entry3D::Arc] {
        let mut pb = pocket_params(0.0, -4.0, 2.0, false, 0, 0.0);
        pb.entry = entry;
        let p_base = pocket(&pock, &pb).expect("pocket base");
        let mut pp = pocket_params(0.0, -4.0, 2.0, false, 1, 0.0);
        pp.entry = entry;
        let path = pocket(&pock, &pp).expect("pocket repeat");
        assert_levels_eq(&planar_level_runs(&path, 10.0), &[-2.0, -4.0, -4.0]);
        assert!(
            moves_at_level(&path, -4.0).len() >= moves_at_level(&p_base, -4.0).len() * 2,
            "entry {entry:?}: final level -4.0 must run twice (got {}, base {})",
            moves_at_level(&path, -4.0).len(),
            moves_at_level(&p_base, -4.0).len()
        );
    }

    // Face (staged & single-level)
    let stock = rect(0.0, 0.0, 80.0, 60.0);
    let f_base = face(&stock, &face_params(Some(0.0), -4.0, Some(2.0), 0, 0.0)).expect("f base");
    let f_rep = face(&stock, &face_params(Some(0.0), -4.0, Some(2.0), 1, 0.0)).expect("f rep");
    assert_levels_eq(&planar_level_runs(&f_rep, 10.0), &[-2.0, -4.0, -4.0]);
    assert_eq!(moves_at_level(&f_rep, -4.0).len(), moves_at_level(&f_base, -4.0).len() * 2);

    let f_single_rep = face(&stock, &face_params(None, -1.0, None, 2, 0.0)).expect("f single rep");
    assert_levels_eq(&planar_level_runs(&f_single_rep, 10.0), &[-1.0, -1.0, -1.0]);
}

#[test]
fn s8_pocket_finishing_levels_shift_passes_identically() {
    let pock = rect(0.0, 0.0, 40.0, 30.0);
    let p = pocket_params(0.0, -6.0, 2.0, false, 3, 0.25);
    let path = pocket(&pock, &p).expect("pocket");
    assert_levels_eq(&distinct_levels(&path), &[-2.0, -4.0, -5.25, -5.5, -5.75, -6.0]);
    assert_level_shifted(&path, -5.25, -5.5, 0.25);
    assert_level_shifted(&path, -5.5, -5.75, 0.25);
    assert_level_shifted(&path, -5.75, -6.0, 0.25);
}

fn box_facets(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Vec<Triangle> {
    let v = [
        Point3::new(x0, y0, z0),
        Point3::new(x1, y0, z0),
        Point3::new(x1, y1, z0),
        Point3::new(x0, y1, z0),
        Point3::new(x0, y0, z1),
        Point3::new(x1, y0, z1),
        Point3::new(x1, y1, z1),
        Point3::new(x0, y1, z1),
    ];
    let q = |a: usize, b: usize, c: usize, d: usize| [Triangle::new(v[a], v[b], v[c]), Triangle::new(v[a], v[c], v[d])];
    [q(0, 3, 2, 1), q(4, 5, 6, 7), q(0, 1, 5, 4), q(2, 3, 7, 6), q(0, 4, 7, 3), q(1, 2, 6, 5)]
        .into_iter()
        .flatten()
        .collect()
}

#[test]
fn s8_whole_model_multi_band_contour_preserves_zero_stepdown_repeat() {
    // Stepped solid: narrow 20x20 upper boss on [-2, 0] atop wider 40x40 base on [-4, -2].
    // Whole-model contour exercises `reach_up` when the outline grows at Z=-2.
    let mut part = box_facets(-10.0, -10.0, -2.0, 10.0, 10.0, 0.0);
    part.extend(box_facets(-20.0, -20.0, -4.0, 20.0, 20.0, -2.0));
    let top_region = rect(-10.0, -10.0, 10.0, 10.0);

    let p_base = contour_boss_params(0.0, -4.0, 2.0, false, 0, 0.0);
    let p_rep = contour_boss_params(0.0, -4.0, 2.0, false, 1, 0.0);
    let base = contour_whole_bodies(&top_region, &part, &[], &p_base).expect("whole base");
    let rep = contour_whole_bodies(&top_region, &part, &[], &p_rep).expect("whole rep");

    assert_eq!(
        moves_at_level(&rep, -4.0).len(),
        moves_at_level(&base, -4.0).len() * 2,
        "multi-band whole-model contour must repeat final level -4.0 twice"
    );
}

#[test]
fn s8_inside_bore_lead_none_repeat_descends_at_opened_wall_not_uncut_core() {
    // 30 x 30 mm bore centered at (0, 0), tool D=6 -> wall offset ring at +-12 mm.
    // With Lead::None, e2 lies on the offset ring (|x|=12 or |y|=12), while the
    // uncut interior core is near (0, 0). On an s=0 repeat level, the tool must
    // descend directly at e2 (already opened on the preceding level) for both
    // center-cutting and non-center-cutting tools, never plunging into the core.
    let bore = rect(-15.0, -15.0, 15.0, 15.0);
    for entry in [Entry3D::Ramp, Entry3D::Helix] {
        for center_cutting in [true, false] {
            let mut p = contour_boss_params(0.0, -4.0, 2.0, false, 1, 0.0);
            p.side = Side::Inside;
            p.lead = Lead::None;
            p.entry = entry;
            p.links.approach.center_cutting = center_cutting;

            let mut p_base = p;
            p_base.links.approach.finish_levels = 0;

            let base = contour(&bore, &p_base).expect("bore base");
            let rep = contour(&bore, &p).expect("bore repeat must succeed for center_cutting={center_cutting}");

            assert_levels_eq(&planar_level_runs(&rep, 10.0), &[-2.0, -4.0, -4.0]);
            // Count the 4-sided +-12 mm offset ring moves at Z=-4.0 (excluding
            // any initial dive-link move on the first Helix level; the repeat
            // level descends directly at opened e2 without a redundant dive link).
            let on_ring = |x: f64, y: f64| (x.abs() - 12.0).abs() < 1e-3 || (y.abs() - 12.0).abs() < 1e-3;
            let ring_moves = |tp: &Toolpath| {
                moves_at_level(tp, -4.0)
                    .into_iter()
                    .filter(|m| on_ring(m.from.x, m.from.y) && on_ring(m.to.x, m.to.y))
                    .count()
            };
            assert_eq!(ring_moves(&base), 4, "entry={entry:?} cc={center_cutting}: base must cut 4 ring sides");
            assert_eq!(
                ring_moves(&rep),
                8,
                "entry={entry:?} cc={center_cutting}: final level -4.0 must cut the 4-sided ring twice"
            );
            // Every plunge landing at Z=-4.0 must lie on the +-12 mm offset ring,
            // never at the uncut interior core near (0, 0).
            let core_plunges: Vec<_> = rep
                .moves
                .iter()
                .filter(|m| {
                    m.kind == MoveKind::Plunge
                        && (m.to.z - (-4.0)).abs() < 1e-6
                        && m.to.x.abs() < 11.5
                        && m.to.y.abs() < 11.5
                })
                .collect();
            assert!(
                core_plunges.is_empty(),
                "entry={entry:?} cc={center_cutting}: repeat plunged into uncut core: {core_plunges:?}"
            );
        }
    }
}

#[test]
fn s8_pocket_avoid_waterline_zero_stepdown_repeat_runs_cleanly() {
    let pock = rect(0.0, 0.0, 60.0, 40.0);
    // Keep-out clamp standing at the right edge of the pocket (at x in [50, 62]).
    let clamp = box_facets(50.0, 10.0, -5.0, 62.0, 30.0, 10.0);
    let avoid = AvoidSpec {
        items: vec![AvoidItem { label: "clamp".into(), tris: clamp, edges: vec![] }],
        cutter: Cutter::flat(&ToolGeom {
            radius: 3.0,
            nose_radius: 0.0,
            flute_len: 10.0,
            shank_radius: 3.0,
            shank_len: 15.0,
            holder_radius: 10.0,
            holder_len: 20.0,
        }),
        clearance: 0.5,
    };

    let mut base_p = pocket_params(0.0, -4.0, 2.0, false, 0, 0.0);
    base_p.strategy = PocketStrategy::Waterline;
    let mut rep_p = base_p;
    rep_p.links.approach.finish_levels = 1;

    let base = pocket_avoid(&pock, &base_p, &avoid).expect("pocket_avoid base");
    let rep = pocket_avoid(&pock, &rep_p, &avoid).expect("pocket_avoid rep");

    assert_levels_eq(&planar_level_runs(&rep, 10.0), &[-2.0, -4.0, -4.0]);
    assert_eq!(
        moves_at_level(&rep, -4.0).len(),
        moves_at_level(&base, -4.0).len() * 2,
        "pocket_avoid waterline must repeat final level -4.0 twice"
    );
    assert!(
        camcraft_sim::verify::crossed_pairs(&rep, 0.01).is_empty(),
        "pocket_avoid waterline repeat must remain crossing-free"
    );
}
