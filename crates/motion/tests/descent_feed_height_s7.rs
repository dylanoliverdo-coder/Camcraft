//! Motion integration tests for Journeyman spec S7: 2D descents, a stated
//! height where the rapid becomes a feed (`Approach::feed_from`).
//!
//! Provenance: Journeyman spec S7 (docs/journeyman/specs/S7-descent-feed-height.md);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007; RS-274 /
//! ISO 6983; no vendor parameters, text or code.

use camcraft_motion::approach::{Approach, FeedFrom};
use camcraft_motion::cl::{Move, MoveKind};
use camcraft_motion::entry::Entry3D;
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::links::{Link, Links};
use camcraft_motion::machining2d::{
    ContourParams, Lead, PocketParams, PocketStrategy, Side, WallPasses, contour_open_side, pocket,
};

fn open_params(bottom_z: f64, stepdown: f64, feed_from: FeedFrom) -> ContourParams {
    ContourParams {
        tool_diameter: 10.0,
        stock_to_leave: 0.0,
        clearance_z: 5.0,
        retract_z: 5.0,
        top_z: 0.0,
        bottom_z,
        stepdown,
        feed: 500.0,
        plunge_feed: 120.0,
        climb: true,
        side: Side::Auto,
        lead: Lead::Linear { length: 5.0 },
        tolerance: 0.005,
        tab_count: 0,
        tab_width: 0.0,
        tab_height: 0.0,
        entry: Entry3D::Plunge,
        helix_radius: 1.0,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            level: Link::Retract,
            approach: Approach { feed_from, ..Approach::DEFAULT },
            ..Links::PLANAR
        },
        passes: WallPasses::default(),
    }
}

fn pocket_params(entry: Entry3D, feed_from: FeedFrom) -> PocketParams {
    PocketParams {
        tool_diameter: 6.0,
        stepover: 3.0,
        stock_to_leave: 0.0,
        clearance_z: 5.0,
        retract_z: Some(5.0),
        top_z: 0.0,
        bottom_z: -5.0,
        stepdown: 2.5,
        feed: 600.0,
        plunge_feed: 150.0,
        climb: true,
        entry,
        helix_radius: 1.5,
        strategy: PocketStrategy::OffsetRings,
        optimal_load: 0.25,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            approach: Approach { feed_from, ..Approach::DEFAULT },
            ..Links::PLANAR
        },
    }
}

fn rect_region(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
    Region::new(vec![Loop::new(vec![
        Point2::new(x0, y0),
        Point2::new(x1, y0),
        Point2::new(x1, y1),
        Point2::new(x0, y1),
    ])
    .expect("rect loop")])
    .expect("rect region")
}

/// Collect every plunge move alongside the rapid immediately preceding it (if any).
fn plunge_pairs(moves: &[Move]) -> Vec<(Option<&Move>, &Move)> {
    moves
        .iter()
        .enumerate()
        .filter(|(_, m)| m.kind == MoveKind::Plunge)
        .map(|(i, m)| (i.checked_sub(1).and_then(|j| moves.get(j)), m))
        .collect()
}

#[test]
fn check1_auto_matches_default_approach_byte_for_byte() {
    let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
    let mut p_default = open_params(-5.0, 5.0, FeedFrom::Auto);
    p_default.links.approach = Approach::DEFAULT;
    let p_auto = open_params(-5.0, 5.0, FeedFrom::Auto);
    let a = contour_open_side(&chain, true, &p_default).expect("default open");
    let b = contour_open_side(&chain, true, &p_auto).expect("auto open");
    assert_eq!(a.moves, b.moves);

    // Under Auto, open runs still feed the whole way from traverse (z = 5.0).
    let plunges = plunge_pairs(&a.moves);
    assert_eq!(plunges.len(), 1);
    assert!((plunges[0].1.from.z - 5.0).abs() < 1e-9);
    assert!((plunges[0].1.to.z + 5.0).abs() < 1e-9);
}

#[test]
fn check2_open_run_single_level_material_gap_1_and_0() {
    let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];

    // material { gap: 1 } -> rapid 5 -> 1, plunge feed 1 -> -5
    let path1 = contour_open_side(&chain, true, &open_params(-5.0, 5.0, FeedFrom::Material { gap: 1.0 }))
        .expect("material gap 1");
    let p1 = plunge_pairs(&path1.moves);
    assert_eq!(p1.len(), 1);
    let (rapid1, plunge1) = p1[0];
    let rapid1 = rapid1.expect("rapid before plunge");
    assert_eq!(rapid1.kind, MoveKind::Rapid);
    assert!((rapid1.from.z - 5.0).abs() < 1e-9, "rapid starts at traverse 5: {rapid1:?}");
    assert!((rapid1.to.z - 1.0).abs() < 1e-9, "rapid ends at z=1: {rapid1:?}");
    assert!((plunge1.from.z - 1.0).abs() < 1e-9, "plunge starts at z=1: {plunge1:?}");
    assert!((plunge1.to.z + 5.0).abs() < 1e-9, "plunge ends at z=-5: {plunge1:?}");
    assert_eq!(plunge1.feed, Some(120.0));

    // material { gap: 0 } -> rapid 5 -> 0, plunge feed 0 -> -5
    let path0 = contour_open_side(&chain, true, &open_params(-5.0, 5.0, FeedFrom::Material { gap: 0.0 }))
        .expect("material gap 0");
    let p0 = plunge_pairs(&path0.moves);
    assert_eq!(p0.len(), 1);
    let (rapid0, plunge0) = p0[0];
    let rapid0 = rapid0.expect("rapid before plunge");
    assert_eq!(rapid0.kind, MoveKind::Rapid);
    assert!((rapid0.from.z - 5.0).abs() < 1e-9);
    assert!((rapid0.to.z - 0.0).abs() < 1e-9);
    assert!((plunge0.from.z - 0.0).abs() < 1e-9);
    assert!((plunge0.to.z + 5.0).abs() < 1e-9);
    assert_eq!(plunge0.feed, Some(120.0));
}

#[test]
fn check3_open_run_three_levels_material_gap_1() {
    let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
    let path = contour_open_side(&chain, true, &open_params(-6.0, 2.0, FeedFrom::Material { gap: 1.0 }))
        .expect("3-level open run");
    let plunges = plunge_pairs(&path.moves);
    assert_eq!(plunges.len(), 3, "3 levels -> 3 plunges: {plunges:?}");

    let expected = [(1.0, -2.0), (-1.0, -4.0), (-3.0, -6.0)];
    for (idx, ((rapid, plunge), (exp_hz, exp_z))) in plunges.iter().zip(expected).enumerate() {
        let rapid = rapid.expect("rapid down before plunge");
        assert_eq!(rapid.kind, MoveKind::Rapid, "level {}", idx + 1);
        assert!(
            (rapid.to.z - exp_hz).abs() < 1e-9,
            "level {}: expected rapid to {exp_hz}, got {}",
            idx + 1,
            rapid.to.z
        );
        assert!(
            (plunge.from.z - exp_hz).abs() < 1e-9 && (plunge.to.z - exp_z).abs() < 1e-9,
            "level {}: expected plunge {exp_hz} -> {exp_z}, got {} -> {}",
            idx + 1,
            plunge.from.z,
            plunge.to.z
        );
    }

    // Multi-pass open run (S1 + S7): level 2 and 3 roughing passes must also step down mat_top.
    let mut mp = open_params(-6.0, 2.0, FeedFrom::Material { gap: 1.0 });
    mp.passes = WallPasses { rough_passes: 2, rough_spacing: 1.0, ..WallPasses::default() };
    let mp_path = contour_open_side(&chain, true, &mp).expect("multi-pass 3-level open run");
    let mp_plunges = plunge_pairs(&mp_path.moves);
    assert_eq!(mp_plunges.len(), 6);
    for (k, (_, pl)) in mp_plunges.iter().enumerate() {
        let exp_hz = expected[k / 2].0;
        let exp_z = expected[k / 2].1;
        assert!(
            (pl.from.z - exp_hz).abs() < 1e-9 && (pl.to.z - exp_z).abs() < 1e-9,
            "multi-pass dive {k}: expected {exp_hz} -> {exp_z}, got {} -> {}",
            pl.from.z,
            pl.to.z
        );
    }
}

#[test]
fn check4_pocket_two_levels_absolute_z_half_mm() {
    let reg = rect_region(0.0, 0.0, 40.0, 30.0);
    let path = pocket(&reg, &pocket_params(Entry3D::Plunge, FeedFrom::Absolute { z: 0.5 }))
        .expect("pocket absolute 0.5");
    let plunges = plunge_pairs(&path.moves);
    assert!(plunges.len() >= 2, "2 levels -> at least 2 plunges");
    for (rapid, plunge) in plunges {
        let rapid = rapid.expect("rapid down before plunge");
        assert_eq!(rapid.kind, MoveKind::Rapid);
        assert!((rapid.to.z - 0.5).abs() < 1e-9, "rapid to 0.5: {rapid:?}");
        assert!((plunge.from.z - 0.5).abs() < 1e-9, "plunge from 0.5: {plunge:?}");
        assert_eq!(plunge.feed, Some(150.0));
    }
}

#[test]
fn check5_pocket_absolute_below_material_top_refused_by_name() {
    let reg = rect_region(0.0, 0.0, 40.0, 30.0);
    let err = pocket(&reg, &pocket_params(Entry3D::Plunge, FeedFrom::Absolute { z: -1.0 }))
        .expect_err("rapid into material must refuse");
    let msg = err.to_string();
    assert!(msg.contains("descent feed height refused"), "{msg}");
    assert!(msg.contains("-1.000") && msg.contains("0.000"), "must name both heights: {msg}");
    assert!(msg.contains('(') && msg.contains(')'), "must name descent position: {msg}");
}

#[test]
fn check6_absolute_above_traverse_feeds_whole_way_without_downward_rapid() {
    let reg = rect_region(0.0, 0.0, 40.0, 30.0);
    let path = pocket(&reg, &pocket_params(Entry3D::Plunge, FeedFrom::Absolute { z: 20.0 }))
        .expect("pocket absolute 20");
    let plunges = plunge_pairs(&path.moves);
    assert!(plunges.len() >= 2);
    for (_, plunge) in plunges {
        assert!((plunge.from.z - 5.0).abs() < 1e-9, "feeds from traverse z=5: {plunge:?}");
    }
    let downward_rapids: Vec<_> = path
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Rapid && m.to.z < m.from.z - 1e-9)
        .collect();
    assert!(downward_rapids.is_empty(), "no rapid down when hz >= z_t: {downward_rapids:?}");

    // Also on open run with traverse at 5.
    let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
    let open = contour_open_side(&chain, true, &open_params(-5.0, 5.0, FeedFrom::Absolute { z: 20.0 }))
        .expect("open absolute 20");
    let open_plunges = plunge_pairs(&open.moves);
    assert_eq!(open_plunges.len(), 1);
    assert!((open_plunges[0].1.from.z - 5.0).abs() < 1e-9);
    assert!(open.moves.iter().all(|m| !(m.kind == MoveKind::Rapid && m.to.z < m.from.z - 1e-9)));
}

#[test]
fn refuses_hz_at_or_below_cut_level_and_invalid_values() {
    // hz <= z_cut on pocket
    let reg = rect_region(0.0, 0.0, 40.0, 30.0);
    let err = pocket(&reg, &pocket_params(Entry3D::Plunge, FeedFrom::Absolute { z: -2.5 }))
        .expect_err("hz == z_cut must refuse");
    let msg = err.to_string();
    assert!(msg.contains("descent feed height refused"), "{msg}");
    assert!(msg.contains("-2.500") && msg.contains('('), "{msg}");

    // hz <= z_cut on open run (surface cut z_cut == 0 with gap == 0)
    let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
    let mut p = open_params(0.0, 1.0, FeedFrom::Material { gap: 0.0 });
    p.top_z = 0.0;
    let err = contour_open_side(&chain, true, &p).expect_err("gap=0 on z=0 cut must refuse");
    assert!(err.to_string().contains("descent feed height refused"), "{err}");

    // Approach::validate refusals for negative/non-finite gap and non-finite z.
    for bad in [
        FeedFrom::Material { gap: -0.1 },
        FeedFrom::Material { gap: f64::NAN },
        FeedFrom::Material { gap: f64::INFINITY },
        FeedFrom::Absolute { z: f64::NAN },
        FeedFrom::Absolute { z: f64::NEG_INFINITY },
    ] {
        let a = Approach { feed_from: bad, ..Approach::DEFAULT };
        let why = a.validate().expect_err("bad feed_from must refuse");
        assert!(why.contains("feed_from"), "{bad:?}: {why}");
    }
}

#[test]
fn helix_and_ramp_entries_keep_their_own_start_heights() {
    let reg = rect_region(0.0, 0.0, 40.0, 30.0);
    for entry in [Entry3D::Helix, Entry3D::Ramp] {
        let base = pocket(&reg, &pocket_params(entry, FeedFrom::Auto)).expect("auto");
        let stated = pocket(&reg, &pocket_params(entry, FeedFrom::Absolute { z: 0.5 })).expect("abs 0.5");
        // First non-rapid move's start Z (the helix or ramp start height) must be unchanged.
        let first_cut = |p: &camcraft_motion::cl::Toolpath| {
            p.moves.iter().find(|m| m.kind == MoveKind::Feed).expect("cut move").from.z
        };
        assert!(
            (first_cut(&base) - first_cut(&stated)).abs() < 1e-9,
            "{entry:?}: helix/ramp start height must not change ({} vs {})",
            first_cut(&base),
            first_cut(&stated)
        );
    }
}
