//! Journeyman spec S3 synthetic checks (Checks 1-4): fed vertical retract on
//! all 8 2D milling operations, switch-height law, keep-down / microlift
//! preservation, and contour `ring_link` safety.
//!
//! Provenance: Journeyman spec S3 (docs/journeyman/specs/S3-fed-retract.md);
//! no vendor parameters, text or code.

use camcraft_motion::approach::Approach;
use camcraft_motion::cl::{Move, MoveKind, Toolpath};
use camcraft_motion::entry::Entry3D;
use camcraft_motion::fed_retract::switch_z;
use camcraft_motion::geo2d::{Loop, Point2, Region};
use camcraft_motion::links::{Link, Links};
use camcraft_motion::machining2d::{
    AdaptiveTrochParams, BoreHole, BoreParams, CircularParams, ContourParams, FaceHoles,
    FaceParams, Lead, PocketParams, PocketStrategy, Side, SlotParams, TrochoidReturn, WallPasses,
    adaptive_trochoidal_with, bore, circular, contour, contour_along_chain, face, pocket, slot,
};

const RF: f64 = 2540.0;
const EPS: f64 = 1e-4;

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
    Region::new(vec![
        Loop::new(vec![
            Point2::new(x0, y0), Point2::new(x1, y0), Point2::new(x1, y1), Point2::new(x0, y1),
        ])
        .expect("loop"),
    ])
    .expect("rect")
}

fn is_vert_up(m: &Move) -> bool {
    (m.to.x - m.from.x).abs() <= EPS && (m.to.y - m.from.y).abs() <= EPS && m.to.z > m.from.z + EPS
}

fn is_fed_retract(m: &Move, rf: f64) -> bool {
    m.kind == MoveKind::Feed && is_vert_up(m) && m.feed.is_some_and(|f| (f - rf).abs() < 1e-6)
}

fn assert_no_bad_retract_feed(path: &Toolpath, rf: f64, label: &str) {
    for (i, m) in path.moves.iter().enumerate() {
        if m.kind == MoveKind::Feed && m.feed.is_some_and(|f| (f - rf).abs() < 1e-6) {
            let dxy = (m.to.x - m.from.x).hypot(m.to.y - m.from.y);
            assert!(
                dxy <= EPS && m.to.z > m.from.z + EPS,
                "{label}: move {i} at retract_feed must be strictly vertical upward, got {:?} -> {:?}",
                m.from, m.to
            );
        }
    }
}

fn assert_final_lift_fed(path: &Toolpath, rf: f64, expected_hz: f64, clearance_z: f64, label: &str) {
    assert_no_bad_retract_feed(path, rf, label);
    let n = path.moves.len();
    assert!(n >= 2, "{label}: expected at least 2 moves, got {n}");
    let mut idx = n - 1;
    assert_eq!(path.moves[idx].kind, MoveKind::Rapid, "{label}: final move must be rapid");
    assert!((path.moves[idx].to.z - clearance_z).abs() <= 1e-3, "{label}: final rapid must reach clearance_z");
    while idx > 0 && path.moves[idx].kind == MoveKind::Rapid && is_vert_up(&path.moves[idx]) {
        idx -= 1;
    }
    let fed = &path.moves[idx];
    let first_rap = &path.moves[idx + 1];
    assert!(is_fed_retract(fed, rf), "{label}: move before trailing rapid must be vertical fed retract at {rf}, got {fed:?}");
    assert!((fed.to.z - expected_hz).abs() <= 1e-3, "{label}: final fed retract must end at hz={expected_hz}, got {}", fed.to.z);
    assert!((first_rap.from.z - expected_hz).abs() <= 1e-3, "{label}: trailing rapid must start at hz={expected_hz}");
}

fn links_retract(pass: Link, retract_feed: f64) -> Links {
    Links {
        level: Link::Retract,
        pass,
        approach: Approach { plunge_height: 1.0, retract_feed, ..Approach::DEFAULT },
        ..Links::PLANAR
    }
}

fn pocket_params(strategy: PocketStrategy, retract_feed: f64) -> PocketParams {
    PocketParams {
        tool_diameter: 6.0, stepover: 3.0, stock_to_leave: 0.0,
        clearance_z: 10.0, retract_z: Some(5.0), top_z: 0.0, bottom_z: -5.0, stepdown: 2.5,
        feed: 800.0, plunge_feed: 200.0, climb: true, strategy, optimal_load: 0.2,
        entry: Entry3D::Plunge, helix_radius: 1.5, blend_turns: false, blend_radius: None,
        links: links_retract(Link::Retract, retract_feed),
    }
}

#[test]
fn s3_check_1_and_2_pocket_40x30_two_levels() {
    let r = rect(0.0, 0.0, 40.0, 30.0);
    let base = pocket(&r, &pocket_params(PocketStrategy::Zigzag, 0.0)).expect("baseline pocket");
    let zero_explicit = pocket(&r, &pocket_params(PocketStrategy::Zigzag, 0.0)).expect("zero pocket");
    assert_eq!(base.moves, zero_explicit.moves, "Check 1: retract_feed=0 matches baseline move-for-move");

    let fed = pocket(&r, &pocket_params(PocketStrategy::Zigzag, RF)).expect("fed pocket");
    assert_no_bad_retract_feed(&fed, RF, "Check 2 pocket");

    let (hz1, hz2) = (switch_z(0.0, 1.0, 5.0), switch_z(-2.5, 1.0, 5.0));
    assert!((hz1 - 1.0).abs() < 1e-6 && (hz2 - (-1.5)).abs() < 1e-6);

    let mut fed_lifts = 0usize;
    let mut cur_hz = hz1;
    for (i, m) in fed.moves.iter().enumerate() {
        if m.kind == MoveKind::Feed && (m.to.z - m.from.z).abs() <= EPS {
            if (m.to.z - (-2.5)).abs() < 1e-3 { cur_hz = hz1; }
            else if (m.to.z - (-5.0)).abs() < 1e-3 { cur_hz = hz2; }
        }
        if is_fed_retract(m, RF) {
            fed_lifts += 1;
            let expected_hz = if (m.from.z - (-2.5)).abs() < 1e-3 { hz1 }
                else if (m.from.z - (-5.0)).abs() < 1e-3 { hz2 }
                else { panic!("unexpected lift start Z: {}", m.from.z); };
            assert!((m.to.z - expected_hz).abs() < 1e-3, "lift {i} leaving {} must end at hz={expected_hz}, got {}", m.from.z, m.to.z);
            let next = fed.moves.get(i + 1).expect("rapid must follow fed lift");
            assert!(next.kind == MoveKind::Rapid && is_vert_up(next), "move after fed lift {i} must be straight-up Rapid");
        }
        if m.kind == MoveKind::Rapid && is_vert_up(m) {
            assert!(m.from.z >= cur_hz - 1e-3, "upward rapid {i} starts below switch height: {} < {cur_hz}", m.from.z);
        }
    }
    assert!(fed_lifts >= 2, "Check 2: expected >= 2 fed lifts, got {fed_lifts}");

    let cuts_of = |tp: &Toolpath| -> Vec<Move> {
        tp.moves.iter().filter(|m| m.kind != MoveKind::Rapid && !is_fed_retract(m, RF)).copied().collect()
    };
    assert_eq!(cuts_of(&base), cuts_of(&fed), "Check 2: cutting moves must be identical");
    assert_final_lift_fed(&fed, RF, hz2, 10.0, "Check 2 pocket final");
}

#[test]
fn s3_check_3_all_eight_2d_ops_final_lift() {
    let r = rect(0.0, 0.0, 40.0, 30.0);
    let links_rf = links_retract(Link::Retract, RF);

    // 1-3. Pocket, Offset2D, Adaptive2D (Spiral + Trochoidal Retract & Microlift)
    assert_final_lift_fed(&pocket(&r, &pocket_params(PocketStrategy::Zigzag, RF)).expect("pocket"), RF, -1.5, 10.0, "Pocket");
    assert_final_lift_fed(&pocket(&r, &pocket_params(PocketStrategy::OffsetRings, RF)).expect("offset"), RF, -1.5, 10.0, "Offset2D");
    assert_final_lift_fed(&pocket(&r, &pocket_params(PocketStrategy::Adaptive, RF)).expect("spiral"), RF, -1.5, 10.0, "Adaptive2D Spiral");

    let troch_ret = adaptive_trochoidal_with(
        &r, &pocket_params(PocketStrategy::Adaptive, RF),
        &AdaptiveTrochParams { ret: Some(TrochoidReturn::Retract), ..AdaptiveTrochParams::default() },
    ).expect("adaptive troch retract");
    assert_final_lift_fed(&troch_ret, RF, -0.5, 10.0, "Adaptive2D Trochoidal Retract");
    let troch_m0 = adaptive_trochoidal_with(&r, &pocket_params(PocketStrategy::Adaptive, 0.0), &AdaptiveTrochParams::default()).expect("troch m0");
    let troch_mf = adaptive_trochoidal_with(&r, &pocket_params(PocketStrategy::Adaptive, RF), &AdaptiveTrochParams::default()).expect("troch mf");
    assert_final_lift_fed(&troch_mf, RF, -0.5, 10.0, "Adaptive2D Trochoidal Microlift");
    let micro_n = |tp: &Toolpath| tp.moves.iter().filter(|m| m.kind == MoveKind::Feed && is_vert_up(m) && m.feed == Some(800.0)).count();
    assert_eq!(micro_n(&troch_m0), micro_n(&troch_mf), "microlifts unchanged");

    // 4. Contour (closed + open chain)
    let cp = ContourParams {
        tool_diameter: 6.0, stock_to_leave: 0.0, clearance_z: 10.0, retract_z: 5.0,
        top_z: 0.0, bottom_z: -5.0, stepdown: 2.5, feed: 800.0, plunge_feed: 200.0,
        climb: true, side: Side::Outside, lead: Lead::None, tolerance: 0.01,
        tab_count: 0, tab_width: 0.0, tab_height: 0.0, entry: Entry3D::Plunge,
        helix_radius: 1.5, blend_turns: false, blend_radius: None, links: links_rf,
        passes: WallPasses::default(),
    };
    assert_final_lift_fed(&contour(&r, &cp).expect("contour"), RF, 1.0, 10.0, "Contour closed");
    let open = contour_along_chain(&[Point2::new(0.0, 0.0), Point2::new(30.0, 0.0)], &cp).expect("open");
    assert_final_lift_fed(&open, RF, -1.5, 10.0, "Contour open");

    // 5. Face (single-level and staged 2-level)
    let f_single = face(&r, &FaceParams {
        tool_diameter: 10.0, stepover: 7.0, clearance_z: 10.0, cut_z: -2.0,
        top_z: None, stepdown: None, feed: 800.0, plunge_feed: 200.0, retract_feed: RF, finish_levels: 0, finish_stepdown: 0.0,
        climb: true, blend_turns: false, blend_radius: None, holes: FaceHoles::Lift, stock_top: None,
    }).expect("face single");
    assert_final_lift_fed(&f_single, RF, 0.0, 10.0, "Face single");
    let f_staged = face(&r, &FaceParams {
        tool_diameter: 10.0, stepover: 7.0, clearance_z: 10.0, cut_z: -4.0,
        top_z: Some(0.0), stepdown: Some(2.0), feed: 800.0, plunge_feed: 200.0, retract_feed: RF, finish_levels: 0, finish_stepdown: 0.0,
        climb: true, blend_turns: false, blend_radius: None, holes: FaceHoles::Lift, stock_top: None,
    }).expect("face staged");
    assert_final_lift_fed(&f_staged, RF, 0.0, 10.0, "Face staged");

    // 6. Slot
    let s_path = slot(Point2::new(0.0, 0.0), Point2::new(30.0, 0.0), &SlotParams {
        tool_diameter: 6.0, width: 10.0, stock_to_leave: 0.0, stepover: 3.0,
        clearance_z: 10.0, retract_z: Some(5.0), top_z: 0.0, bottom_z: -5.0, stepdown: 2.5,
        feed: 800.0, plunge_feed: 200.0, climb: true, entry: Entry3D::Plunge,
        helix_radius: 1.0, blend_turns: false, blend_radius: None, links: links_rf,
    }).expect("slot");
    assert_final_lift_fed(&s_path, RF, -1.5, 10.0, "Slot");

    // 7. Circular
    let circ_path = circular(&r, &CircularParams {
        center: Point2::new(0.0, 0.0), radius: 12.0, tool_diameter: 6.0, stepover: 2.5,
        stock_to_leave: 0.0, clearance_z: 10.0, retract_z: Some(5.0), top_z: 0.0, bottom_z: -5.0,
        stepdown: 2.5, feed: 800.0, plunge_feed: 200.0, climb: true, entry: Entry3D::Plunge,
        helix_radius: 1.5, vault: false, links: links_rf, pass_start: Default::default(),
    }).expect("circular");
    assert_final_lift_fed(&circ_path, RF, -1.5, 10.0, "Circular");

    // 8. Bore
    let bore_path = bore(&BoreParams {
        holes: vec![
            BoreHole { xy: Point2::new(0.0, 0.0), radius: 8.0, top_z: 0.0, floor_z: -5.0, through: false, stepped: false },
            BoreHole { xy: Point2::new(30.0, 0.0), radius: 8.0, top_z: 0.0, floor_z: -5.0, through: false, stepped: false },
        ],
        tool_diameter: 6.0, corner_radius: 0.0, stock_to_leave: 0.0, stepdown: 2.5,
        through_allowance: 0.0, clearance_z: 10.0, retract_z: Some(5.0),
        feed: 800.0, plunge_feed: 200.0, climb: true, tolerance: 0.01,
        entry: Entry3D::Plunge, helix_radius: 1.5, links: links_rf,
    }).expect("bore");
    assert_final_lift_fed(&bore_path, RF, -1.5, 10.0, "Bore");
}

#[test]
fn s3_check_4_contour_keep_down_and_ring_link_modes() {
    let two_islands = Region::new(vec![
        Loop::new(vec![Point2::new(0.0, 0.0), Point2::new(15.0, 0.0), Point2::new(15.0, 15.0), Point2::new(0.0, 15.0)]).expect("l1"),
        Loop::new(vec![Point2::new(30.0, 0.0), Point2::new(45.0, 0.0), Point2::new(45.0, 15.0), Point2::new(30.0, 15.0)]).expect("l2"),
    ]).expect("two islands");

    let make = |pass: Link, rf: f64| ContourParams {
        tool_diameter: 6.0, stock_to_leave: 0.0, clearance_z: 12.0, retract_z: 5.0,
        top_z: 0.0, bottom_z: -4.0, stepdown: 4.0, feed: 800.0, plunge_feed: 200.0,
        climb: true, side: Side::Outside, lead: Lead::None, tolerance: 0.01,
        tab_count: 0, tab_width: 0.0, tab_height: 0.0, entry: Entry3D::Plunge,
        helix_radius: 1.5, blend_turns: false, blend_radius: None,
        links: links_retract(pass, rf), passes: WallPasses::default(),
    };

    let kd0 = contour(&two_islands, &make(Link::KeepDown, 0.0)).expect("kd 0");
    let kdf = contour(&two_islands, &make(Link::KeepDown, RF)).expect("kd rf");
    assert_no_bad_retract_feed(&kdf, RF, "Contour KeepDown");
    let horiz_feeds = |tp: &Toolpath| -> Vec<Move> {
        tp.moves.iter().filter(|m| m.kind == MoveKind::Feed && (m.to.z - m.from.z).abs() <= EPS).copied().collect()
    };
    assert_eq!(horiz_feeds(&kd0), horiz_feeds(&kdf), "keep-down & cutting feeds unchanged");
    assert_final_lift_fed(&kdf, RF, 1.0, 12.0, "Contour KeepDown final");

    for (label, mode) in [("Retract", Link::Retract), ("Clearance", Link::Clearance)] {
        let path = contour(&two_islands, &make(mode, RF)).expect(label);
        assert_no_bad_retract_feed(&path, RF, label);
        let lifts = path.moves.iter().filter(|m| is_fed_retract(m, RF)).count();
        assert!(lifts >= 2, "Contour ring_link {label}: expected >= 2 fed lifts, got {lifts}");
        assert_final_lift_fed(&path, RF, 1.0, 12.0, label);
    }
}
