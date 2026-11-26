//! Journeyman spec S9 synthetic checks (1-10): open sketch chain cut to a
//! chosen side (`left` / `right`), geometry checks, and named refusals.
//!
//! Provenance: Journeyman spec S9 (docs/journeyman/specs/S9-open-sketch-chain-side.md);
//! no vendor parameters, text or code.

mod sim_and_rt;

use flecs_ecs::prelude::World;

use camcraft_app::cl::MoveKind;
use camcraft_app::commands::{
    AddTool, Command, CommandError, CreateOperation, DxfOptions, EditOperationParams,
    GenerateOperation, History, ImportDxf, SetOperationGeometry, SetStock,
};
use camcraft_app::geometry::Point3;
use camcraft_app::geometry::sketch_chains::ChainId;
use camcraft_app::machining2d::{Lead, Side, WallPasses};
use camcraft_app::model;
use camcraft_app::operation::{
    self, ChainOverride, FeatureSel, GeoSel, Height, HeightReference, OpParams, OpType,
    OpenChainSide, SrcRef,
};
use camcraft_app::schema::Location;
use camcraft_app::sketch;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::{Tool, catalog};
use camcraft_app::world::new_world;

const HEAD: &str = "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n";
const TAIL: &str = "0\nENDSEC\n0\nEOF\n";
pub(super) const TOOL10: &str = "EM-1000-4FL";

pub(super) fn run(
    world: &World,
    history: &mut History,
    cmd: Result<impl Command + 'static, CommandError>,
    what: &str,
) {
    history.execute(world, cmd.unwrap_or_else(|e| panic!("{what}: {e}")));
}

pub(super) fn tool10() -> Tool {
    catalog()
        .into_iter()
        .find(|(n, _)| *n == TOOL10)
        .map(|(_, t)| t)
        .expect("10mm catalog tool")
}

pub(super) fn dxf_polyline(layer: &str, pts: &[(f64, f64)], closed: bool) -> String {
    let mut s = format!(
        "{HEAD}0\nLWPOLYLINE\n8\n{layer}\n90\n{}\n70\n{}\n",
        pts.len(),
        if closed { 1 } else { 0 }
    );
    for (x, y) in pts {
        s.push_str(&format!("10\n{x}\n20\n{y}\n"));
    }
    s.push_str(TAIL);
    s
}

pub(super) fn world_with_dxf(dxf: &str, stock: &StockShape, loc: Location) -> (World, History, ChainId) {
    let world = new_world();
    let mut history = History::default();
    run(&world, &mut history, SetStock::new(&world, stock.clone(), loc), "stock");
    run(&world, &mut history, AddTool::new(&world, TOOL10, tool10()), "tool");
    run(
        &world,
        &mut history,
        ImportDxf::from_bytes(&world, None, "part.dxf", dxf.as_bytes(), &DxfOptions::default()),
        "import dxf",
    );
    let (_, sk) = sketch::find(&world, "part").expect("part sketch");
    let cid = sk.data.chains().all().next().expect("chain").id();
    (world, history, cid)
}

pub(super) fn s9_contour(bottom: f64) -> OpParams {
    let mut params = operation::seed_params(OpType::Contour.ordinal());
    if let OpParams::Contour {
        clearance_z,
        retract_z,
        top_z,
        bottom_z,
        stepdown,
        lead,
        tolerance,
        stock_to_leave,
        ..
    } = &mut params
    {
        *clearance_z = Height::of(HeightReference::TopOfStock, 10.0);
        *retract_z = Height::of(HeightReference::TopOfStock, 5.0);
        *top_z = Height::of(HeightReference::TopOfStock, 0.0);
        *bottom_z = Height::of(HeightReference::TopOfStock, bottom);
        *stepdown = bottom.abs().max(1.0);
        *lead = Lead::None;
        *tolerance = 0.005;
        *stock_to_leave = 0.0;
    }
    params
}

fn feed_moves(world: &World, op: &str) -> Vec<camcraft_app::cl::Move> {
    let e = model::find_by_name(world, op).expect("op");
    let tp = model::toolpath_of(world, &e).expect("toolpath");
    tp.path.moves.iter().filter(|m| m.kind == MoveKind::Feed).copied().collect()
}

fn open_ov(chain: ChainId, open_side: OpenChainSide, reverse: bool) -> Vec<ChainOverride> {
    vec![ChainOverride {
        entity: "part".into(),
        chain,
        side: None,
        reverse,
        open_side,
    }]
}

#[test]
fn s9_checks_1_to_7_straight_reverse_stock_l_concave_convex_and_op_side_independence() {
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };
    let line_dxf = dxf_polyline("open", &[(0.0, 0.0), (50.0, 0.0)], false);
    let (world, mut history, cid) = world_with_dxf(&line_dxf, &stock, loc);
    let pick = SrcRef::Selection(vec![GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(cid) }]);

    // Check 1: default open_side = None rides centerline; op_side Inside refuses.
    run(&world, &mut history, CreateOperation::new(&world, "Center", TOOL10, pick.clone(), s9_contour(-2.0)), "center");
    run(&world, &mut history, GenerateOperation::new(&world, "Center"), "gen center");
    let cf = feed_moves(&world, "Center");
    assert_eq!(cf.len(), 1);
    assert!(cf[0].from.y.abs() < 1e-4 && cf[0].to.y.abs() < 1e-4, "{cf:?}");

    let mut p_inside = s9_contour(-2.0);
    if let OpParams::Contour { side, .. } = &mut p_inside {
        *side = Side::Inside;
    }
    run(&world, &mut history, CreateOperation::new(&world, "BadInside", TOOL10, pick.clone(), p_inside), "bad inside");
    let err = GenerateOperation::new(&world, "BadInside").err().expect("Inside without open_side refuses");
    assert!(err.to_string().contains("open sketch chain"), "{err}");

    // Check 2: left, climb on -> y = +5, +x. Also confirm op_side = Inside does not block it.
    let mut p_left = s9_contour(-2.0);
    if let OpParams::Contour { side, chain_overrides, .. } = &mut p_left {
        *side = Side::Inside;
        *chain_overrides = open_ov(cid, OpenChainSide::Left, false);
    }
    run(&world, &mut history, CreateOperation::new(&world, "Left", TOOL10, pick.clone(), p_left), "left");
    run(&world, &mut history, GenerateOperation::new(&world, "Left"), "gen left");
    let lf = feed_moves(&world, "Left");
    assert_eq!(lf.len(), 1);
    assert!((lf[0].from.x - 0.0).abs() < 1e-3 && (lf[0].from.y - 5.0).abs() < 1e-3, "{lf:?}");
    assert!((lf[0].to.x - 50.0).abs() < 1e-3 && (lf[0].to.y - 5.0).abs() < 1e-3, "{lf:?}");

    // Check 3: right, climb on -> y = -5, -x (50 -> 0).
    let mut p_right = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut p_right {
        *chain_overrides = open_ov(cid, OpenChainSide::Right, false);
    }
    run(&world, &mut history, CreateOperation::new(&world, "Right", TOOL10, pick.clone(), p_right), "right");
    run(&world, &mut history, GenerateOperation::new(&world, "Right"), "gen right");
    let rf = feed_moves(&world, "Right");
    assert_eq!(rf.len(), 1);
    assert!((rf[0].from.x - 50.0).abs() < 1e-3 && (rf[0].from.y + 5.0).abs() < 1e-3, "{rf:?}");
    assert!((rf[0].to.x - 0.0).abs() < 1e-3 && (rf[0].to.y + 5.0).abs() < 1e-3, "{rf:?}");

    // Check 4: left + reverse -> direction -x, left is y = -5, travelling -x.
    let mut p_rev = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut p_rev {
        *chain_overrides = open_ov(cid, OpenChainSide::Left, true);
    }
    run(&world, &mut history, CreateOperation::new(&world, "RevLeft", TOOL10, pick.clone(), p_rev), "rev left");
    run(&world, &mut history, GenerateOperation::new(&world, "RevLeft"), "gen rev left");
    let vf = feed_moves(&world, "RevLeft");
    assert_eq!(vf.len(), 1);
    assert!((vf[0].from.x - 50.0).abs() < 1e-3 && (vf[0].from.y + 5.0).abs() < 1e-3, "{vf:?}");
    assert!((vf[0].to.x - 0.0).abs() < 1e-3 && (vf[0].to.y + 5.0).abs() < 1e-3, "{vf:?}");

    // Check 5: stock_to_leave 0.5 + left -> y = +5.5, plus S1 wall passes compatibility.
    let mut p_stl = s9_contour(-2.0);
    if let OpParams::Contour { stock_to_leave, chain_overrides, passes, .. } = &mut p_stl {
        *stock_to_leave = 0.5;
        *chain_overrides = open_ov(cid, OpenChainSide::Left, false);
        *passes = WallPasses { rough_passes: 2, rough_spacing: 1.0, ..WallPasses::default() };
    }
    run(&world, &mut history, CreateOperation::new(&world, "Stl", TOOL10, pick, p_stl), "stl");
    run(&world, &mut history, GenerateOperation::new(&world, "Stl"), "gen stl");
    let sf = feed_moves(&world, "Stl");
    assert_eq!(sf.len(), 2, "two wall passes at y=6.5 then y=5.5: {sf:?}");
    assert!((sf[0].from.y - 6.5).abs() < 1e-3 && (sf[1].from.y - 5.5).abs() < 1e-3, "{sf:?}");

    // Checks 6 & 7: L chain (0,0) -> (40,0) -> (40,30).
    let l_dxf = dxf_polyline("open", &[(0.0, 0.0), (40.0, 0.0), (40.0, 30.0)], false);
    let (l_world, mut l_hist, l_cid) = world_with_dxf(&l_dxf, &stock, loc);
    let l_pick = SrcRef::Selection(vec![GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(l_cid) }]);

    let mut p_l_in = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut p_l_in {
        *chain_overrides = open_ov(l_cid, OpenChainSide::Left, false);
    }
    run(&l_world, &mut l_hist, CreateOperation::new(&l_world, "LIn", TOOL10, l_pick.clone(), p_l_in), "L in");
    run(&l_world, &mut l_hist, GenerateOperation::new(&l_world, "LIn"), "gen L in");
    let lif = feed_moves(&l_world, "LIn");
    assert_eq!(lif.len(), 2, "concave L meets at single corner (35,5): {lif:?}");
    assert!((lif[0].to.x - 35.0).abs() < 1e-3 && (lif[0].to.y - 5.0).abs() < 1e-3);
    assert!((lif[1].to.x - 35.0).abs() < 1e-3 && (lif[1].to.y - 30.0).abs() < 1e-3);

    let mut p_l_out = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut p_l_out {
        *chain_overrides = open_ov(l_cid, OpenChainSide::Right, false);
    }
    run(&l_world, &mut l_hist, CreateOperation::new(&l_world, "LOut", TOOL10, l_pick, p_l_out), "L out");
    run(&l_world, &mut l_hist, GenerateOperation::new(&l_world, "LOut"), "gen L out");
    let lof = feed_moves(&l_world, "LOut");
    assert!((lof[0].from.x - 45.0).abs() < 1e-3 && (lof[0].from.y - 30.0).abs() < 1e-3, "{lof:?}");
    assert!((lof[lof.len() - 1].to.x - 0.0).abs() < 1e-3 && (lof[lof.len() - 1].to.y + 5.0).abs() < 1e-3, "{lof:?}");
    let has_arc = lof.iter().any(|m| {
        m.arc.is_some_and(|a| {
            let (r, _) = a.radius_sweep(m.from, m.to);
            (a.center.x - 40.0).abs() < 0.02 && a.center.y.abs() < 0.02 && (r - 5.0).abs() < 0.02
        })
    });
    assert!(has_arc, "convex L corner should carry R=5 arc about (40,0): {lof:?}");
}

#[test]
fn s9_checks_8_9_10_refusals_folding_crossing_and_closed_open_side() {
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };

    // Check 8: U chain (0,30)->(0,0)->(8,0)->(8,30), D=10, inside (left) -> refused naming chain and d.
    let u_dxf = dxf_polyline("open", &[(0.0, 30.0), (0.0, 0.0), (8.0, 0.0), (8.0, 30.0)], false);
    let (uw, mut uh, u_cid) = world_with_dxf(&u_dxf, &stock, loc);
    let mut pu = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut pu {
        *chain_overrides = open_ov(u_cid, OpenChainSide::Left, false);
    }
    run(&uw, &mut uh, CreateOperation::new(&uw, "U", TOOL10, SrcRef::Entity("part".into()), pu), "U");
    let err = GenerateOperation::new(&uw, "U").err().expect("U folds");
    let msg = err.to_string();
    assert!(msg.contains("part") && msg.contains("5.000") && msg.contains("folds"), "{msg}");

    // Check 9: self-crossing chain (four segments through one point) -> refused by name.
    let x_dxf = dxf_polyline("open", &[(0.0, 0.0), (20.0, 20.0), (20.0, 0.0), (0.0, 20.0), (0.0, 10.0)], false);
    let (xw, mut xh, x_cid) = world_with_dxf(&x_dxf, &stock, loc);
    let mut px = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut px {
        *chain_overrides = open_ov(x_cid, OpenChainSide::Left, false);
    }
    run(&xw, &mut xh, CreateOperation::new(&xw, "X", TOOL10, SrcRef::Entity("part".into()), px), "X");
    let err = GenerateOperation::new(&xw, "X").err().expect("crossing chain refused");
    let msg = err.to_string();
    assert!(msg.contains("part") && msg.contains("crosses or touches itself"), "{msg}");

    // Check 10: closed rectangle chain with open_side set -> refused when the override is set!
    let r_dxf = dxf_polyline("cut", &[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)], true);
    let (rw, mut rh, r_cid) = world_with_dxf(&r_dxf, &stock, loc);
    let mut pr = s9_contour(-2.0);
    let bad_ov = open_ov(r_cid, OpenChainSide::Left, false);
    if let OpParams::Contour { chain_overrides, .. } = &mut pr {
        *chain_overrides = bad_ov.clone();
    }
    let err = CreateOperation::new(&rw, "BadCreate", TOOL10, SrcRef::Entity("part".into()), pr.clone())
        .err()
        .expect("closed chain open_side refused at CreateOperation");
    assert!(err.to_string().contains("closed sketch chain") && err.to_string().contains("left"), "{err}");

    run(&rw, &mut rh, CreateOperation::new(&rw, "Rect", TOOL10, SrcRef::Entity("part".into()), s9_contour(-2.0)), "Rect");
    let err = EditOperationParams::new(&rw, "Rect", pr)
        .err()
        .expect("closed chain open_side refused at EditOperationParams");
    assert!(err.to_string().contains("closed sketch chain") && err.to_string().contains("left"), "{err}");

    let err = SetOperationGeometry::with_chain_overrides(
        &rw,
        "Rect",
        vec![GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(r_cid) }],
        bad_ov,
    )
    .err()
    .expect("closed chain open_side refused at SetOperationGeometry");
    assert!(err.to_string().contains("closed sketch chain") && err.to_string().contains("left"), "{err}");
}

#[test]
fn s9_narrow_u_dropped_wall_pass_note_and_viewport_preview_offset() {
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };

    // Inside a 13 mm wide U with R=5 and 3 rough passes at 1 mm spacing (7, 6, 5 mm),
    // the outermost 7 mm pass is dropped before 6 and 5 fit; the note must survive.
    let u_dxf = dxf_polyline("open", &[(0.0, 30.0), (0.0, 0.0), (13.0, 0.0), (13.0, 30.0)], false);
    let (uw, mut uh, u_cid) = world_with_dxf(&u_dxf, &stock, loc);
    let mut pu = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, passes, .. } = &mut pu {
        *chain_overrides = open_ov(u_cid, OpenChainSide::Left, false);
        *passes = WallPasses { rough_passes: 3, rough_spacing: 1.0, ..WallPasses::default() };
    }
    run(&uw, &mut uh, CreateOperation::new(&uw, "UDrop", TOOL10, SrcRef::Entity("part".into()), pu), "UDrop");
    run(&uw, &mut uh, GenerateOperation::new(&uw, "UDrop"), "gen UDrop");
    let u_feeds = feed_moves(&uw, "UDrop");
    let mut floors: Vec<f64> = Vec::new();
    for m in u_feeds.iter().filter(|m| (m.from.y - m.to.y).abs() < 1e-9 && m.from.y < 10.0) {
        if floors.last().is_none_or(|y| (y - m.from.y).abs() > 1e-3) {
            floors.push(m.from.y);
        }
    }
    assert_eq!(floors.len(), 2, "two passes cross the U floor: {floors:?}");
    assert!((floors[0] - 6.0).abs() < 1e-3 && (floors[1] - 5.0).abs() < 1e-3, "{floors:?}");
    let ue = model::find_by_name(&uw, "UDrop").expect("UDrop op");
    let notes = model::toolpath_of(&uw, &ue).expect("UDrop tp").path.notes.join("\n");
    assert!(notes.contains("a wall pass at 7.000 mm from the open run does not fit"), "{notes}");

    // Viewport preview (`chains_of`) draws the offset tool-centre run when `open_side` is set.
    let line_dxf = dxf_polyline("open", &[(0.0, 0.0), (50.0, 0.0)], false);
    let (lw, mut lh, l_cid) = world_with_dxf(&line_dxf, &stock, loc);
    let mut pl = s9_contour(-2.0);
    if let OpParams::Contour { stock_to_leave, chain_overrides, .. } = &mut pl {
        *stock_to_leave = 0.5;
        *chain_overrides = open_ov(l_cid, OpenChainSide::Left, false);
    }
    run(&lw, &mut lh, CreateOperation::new(&lw, "Prev", TOOL10, SrcRef::Entity("part".into()), pl), "Prev");
    let pe = model::find_by_name(&lw, "Prev").expect("Prev op");
    let pop = model::operation_of(&lw, &pe).expect("Prev op data");
    let chains = operation::chains_of(&lw, &pop);
    assert_eq!(chains.len(), 1, "one preview run: {chains:?}");
    assert!(chains[0].iter().all(|p| (p.y - 5.5).abs() < 1e-3), "preview offset at y=5.5: {:?}", chains[0]);
}
