//! Journeyman spec S10 synthetic checks (1-5, 7): 2D Chamfer on closed and open
//! chains of a DXF profile, angle depth, nested auto sides, and named refusals.
//!
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! no vendor parameters, text or code.

mod sim_and_rt;

use flecs_ecs::prelude::World;

use camcraft_app::cl::MoveKind;
use camcraft_app::commands::{
    AddTool, Command, CommandError, CreateOperation, DxfOptions, GenerateOperation, History,
    ImportDxf, ImportModel, SetStock,
};
use camcraft_app::geometry::sketch_chains::ChainId;
use camcraft_app::geometry::{Point3, cube};
use camcraft_app::machining2d::Side;
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
pub(super) const CONE90: &str = "CM-0600-90";
const CONE120: &str = "CM-0600-120";

pub(super) type ChainSpec<'a> = (&'a str, &'a [(f64, f64)], bool);

pub(super) fn run(
    world: &World,
    history: &mut History,
    cmd: Result<impl Command + 'static, CommandError>,
    what: &str,
) {
    history.execute(world, cmd.unwrap_or_else(|e| panic!("{what}: {e}")));
}

fn cone_tool(tip_angle: f64) -> Tool {
    let mut t = catalog()
        .into_iter()
        .find(|(n, _)| *n == CONE90)
        .map(|(_, t)| t)
        .expect("CM-0600-90 catalog tool");
    t.tip_angle = tip_angle;
    t
}

pub(super) fn dxf_polylines(chains: &[ChainSpec<'_>]) -> String {
    let mut s = String::from(HEAD);
    for (layer, pts, closed) in chains {
        s.push_str(&format!(
            "0\nLWPOLYLINE\n8\n{layer}\n90\n{}\n70\n{}\n",
            pts.len(),
            if *closed { 1 } else { 0 }
        ));
        for (x, y) in *pts {
            s.push_str(&format!("10\n{x}\n20\n{y}\n"));
        }
    }
    s.push_str(TAIL);
    s
}

pub(super) fn world_with_dxf(dxf: &str, stock: &StockShape, loc: Location) -> (World, History, Vec<ChainId>) {
    let world = new_world();
    let mut history = History::default();
    run(&world, &mut history, SetStock::new(&world, stock.clone(), loc), "stock");
    run(&world, &mut history, AddTool::new(&world, CONE90, cone_tool(90.0)), "cone90");
    run(&world, &mut history, AddTool::new(&world, CONE120, cone_tool(120.0)), "cone120");
    run(
        &world,
        &mut history,
        ImportDxf::from_bytes(&world, None, "part.dxf", dxf.as_bytes(), &DxfOptions::default()),
        "import dxf",
    );
    let (_, sk) = sketch::find(&world, "part").expect("part sketch");
    let ids = sk.data.chains().all().map(|c| c.id()).collect();
    (world, history, ids)
}

pub(super) fn s10_chamfer(width: f64, angle_deg: f64) -> OpParams {
    let mut params = operation::seed_params(OpType::Chamfer2D.ordinal());
    if let OpParams::Chamfer2D {
        width: w,
        angle_deg: a,
        clearance_z,
        retract_z,
        top_z,
        side,
        ..
    } = &mut params
    {
        *w = width;
        *a = angle_deg;
        *clearance_z = Height::of(HeightReference::TopOfStock, 10.0);
        *retract_z = Some(Height::of(HeightReference::TopOfStock, 5.0));
        *top_z = Height::of(HeightReference::TopOfStock, 0.0);
        *side = Side::Auto;
    }
    params
}

fn feed_moves(world: &World, op: &str) -> Vec<camcraft_app::cl::Move> {
    let e = model::find_by_name(world, op).expect("op");
    let tp = model::toolpath_of(world, &e).expect("toolpath");
    tp.path.moves.iter().filter(|m| m.kind == MoveKind::Feed).copied().collect()
}

#[test]
fn s10_checks_2_3_4_5_7_closed_open_refusal_angle_and_nested_auto() {
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };

    // Check 2: Closed rectangle 40 x 20, top 0, width 0.5, angle 45, 90-deg D6 chamfer mill
    // matches a picked rectangle loop of a solid at z = -0.5.
    let rect_dxf = dxf_polylines(&[("cut", &[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)], true)]);
    let (w2, mut h2, _) = world_with_dxf(&rect_dxf, &stock, loc);
    let p2 = s10_chamfer(0.5, 45.0);
    run(&w2, &mut h2, CreateOperation::new(&w2, "ChRect", CONE90, SrcRef::Entity("part".into()), p2.clone()), "chrect");
    run(&w2, &mut h2, GenerateOperation::new(&w2, "ChRect"), "gen chrect");
    let dxf_moves = model::toolpath_of(&w2, &model::find_by_name(&w2, "ChRect").unwrap()).unwrap().path.moves.clone();
    let dxf_feeds = feed_moves(&w2, "ChRect");
    assert_eq!(dxf_feeds.len(), 4);
    assert!(dxf_feeds.iter().all(|m| (m.to.z + 0.5).abs() < 1e-6), "{dxf_feeds:?}");

    let sw = new_world();
    let mut sh = History::default();
    run(&sw, &mut sh, SetStock::new(&sw, stock.clone(), loc), "solid stock");
    run(&sw, &mut sh, AddTool::new(&sw, CONE90, cone_tool(90.0)), "cone");
    let brick = cube(Point3::new(0.0, 0.0, -10.0), Point3::new(40.0, 20.0, 0.0));
    run(&sw, &mut sh, Ok::<_, CommandError>(ImportModel::from_mesh("brick", brick)), "brick");
    run(&sw, &mut sh, CreateOperation::new(&sw, "SolidCh", CONE90, SrcRef::Entity("brick".into()), p2), "solid ch");
    run(&sw, &mut sh, GenerateOperation::new(&sw, "SolidCh"), "gen solid ch");
    let solid_feeds = feed_moves(&sw, "SolidCh");
    assert_eq!(dxf_feeds.len(), solid_feeds.len());
    assert!(dxf_moves.iter().filter(|m| m.kind == MoveKind::Plunge).count() == 1);
    let signed_area = |ms: &[camcraft_app::cl::Move]| -> f64 {
        ms.iter().map(|m| m.from.x * m.to.y - m.to.x * m.from.y).sum::<f64>() * 0.5
    };
    assert!((signed_area(&dxf_feeds) - signed_area(&solid_feeds)).abs() < 1e-4, "same boss CW loop");

    // Checks 3 & 4: Open straight chain (0,0)->(40,0), left (+x), right (-x), none (refused).
    let line_dxf = dxf_polylines(&[("open", &[(0.0, 0.0), (40.0, 0.0)], false)]);
    let (w3, mut h3, ids3) = world_with_dxf(&line_dxf, &stock, loc);
    let open_id = ids3[0];

    let mut p_left = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p_left {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: open_id,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
        }];
    }
    run(&w3, &mut h3, CreateOperation::new(&w3, "OpenL", CONE90, SrcRef::Entity("part".into()), p_left), "open L");
    run(&w3, &mut h3, GenerateOperation::new(&w3, "OpenL"), "gen open L");
    let lf = feed_moves(&w3, "OpenL");
    assert_eq!(lf.len(), 1);
    assert!((lf[0].from.x - 0.0).abs() < 1e-4 && lf[0].from.y.abs() < 1e-4 && (lf[0].from.z + 0.5).abs() < 1e-6);
    assert!((lf[0].to.x - 40.0).abs() < 1e-4 && lf[0].to.y.abs() < 1e-4 && (lf[0].to.z + 0.5).abs() < 1e-6);

    let mut p_right = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p_right {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: open_id,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Right,
        }];
    }
    run(&w3, &mut h3, CreateOperation::new(&w3, "OpenR", CONE90, SrcRef::Entity("part".into()), p_right), "open R");
    run(&w3, &mut h3, GenerateOperation::new(&w3, "OpenR"), "gen open R");
    let rf = feed_moves(&w3, "OpenR");
    assert_eq!(rf.len(), 1);
    assert!((rf[0].from.x - 40.0).abs() < 1e-4 && (rf[0].to.x - 0.0).abs() < 1e-4 && (rf[0].to.z + 0.5).abs() < 1e-6);

    // Check 4: open_side none refused by name (both under Auto and under Inside, never citing Contour's "choose Side 'Auto'").
    run(&w3, &mut h3, CreateOperation::new(&w3, "OpenNone", CONE90, SrcRef::Entity("part".into()), s10_chamfer(0.5, 45.0)), "open none");
    let err = GenerateOperation::new(&w3, "OpenNone").err().expect("open_side none must refuse");
    let msg = err.to_string();
    assert!(msg.contains("part") && msg.contains("open_side"), "{msg}");

    let mut p_inside_none = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { side, .. } = &mut p_inside_none {
        *side = Side::Inside;
    }
    run(&w3, &mut h3, CreateOperation::new(&w3, "OpenNoneInside", CONE90, SrcRef::Entity("part".into()), p_inside_none), "open none inside");
    let err = GenerateOperation::new(&w3, "OpenNoneInside").err().expect("open_side none under Inside must refuse");
    let msg = err.to_string();
    assert!(msg.contains("2D Chamfer") && msg.contains("open_side") && !msg.contains("choose Side 'Auto'"), "{msg}");

    // Check 5: angle 30 with 120-degree chamfer mill, width 0.5 -> z = -0.2887 (to 4 places).
    let mut p30 = s10_chamfer(0.5, 30.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p30 {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: open_id,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
        }];
    }
    run(&w3, &mut h3, CreateOperation::new(&w3, "Ang30", CONE120, SrcRef::Entity("part".into()), p30), "ang30");
    run(&w3, &mut h3, GenerateOperation::new(&w3, "Ang30"), "gen ang30");
    let f30 = feed_moves(&w3, "Ang30");
    assert_eq!(f30.len(), 1);
    assert!((f30[0].to.z - (-0.2887)).abs() < 1e-4, "z={}", f30[0].to.z);

    // Check 7: Two closed chains, one inside the other, Auto side:
    // outer loop rides CW (boss) first, inner loop rides CCW (bore) second.
    let nested_dxf = dxf_polylines(&[
        ("cut", &[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)], true),
        ("cut", &[(10.0, 10.0), (30.0, 10.0), (30.0, 30.0), (10.0, 30.0)], true),
    ]);
    let (w7, mut h7, _) = world_with_dxf(&nested_dxf, &stock, loc);
    run(&w7, &mut h7, CreateOperation::new(&w7, "Nested", CONE90, SrcRef::Entity("part".into()), s10_chamfer(0.5, 45.0)), "nested");
    run(&w7, &mut h7, GenerateOperation::new(&w7, "Nested"), "gen nested");
    let nf = feed_moves(&w7, "Nested");
    assert_eq!(nf.len(), 8);
    assert!(signed_area(&nf[..4]) < -1000.0, "outer loop rides CW (boss): {:?}", &nf[..4]);
    assert!(signed_area(&nf[4..]) > 300.0, "inner loop rides CCW (bore): {:?}", &nf[4..]);

    // Refusal doors: empty DXF has no chains, missing layer is named, and closed chain open_side is refused.
    let empty_dxf = format!("{HEAD}{TAIL}");
    let import_err = ImportDxf::from_bytes(&w3, None, "empty.dxf", empty_dxf.as_bytes(), &DxfOptions::default())
        .err()
        .expect("empty DXF refused");
    assert!(import_err.to_string().contains("no line, arc or circle"), "{import_err}");

    let missing_layer = SrcRef::Selection(vec![GeoSel {
        entity: "part".into(),
        feature: FeatureSel::SketchLayer("missing".into()),
    }]);
    run(&w3, &mut h3, CreateOperation::new(&w3, "MissLayer", CONE90, missing_layer, s10_chamfer(0.5, 45.0)), "miss layer");
    let err = GenerateOperation::new(&w3, "MissLayer").err().expect("missing layer refused");
    assert!(err.to_string().contains("part") && err.to_string().contains("missing"), "{err}");

    let (_, sk2) = sketch::find(&w2, "part").expect("rect sketch");
    let rect_cid = sk2.data.chains().all().next().expect("rect chain").id();
    let mut p_closed_os = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p_closed_os {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: rect_cid,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
        }];
    }
    let err = CreateOperation::new(&w2, "BadClosedOs", CONE90, SrcRef::Entity("part".into()), p_closed_os)
        .err()
        .expect("closed chain open_side refused on Chamfer2D");
    assert!(err.to_string().contains("closed sketch chain") && err.to_string().contains("left"), "{err}");

    // Existing tool congruence doors stay enforced on DXF profiles.
    let mut p_bad_angle = s10_chamfer(0.5, 30.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p_bad_angle {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: open_id,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
        }];
    }
    run(&w3, &mut h3, CreateOperation::new(&w3, "BadCone", CONE90, SrcRef::Entity("part".into()), p_bad_angle), "bad cone");
    let err = GenerateOperation::new(&w3, "BadCone").err().expect("90-deg tool on 30-deg chamfer refused");
    assert!(err.to_string().contains("120-degree cone"), "{err}");
}

#[test]
fn s10_open_chains_ride_in_profile_order_regardless_of_pick_order() {
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };
    let dxf = dxf_polylines(&[
        ("c", &[(0.0, -40.0), (40.0, -40.0), (40.0, -20.0), (0.0, -20.0)], true),
        ("c", &[(0.0, 0.0), (40.0, 0.0)], false),
        ("c", &[(0.0, 27.0), (40.0, 27.0)], false),
    ]);
    let (w, mut h, ids) = world_with_dxf(&dxf, &stock, loc);
    let (closed, open, want_ys) = {
        let (_, sk) = sketch::find(&w, "part").expect("part");
        let chains = sk.data.chains();
        let closed: Vec<ChainId> = ids.iter().copied().filter(|c| chains.find(*c).is_some_and(|k| k.is_closed())).collect();
        let open: Vec<ChainId> = ids.iter().copied().filter(|c| !closed.contains(c)).collect();
        let ys: Vec<f64> = open.iter().map(|c| chains.find(*c).expect("open").sample(0.05)[0].y).collect();
        (closed, open, ys)
    };
    assert_eq!((closed.len(), open.len()), (1, 2));

    let mut params = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut params {
        *chain_overrides = open
            .iter()
            .map(|&chain| ChainOverride {
                entity: "part".into(),
                chain,
                side: None,
                reverse: false,
                open_side: OpenChainSide::Left,
            })
            .collect();
    }
    // Pick open chains in reversed order ahead of the closed loop.
    let picks = SrcRef::Selection(vec![
        GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(open[1]) },
        GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(open[0]) },
        GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(closed[0]) },
    ]);
    run(&w, &mut h, CreateOperation::new(&w, "Ord", CONE90, picks, params), "Ord");
    run(&w, &mut h, GenerateOperation::new(&w, "Ord"), "gen Ord");
    let feeds = feed_moves(&w, "Ord");
    assert_eq!(feeds.len(), 6, "4 closed moves + 2 open moves: {feeds:?}");
    assert!(feeds[..4].iter().all(|m| m.from.y < -19.0 && m.to.y < -19.0), "closed loop first: {:?}", &feeds[..4]);
    assert!((feeds[4].from.y - want_ys[0]).abs() < 1e-3 && (feeds[4].to.y - want_ys[0]).abs() < 1e-3, "first open by profile order: {:?}", feeds[4]);
    assert!((feeds[5].from.y - want_ys[1]).abs() < 1e-3 && (feeds[5].to.y - want_ys[1]).abs() < 1e-3, "second open by profile order: {:?}", feeds[5]);
}
