//! Journeyman spec S10 synthetic checks 6 and 8: stock simulator 45-degree
//! open-edge chamfer, project file roundtrip / legacy load, and MCP surface.
//!
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! no vendor parameters, text or code.

use std::path::PathBuf;

use serde_json::{Value, json};

use camcraft_app::commands::{CreateOperation, GenerateOperation};
use camcraft_app::geometry::Point3;
use camcraft_app::mcp::Server;
use camcraft_app::model;
use camcraft_app::operation::{
    self, ChainOverride, FeatureSel, GeoSel, OpParams, OpStatus, OpenChainSide, SrcRef,
};
use camcraft_app::project;
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::stock_sim::simulate_document;
use camcraft_app::world::new_world;

use super::{CONE90, dxf_polylines, run, s10_chamfer, world_with_dxf};

fn mcp_call(server: &mut Server, id: i64, name: &str, args: Value) -> (bool, Value) {
    let msg = json!({
        "jsonrpc": "2.0",
        "id": id,
        "method": "tools/call",
        "params": { "name": name, "arguments": args }
    });
    let reply = server.handle(&msg).expect("mcp reply");
    let result = &reply["result"];
    let is_err = result["isError"] == json!(true);
    let body = if is_err {
        result["content"][0]["text"].clone()
    } else {
        result["structuredContent"].clone()
    };
    (is_err, body)
}

#[test]
fn s10_check_6_stock_simulator_45deg_1mm_open_edge_chamfer() {
    // Block from (0,0) to (60,40), 10 mm thick, top at 0 -> centered at (30, 20, -10).
    // Sketch origin sits at (30, 20, 0), so DXF (-30, 20)->(30, 20) is world (0, 40)->(60, 40).
    let stock = StockShape::Box { dx: 60.0, dy: 40.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(30.0, 20.0, -10.0), ..Location::default() };
    let edge_dxf = dxf_polylines(&[("edge", &[(-30.0, 20.0), (30.0, 20.0)], false)]);
    let (world, mut history, ids) = world_with_dxf(&edge_dxf, &stock, loc);

    let mut params = s10_chamfer(1.0, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut params {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: ids[0],
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
            straight_ramp: None,
        }];
    }
    run(&world, &mut history, CreateOperation::new(&world, "EdgeCh", CONE90, SrcRef::Entity("part".into()), params), "create");
    run(&world, &mut history, GenerateOperation::new(&world, "EdgeCh"), "generate");

    let sim = simulate_document(&world).expect("simulate open-edge chamfer");
    assert_eq!(sim.dev.gouge_cells, 0, "no gouges");

    // Nothing further than 1 mm from the edge (y <= 38.5) is touched: stays at z = 0.
    let untouched: Vec<f64> = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 2.0 && v.x <= 58.0 && v.y >= 2.0 && v.y <= 38.5)
        .map(|v| v.z)
        .collect();
    assert!(!untouched.is_empty());
    assert!(
        untouched.iter().all(|&z| z.abs() < 0.1),
        "interior untouched at z=0: min={}",
        untouched.iter().copied().fold(0.0_f64, f64::min)
    );

    // Along the +y edge (y in [39.15, 39.85], x in [4, 56]): 45-degree chamfer face z ≈ 39.0 - y,
    // and nothing cut below z = -1.0.
    let bevel: Vec<_> = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 4.0 && v.x <= 56.0 && v.y >= 39.15 && v.y <= 39.85)
        .collect();
    assert!(!bevel.is_empty(), "expected mesh verts along the 1mm chamfer face");
    assert!(bevel.iter().any(|v| v.z < -0.2), "chamfer face must remove material: {bevel:?}");
    assert!(
        bevel.iter().all(|v| v.z >= -1.05 && (v.z - (39.0 - v.y)).abs() < 0.25),
        "bevel must match 45-deg plane z = 39 - y and stay >= -1: {bevel:?}"
    );
    let min_top_z = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 2.0 && v.x <= 58.0 && v.y >= 2.0 && v.y <= 39.85)
        .map(|v| v.z)
        .fold(f64::INFINITY, f64::min);
    assert!(min_top_z >= -1.05, "nothing below z=-1 should be touched: min_top_z={min_top_z}");
}

#[test]
fn s10_check_8_project_roundtrip_legacy_load_and_mcp_surface() {
    let mut dir = std::env::temp_dir();
    dir.push(format!("camcraft-s10-rt-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp dir");
    let dxf_path: PathBuf = dir.join("profile.dxf");
    let dxf = dxf_polylines(&[
        ("cut", &[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)], true),
        ("open", &[(0.0, 30.0), (40.0, 30.0)], false),
    ]);
    std::fs::write(&dxf_path, &dxf).expect("write dxf");

    let mut s = Server::new();
    let init = json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "initialize",
        "params": { "protocolVersion": "2025-06-18" }
    });
    s.handle(&init).expect("init");
    let (err, r) = mcp_call(&mut s, 1, "set_stock", json!({ "box": { "dx": 120.0, "dy": 120.0, "dz": 10.0 } }));
    assert!(!err, "{r}");
    let (err, r) = mcp_call(&mut s, 2, "add_tool", json!({ "catalog": CONE90 }));
    assert!(!err, "{r}");
    let (err, r) = mcp_call(&mut s, 3, "import_dxf", json!({ "path": dxf_path.to_string_lossy() }));
    assert!(!err, "{r}");
    let (err, feat) = mcp_call(&mut s, 4, "list_features", json!({ "model": "profile" }));
    assert!(!err, "{feat}");
    let chains = feat["chains"].as_array().expect("chains");
    let closed_id = chains.iter().find(|c| c["closed"] == json!(true)).unwrap()["id"].as_str().unwrap().to_owned();
    let open_id = chains.iter().find(|c| c["closed"] == json!(false)).unwrap()["id"].as_str().unwrap().to_owned();

    let (err, r) = mcp_call(&mut s, 5, "add_operation", json!({
        "kind": "2D Chamfer",
        "tool": CONE90,
        "source": "profile",
        "name": "Ch",
        "params": &project::params_to_json(&s10_chamfer(0.5, 45.0))["chamfer2_d"]
    }));
    assert!(!err, "{r}");
    let (err, geom) = mcp_call(&mut s, 6, "set_operation_geometry", json!({
        "operation": "Ch",
        "picks": [
            { "model": "profile", "id": closed_id },
            { "model": "profile", "id": open_id, "open_side": "left" }
        ]
    }));
    assert!(!err, "{geom}");
    assert_eq!(geom["chain_overrides"][0]["open_side"], json!("left"));
    let (err, view) = mcp_call(&mut s, 7, "get_operation_geometry", json!({ "operation": "Ch" }));
    assert!(!err, "{view}");
    assert_eq!(view["chain_overrides"][0]["open_side"], json!("left"));
    let (err, gen_res) = mcp_call(&mut s, 8, "generate", json!({ "name": "Ch" }));
    assert!(!err, "{gen_res}");

    // R1 [M1]: MCP edit_operation clearing chain_overrides back to [] on Chamfer2D must succeed.
    let (err, edit_res) = mcp_call(&mut s, 9, "edit_operation", json!({
        "name": "Ch",
        "params": { "chain_overrides": [] }
    }));
    assert!(!err, "edit_operation clearing Chamfer2D chain_overrides failed: {edit_res}");
    let (err, view2) = mcp_call(&mut s, 10, "get_operation_geometry", json!({ "operation": "Ch" }));
    assert!(!err, "{view2}");
    assert_eq!(view2["chain_overrides"], json!([]));

    // Project save / reopen roundtrip keeps status Ok and overrides intact.
    let stock = StockShape::Box { dx: 120.0, dy: 120.0, dz: 10.0 };
    let (w, mut h, ids) = world_with_dxf(&dxf, &stock, Location::default());
    let open_cid = ids[1];
    let mut p = s10_chamfer(0.5, 45.0);
    if let OpParams::Chamfer2D { chain_overrides, .. } = &mut p {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: open_cid,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
            straight_ramp: None,
        }];
    }
    let picks = SrcRef::Selection(vec![
        GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(ids[0]) },
        GeoSel { entity: "part".into(), feature: FeatureSel::SketchChain(open_cid) },
    ]);
    run(&w, &mut h, CreateOperation::new(&w, "ChOp", CONE90, picks, p), "create");
    run(&w, &mut h, GenerateOperation::new(&w, "ChOp"), "gen");
    let proj = dir.join("s10.camproj");
    project::save(&w, &proj).expect("save");
    let text = std::fs::read_to_string(&proj).expect("read");
    let fresh = new_world();
    project::open(&fresh, &text).expect("open");
    let e = model::find_by_name(&fresh, "ChOp").expect("ChOp");
    assert_eq!(operation::status_of(&fresh, &e), OpStatus::Ok);
    let OpParams::Chamfer2D { chain_overrides, .. } = model::operation_of(&fresh, &e).unwrap().params else {
        panic!("Chamfer2D")
    };
    assert_eq!(chain_overrides.len(), 1);
    assert_eq!(chain_overrides[0].open_side, OpenChainSide::Left);

    // Pre-S10 Chamfer2D JSON (no `chain_overrides` key) loads unchanged and omits it on save.
    let rect_only = dxf_polylines(&[("cut", &[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)], true)]);
    let (wl, mut hl, _) = world_with_dxf(&rect_only, &stock, Location::default());
    run(&wl, &mut hl, CreateOperation::new(&wl, "LegCh", CONE90, SrcRef::Entity("part".into()), s10_chamfer(0.5, 45.0)), "leg");
    run(&wl, &mut hl, GenerateOperation::new(&wl, "LegCh"), "gen leg");
    let leg_proj = dir.join("s10_leg.camproj");
    project::save(&wl, &leg_proj).expect("save leg");
    let leg_text = std::fs::read_to_string(&leg_proj).expect("read leg");
    assert!(!leg_text.contains("\"chain_overrides\""), "empty chain_overrides omitted: {leg_text}");
    let fresh_leg = new_world();
    project::open(&fresh_leg, &leg_text).expect("open leg");
    assert_eq!(operation::status_of(&fresh_leg, &model::find_by_name(&fresh_leg, "LegCh").unwrap()), OpStatus::Ok);
}
