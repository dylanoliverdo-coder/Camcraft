//! Journeyman spec S9 synthetic checks 11 and 12: stock simulator cut band,
//! project file roundtrip / legacy load, and MCP `open_side` surface.
//!
//! Provenance: Journeyman spec S9 (docs/journeyman/specs/S9-open-sketch-chain-side.md);
//! no vendor parameters, text or code.

use std::path::PathBuf;

use serde_json::{Value, json};

use camcraft_app::commands::{CreateOperation, GenerateOperation};
use camcraft_app::geometry::Point3;
use camcraft_app::mcp::Server;
use camcraft_app::model;
use camcraft_app::operation::{self, ChainOverride, OpParams, OpStatus, OpenChainSide, SrcRef};
use camcraft_app::project;
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::stock_sim::simulate_document;
use camcraft_app::world::new_world;

use super::{TOOL10, dxf_polyline, run, s9_contour, world_with_dxf};

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
fn s9_check_11_stock_simulator_removes_10mm_band_on_plus_y_only() {
    // 60 x 40 x 10 mm block, top at z = 0, centered at (25, 0) -> x in [-5, 55], y in [-20, 20], z in [-10, 0].
    // Sketch origin sits at stock center-top (25, 0, 0), so DXF (-25, 0)->(25, 0) lies along world y = 0, x = 0..50.
    let stock = StockShape::Box { dx: 60.0, dy: 40.0, dz: 10.0 };
    let loc = Location { pos: Point3::new(25.0, 0.0, -10.0), ..Location::default() };
    let line_dxf = dxf_polyline("open", &[(-25.0, 0.0), (25.0, 0.0)], false);
    let (world, mut history, cid) = world_with_dxf(&line_dxf, &stock, loc);

    let mut params = s9_contour(-3.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut params {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: cid,
            side: None,
            reverse: false,
            open_side: OpenChainSide::Left,
        }];
    }
    run(
        &world,
        &mut history,
        CreateOperation::new(&world, "Band", TOOL10, SrcRef::Entity("part".into()), params),
        "create",
    );
    run(&world, &mut history, GenerateOperation::new(&world, "Band"), "generate");

    let sim = simulate_document(&world).expect("simulate open-sided contour");
    assert_eq!(sim.dev.gouge_cells, 0, "no gouges");

    // Inside the 10 mm band on +y (y in [1.5, 8.5], x in [5, 45]): cut down to z = -3.
    let cut_band: Vec<f64> = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 5.0 && v.x <= 45.0 && v.y >= 1.5 && v.y <= 8.5)
        .map(|v| v.z)
        .collect();
    assert!(!cut_band.is_empty(), "expected mesh verts in cut band");
    assert!(
        cut_band.iter().all(|&z| (z - (-3.0)).abs() < 0.2),
        "cut band should sit at z=-3: {cut_band:?}"
    );

    // Material at y < 0 (y in [-18, -1.5], x in [5, 45]): untouched at z = 0.
    let standing_neg: Vec<f64> = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 5.0 && v.x <= 45.0 && v.y <= -1.5 && v.y >= -18.0)
        .map(|v| v.z)
        .collect();
    assert!(!standing_neg.is_empty(), "expected mesh verts in y < 0");
    assert!(
        standing_neg.iter().all(|&z| z.abs() < 0.2),
        "y < 0 material must remain standing at z=0: {standing_neg:?}"
    );

    // Beyond the 10 mm band (y >= 11.5): untouched at z = 0.
    let standing_far: Vec<f64> = sim
        .surface
        .verts
        .iter()
        .filter(|v| v.x >= 5.0 && v.x <= 45.0 && v.y >= 11.5 && v.y <= 18.0)
        .map(|v| v.z)
        .collect();
    assert!(!standing_far.is_empty());
    assert!(
        standing_far.iter().all(|&z| z.abs() < 0.2),
        "y > 10 material must stay at z=0: {standing_far:?}"
    );
}

#[test]
fn s9_check_12_project_roundtrip_legacy_load_and_mcp_surface() {
    let mut dir = std::env::temp_dir();
    dir.push(format!("camcraft-s9-rt-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp dir");
    let dxf_path: PathBuf = dir.join("line.dxf");
    let line_dxf = dxf_polyline("open", &[(0.0, 0.0), (50.0, 0.0)], false);
    std::fs::write(&dxf_path, &line_dxf).expect("write dxf");

    let mut s = Server::new();
    let init = json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "initialize",
        "params": { "protocolVersion": "2025-06-18" }
    });
    s.handle(&init).expect("init");
    let (err, r) = mcp_call(&mut s, 1, "set_stock", json!({ "box": { "dx": 100.0, "dy": 80.0, "dz": 10.0 } }));
    assert!(!err, "{r}");
    let (err, r) = mcp_call(&mut s, 2, "add_tool", json!({ "catalog": "EM-0400-2FL" }));
    assert!(!err, "{r}");
    let (err, r) = mcp_call(&mut s, 3, "import_dxf", json!({ "path": dxf_path.to_string_lossy() }));
    assert!(!err, "{r}");
    let (err, feat) = mcp_call(&mut s, 4, "list_features", json!({ "model": "line" }));
    assert!(!err, "{feat}");
    let chain_id_str = feat["chains"][0]["id"].as_str().expect("chain id").to_owned();

    let (err, r) = mcp_call(&mut s, 5, "add_operation", json!({
        "kind": "2D Contour",
        "tool": "EM-0400-2FL",
        "source": "line",
        "name": "Cut",
        "params": &project::params_to_json(&s9_contour(-2.0))["contour"]
    }));
    assert!(!err, "{r}");
    let (err, geom) = mcp_call(&mut s, 6, "set_operation_geometry", json!({
        "operation": "Cut",
        "picks": [{ "model": "line", "id": chain_id_str, "open_side": "right" }]
    }));
    assert!(!err, "{geom}");
    assert_eq!(geom["chain_overrides"][0]["open_side"], json!("right"));
    let (err, view) = mcp_call(&mut s, 7, "get_operation_geometry", json!({ "operation": "Cut" }));
    assert!(!err, "{view}");
    assert_eq!(view["chain_overrides"][0]["open_side"], json!("right"));
    let (err, gen_res) = mcp_call(&mut s, 8, "generate", json!({ "name": "Cut" }));
    assert!(!err, "{gen_res}");

    let stock = StockShape::Box { dx: 100.0, dy: 80.0, dz: 10.0 };
    for side in [OpenChainSide::Left, OpenChainSide::Right] {
        let (w, mut h, cid) = world_with_dxf(&line_dxf, &stock, Location::default());
        let mut p = s9_contour(-2.0);
        if let OpParams::Contour { chain_overrides, .. } = &mut p {
            *chain_overrides = vec![ChainOverride {
                entity: "part".into(),
                chain: cid,
                side: None,
                reverse: false,
                open_side: side,
            }];
        }
        run(&w, &mut h, CreateOperation::new(&w, "Op", TOOL10, SrcRef::Entity("part".into()), p), "create");
        run(&w, &mut h, GenerateOperation::new(&w, "Op"), "gen");
        let proj = dir.join(format!("s9_{side:?}.camproj"));
        project::save(&w, &proj).expect("save");
        let text = std::fs::read_to_string(&proj).expect("read");
        let fresh = new_world();
        project::open(&fresh, &text).expect("open");
        let e = model::find_by_name(&fresh, "Op").expect("op");
        assert_eq!(operation::status_of(&fresh, &e), OpStatus::Ok);
        let OpParams::Contour { chain_overrides, .. } = model::operation_of(&fresh, &e).expect("op").params else {
            panic!("Contour")
        };
        assert_eq!(chain_overrides[0].open_side, side);
    }

    // Legacy JSON without `open_side` key loads as `OpenChainSide::None` and stays fresh.
    let (w, mut h, cid) = world_with_dxf(&line_dxf, &stock, Location::default());
    let mut p_legacy = s9_contour(-2.0);
    if let OpParams::Contour { chain_overrides, .. } = &mut p_legacy {
        *chain_overrides = vec![ChainOverride {
            entity: "part".into(),
            chain: cid,
            side: None,
            reverse: true,
            open_side: OpenChainSide::None,
        }];
    }
    run(&w, &mut h, CreateOperation::new(&w, "Legacy", TOOL10, SrcRef::Entity("part".into()), p_legacy), "legacy");
    run(&w, &mut h, GenerateOperation::new(&w, "Legacy"), "gen legacy");
    let proj = dir.join("s9_legacy.camproj");
    project::save(&w, &proj).expect("save legacy");
    let text = std::fs::read_to_string(&proj).expect("read legacy");
    assert!(!text.contains("\"open_side\""), "default open_side must be omitted from JSON: {text}");
    let fresh = new_world();
    project::open(&fresh, &text).expect("open legacy");
    let e = model::find_by_name(&fresh, "Legacy").expect("legacy op");
    assert_eq!(operation::status_of(&fresh, &e), OpStatus::Ok);
    let OpParams::Contour { chain_overrides, .. } = model::operation_of(&fresh, &e).expect("op").params else {
        panic!("Contour")
    };
    assert_eq!(chain_overrides[0].open_side, OpenChainSide::None);
}
