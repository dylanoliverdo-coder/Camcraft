//! Journeyman spec S4: 2D Pocket wall finishing passes (app layer — commands,
//! MCP, project serialization, digest invariance, flute-length refusal, and
//! end-to-end generation + verification).
//!
//! Gate map: covers camcraft_app::operation (types, sane, params, generate, rest_op),
//! camcraft_app::project (params, params_impl), camcraft_app::mcp::edit.
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held 1991; P. Smid 2007; no vendor parameters, text or code.

use camcraft_app::commands::{
    AddTool, CreateOperation, EditOperationParams, GenerateOperation, History, SetStock,
};
use camcraft_app::machining2d::{
    FinishDirection, FinishLevel, PocketFinish, PocketStrategy, RestParams, RestSource,
};
use camcraft_app::machining3d::Entry3D;
use camcraft_app::model;
use camcraft_app::operation::{Height, OpParams, SrcRef};
use camcraft_app::project;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::catalog;
use camcraft_app::verify::Warning;
use camcraft_app::world::new_world;
use serde_json::json;

fn catalog_tool(name: &str) -> camcraft_app::tools::Tool {
    catalog()
        .into_iter()
        .find(|(n, _)| *n == name)
        .map(|(_, t)| t)
        .expect("catalog tool")
}

fn make_pocket(finish: PocketFinish) -> OpParams {
    OpParams::Pocket {
        floor_stock: 0.0,
        stepover: 2.0,
        stock_to_leave: 0.0,
        clearance_z: Height::fixed(15.0),
        retract_z: Some(Height::fixed(12.0)),
        top_z: Height::fixed(10.0),
        bottom_z: Height::fixed(0.0),
        stepdown: 2.5,
        feed: 900.0,
        plunge_feed: 250.0,
        climb: true,
        strategy: PocketStrategy::Waterline,
        entry: Entry3D::Helix,
        helix_radius: 1.5,
        links: camcraft_app::links::Links::PLANAR,
        optimal_load: camcraft_app::load::DEFAULT_OPTIMAL_LOAD,
        blend_turns: false,
        blend_radius: None,
        rest: None,
        corner_relief: Default::default(),
        tilt: None,
        pass_start: Default::default(),
        finish,
    }
}

/// S4 Check 6 + refusals + undo/redo + flute-length law + end-to-end verification.
#[test]
fn s4_pocket_finish_round_trip_refusals_undo_and_generate() {
    let set = make_pocket(PocketFinish {
        finish_passes: 2,
        finish_spacing: 0.1,
        spring_passes: 1,
        finish_direction: FinishDirection::Conventional,
        finish_level: FinishLevel::Last,
    });
    let wire = project::params_to_json(&set);
    assert_eq!(project::params_from_json(&wire).expect("reads back"), set, "S4 fields round-trip");

    // Defaults stay off the wire so legacy projects load with defaults and save unchanged.
    let plain = make_pocket(PocketFinish::default());
    let plain_wire = project::params_to_json(&plain);
    assert!(!plain_wire.to_string().contains("\"finish\""), "default stays off wire: {plain_wire}");
    assert_eq!(project::params_from_json(&plain_wire).expect("reads back"), plain, "absent key loads defaults");

    // Command creation & edit refusals.
    let world = new_world();
    let mut history = History::default();
    let dir = std::env::temp_dir().join(format!("cc_s4_rt_{}", std::process::id()));
    let _ = std::fs::create_dir_all(&dir);
    history.execute(
        &world,
        SetStock::new(&world, StockShape::Box { dx: 50.0, dy: 40.0, dz: 10.0 }, Default::default()).expect("stock"),
    );
    let mut short_tool = catalog_tool("EM-0600-4FL");
    short_tool.flute_len = 5.0;
    history.execute(&world, AddTool::new(&world, "T_SHORT", short_tool).expect("add tool"));

    // Digest changes when PocketFinish moves off default.
    assert_ne!(
        camcraft_app::operation::current_stamp(&world, "T_SHORT", &SrcRef::Stock, &plain, "P_D", None),
        camcraft_app::operation::current_stamp(&world, "T_SHORT", &SrcRef::Stock, &set, "P_D", None),
        "non-default PocketFinish must change operation stamp digest"
    );

    for bad_fin in [
        PocketFinish { finish_passes: 2, finish_spacing: 0.0, ..Default::default() },
        PocketFinish { finish_passes: 0, finish_spacing: -0.2, ..Default::default() },
        PocketFinish { finish_passes: 1, finish_spacing: f64::NAN, ..Default::default() },
        PocketFinish { finish_passes: 1, finish_spacing: 20_000.0, ..Default::default() },
    ] {
        let err = CreateOperation::new(&world, "P_BAD", "T_SHORT", SrcRef::Stock, make_pocket(bad_fin))
            .err()
            .expect("invalid finish params must refuse at CreateOperation");
        assert!(err.to_string().contains("finishing passes"), "refusal by name: {err}");
    }

    history.execute(&world, CreateOperation::new(&world, "P1", "T_SHORT", SrcRef::Stock, plain.clone()).expect("create"));
    let bad_edit = make_pocket(PocketFinish { finish_passes: 1, finish_spacing: f64::INFINITY, ..Default::default() });
    assert!(EditOperationParams::new(&world, "P1", bad_edit).is_err(), "non-finite refuses at edit");

    // Edit to S4 settings, verify undo/redo.
    history.execute(&world, EditOperationParams::new(&world, "P1", set.clone()).expect("edit"));
    assert_eq!(model::operation_of(&world, &model::find_by_name(&world, "P1").expect("P1")).expect("op").params, set);
    assert!(history.undo(&world));
    assert_eq!(model::operation_of(&world, &model::find_by_name(&world, "P1").expect("P1")).expect("op").params, plain);
    assert!(history.redo(&world));
    assert_eq!(model::operation_of(&world, &model::find_by_name(&world, "P1").expect("P1")).expect("op").params, set);

    // Full project file save/load round-trip preserves all 5 S4 fields.
    let proj = dir.join("s4.camcraft.json");
    project::save(&world, &proj).expect("save project");
    let loaded = new_world();
    project::load(&loaded, &proj).expect("load project");
    assert_eq!(
        model::operation_of(&loaded, &model::find_by_name(&loaded, "P1").expect("P1")).expect("op").params,
        set,
        "full project save/load preserves S4 fields"
    );

    // `finish_level = Last` with 5 mm flute on 10 mm wall must refuse by name at GenerateOperation.
    let err = GenerateOperation::new(&world, "P1").err().expect("short flute must refuse");
    assert!(err.to_string().contains("flute"), "refusal must name flute: {err}");

    // Rest machining + active finishing passes refuses by name at command creation.
    let mut with_rest = set.clone();
    if let OpParams::Pocket { rest, .. } = &mut with_rest {
        *rest = Some(RestParams {
            source: RestSource::EnteredTool,
            previous_tool_diameter: 12.0,
            previous_corner_radius: 0.0,
            previous_stock_to_leave: 0.0,
            allowance: 0.0,
        });
    }
    let rest_err = CreateOperation::new(&world, "P_REST", "T_SHORT", SrcRef::Stock, with_rest)
        .err()
        .expect("rest + active finish must refuse");
    assert!(rest_err.to_string().contains("wall finishing passes"), "rest refusal: {rest_err}");

    // End-to-end multi-level `finish_level = Last` with sufficient flute length:
    // zero RapidInStock, Gouge, Crossing, or post-blocking warnings.
    let mut tall_tool = catalog_tool("EM-0600-4FL");
    tall_tool.flute_len = 15.0;
    history.execute(&world, AddTool::new(&world, "T_TALL", tall_tool).expect("add tall tool"));
    history.execute(&world, CreateOperation::new(&world, "P_LAST", "T_TALL", SrcRef::Stock, set).expect("create P_LAST"));
    history.execute(&world, GenerateOperation::new(&world, "P_LAST").expect("generate P_LAST"));
    let op_e = model::find_by_name(&world, "P_LAST").expect("P_LAST entity");
    let cache = model::toolpath_of(&world, &op_e).expect("P_LAST cache");
    assert!(!cache.path.moves.is_empty());
    for w in &cache.warnings {
        assert!(
            !matches!(w, Warning::RapidInStock { .. } | Warning::Gouge { .. } | Warning::Crossing { .. })
                && !w.blocks_post(),
            "unexpected warning on P_LAST: {w:?}"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// MCP `add_operation` / `edit_operation` edits S4 fields, resets them back to
/// defaults, and refuses invalid values and unknown sub-keys by name.
#[test]
fn s4_mcp_edit_operation_sets_resets_and_refuses_invalid() {
    let mcp = |server: &mut camcraft_app::mcp::Server, name: &str, args: serde_json::Value| -> (bool, serde_json::Value) {
        let msg = json!({
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": { "name": name, "arguments": args }
        });
        let reply = server.handle(&msg).expect("MCP response");
        let result = &reply["result"];
        let refused = result["isError"] == json!(true);
        (refused, if refused { result["content"][0]["text"].clone() } else { result["structuredContent"].clone() })
    };

    let mut server = camcraft_app::mcp::Server::new();
    let _ = server.handle(&json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "initialize",
        "params": { "protocolVersion": "2025-06-18" }
    }));
    let (err, body) = mcp(&mut server, "set_stock", json!({ "box": { "dx": 50.0, "dy": 40.0, "dz": 10.0 } }));
    assert!(!err, "set_stock failed: {body}");
    let (err, body) = mcp(&mut server, "add_tool", json!({ "catalog": "EM-0600-4FL", "name": "T1" }));
    assert!(!err, "add_tool failed: {body}");
    let (err, body) = mcp(
        &mut server,
        "add_operation",
        json!({
            "kind": "pocket",
            "name": "P_MCP",
            "tool": "T1",
            "params": {
                "finish": {
                    "finish_passes": 2,
                    "finish_spacing": 0.1,
                    "spring_passes": 1,
                    "finish_direction": "conventional",
                    "finish_level": "last"
                }
            }
        }),
    );
    assert!(!err, "add_operation with S4 finish failed: {body}");

    // Resetting every field back to default via MCP edit_operation succeeds and omits `finish` from wire.
    let (err, body) = mcp(
        &mut server,
        "edit_operation",
        json!({
            "name": "P_MCP",
            "params": {
                "finish": {
                    "finish_passes": 0,
                    "finish_spacing": 0.0,
                    "spring_passes": 0,
                    "finish_direction": "same",
                    "finish_level": "every"
                }
            }
        }),
    );
    assert!(!err, "reset S4 fields back to default failed: {body}");
    assert!(body["params"]["pocket"].get("finish").is_none(), "reset defaults stay off wire: {body}");

    // Invalid spacing via MCP refuses by name.
    let (err, body) = mcp(
        &mut server,
        "edit_operation",
        json!({
            "name": "P_MCP",
            "params": { "finish": { "finish_passes": 1, "finish_spacing": 0.0 } }
        }),
    );
    assert!(err, "MCP must refuse finish_passes > 0 with spacing == 0: {body}");
    assert!(body.as_str().unwrap_or("").contains("spacing"), "refusal names spacing: {body}");

    // Unknown sub-key refuses by name.
    let (err, body) = mcp(
        &mut server,
        "edit_operation",
        json!({
            "name": "P_MCP",
            "params": { "finish": { "bogus_key": 1 } }
        }),
    );
    assert!(err, "MCP must refuse unknown finish sub-key: {body}");
    assert!(body.as_str().unwrap_or("").contains("bogus_key"), "refusal names unknown key: {body}");
}
