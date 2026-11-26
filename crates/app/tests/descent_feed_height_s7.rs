//! App, project roundtrip, digest, command validation, and MCP tests for
//! Journeyman spec S7 (`Approach::feed_from`).
//!
//! Provenance: Journeyman spec S7 (docs/journeyman/specs/S7-descent-feed-height.md);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007; RS-274 /
//! ISO 6983; no vendor parameters, text or code.

use serde_json::{Value, json};

use camcraft_app::commands::{
    AddTool, CreateOperation, EditOperationParams, GenerateOperation, History, ImportModel, SetStock,
};
use camcraft_app::geometry::{Point3, cube};
use camcraft_app::links::{Approach, FeedFrom, Link, Links};
use camcraft_app::machining2d::{Lead, PocketStrategy, Side, WallPasses};
use camcraft_app::machining3d::Entry3D;
use camcraft_app::mcp::Server;
use camcraft_app::model;
use camcraft_app::operation::{self, Height, OpParams, OpStatus, OpType, SrcRef, seed_params};
use camcraft_app::project;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::catalog;
use camcraft_app::world::new_world;

fn tool(name: &str) -> camcraft_app::tools::Tool {
    catalog()
        .into_iter()
        .find(|(n, _)| *n == name)
        .map(|(_, t)| t)
        .expect("catalog tool")
}

fn pocket_op_params(feed_from: FeedFrom) -> OpParams {
    OpParams::Pocket {
        stepover: 3.0,
        stock_to_leave: 0.0,
        floor_stock: 0.0,
        clearance_z: Height::fixed(5.0),
        retract_z: Some(Height::fixed(5.0)),
        top_z: Height::fixed(0.0),
        bottom_z: Height::fixed(-5.0),
        stepdown: 2.5,
        feed: 600.0,
        plunge_feed: 150.0,
        climb: true,
        entry: Entry3D::Plunge,
        helix_radius: 1.5,
        strategy: PocketStrategy::OffsetRings,
        optimal_load: 0.25,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            level: Link::Retract,
            pass: Link::Retract,
            approach: Approach { feed_from, ..Approach::DEFAULT },
            ..Links::PLANAR
        },
        tilt: None,
        rest: None,
        corner_relief: Default::default(),
        pass_start: Default::default(),
        finish: Default::default(),
    }
}

fn setup_world() -> (flecs_ecs::prelude::World, History) {
    let world = new_world();
    let mut history = History::default();
    let stock_cmd = SetStock::new(
        &world,
        StockShape::Box { dx: 40.0, dy: 30.0, dz: 10.0 },
        camcraft_app::schema::Location {
            pos: Point3::new(20.0, 15.0, -10.0),
            ..Default::default()
        },
    )
    .expect("stock");
    history.execute(&world, stock_cmd);
    history.execute(&world, AddTool::new(&world, "EM-0600-4FL", tool("EM-0600-4FL")).expect("tool"));
    history.execute(
        &world,
        ImportModel::from_mesh("boss", cube(Point3::new(15.0, 10.0, -10.0), Point3::new(25.0, 20.0, 0.0))),
    );
    (world, history)
}

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
fn check7_project_roundtrip_and_pre_s7_compatibility() {
    let (world, mut history) = setup_world();
    let src = SrcRef::Entity("boss".into());

    // Default (Auto) omits `feed_from` in JSON (pre-S7 byte compatibility).
    let p_auto = pocket_op_params(FeedFrom::Auto);
    let auto_json = serde_json::to_string(&project::params_to_json(&p_auto)).expect("auto json");
    assert!(!auto_json.contains("feed_from"), "Auto omits feed_from: {auto_json}");

    for (name, mode) in [
        ("op_auto", FeedFrom::Auto),
        ("op_mat1", FeedFrom::Material { gap: 1.0 }),
        ("op_mat0", FeedFrom::Material { gap: 0.0 }),
        ("op_abs", FeedFrom::Absolute { z: 0.5 }),
    ] {
        let cmd = CreateOperation::new(&world, name, "EM-0600-4FL", src.clone(), pocket_op_params(mode))
            .expect("create op");
        history.execute(&world, cmd);
    }

    let mut file = std::env::temp_dir();
    file.push(format!("camcraft-s7-check7-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&file);
    project::save(&world, &file).expect("save project");
    let raw = std::fs::read_to_string(&file).expect("read saved json");
    let _ = std::fs::remove_file(&file);

    // Reload and verify all 4 modes survived verbatim.
    let loaded = new_world();
    project::open(&loaded, &raw).expect("open project");
    for (name, expected) in [
        ("op_auto", FeedFrom::Auto),
        ("op_mat1", FeedFrom::Material { gap: 1.0 }),
        ("op_mat0", FeedFrom::Material { gap: 0.0 }),
        ("op_abs", FeedFrom::Absolute { z: 0.5 }),
    ] {
        let e = model::find_by_name(&loaded, name).expect("op present");
        let op = model::operation_of(&loaded, &e).expect("op record");
        assert_eq!(op.params.links().expect("links").approach.feed_from, expected, "{name}");
    }

    // Legacy pre-S7 file (strip every `feed_from` key recursively) loads with FeedFrom::Auto.
    fn strip_key(v: &mut Value, key: &str) -> usize {
        match v {
            Value::Object(m) => {
                let mut n = usize::from(m.remove(key).is_some());
                for child in m.values_mut() {
                    n += strip_key(child, key);
                }
                n
            }
            Value::Array(a) => a.iter_mut().map(|c| strip_key(c, key)).sum(),
            _ => 0,
        }
    }
    let mut legacy: Value = serde_json::from_str(&raw).expect("parse json");
    let stripped = strip_key(&mut legacy, "feed_from");
    assert_eq!(stripped, 3, "expected 3 non-auto feed_from keys stripped from saved project");
    let legacy_world = new_world();
    project::open(&legacy_world, &serde_json::to_string_pretty(&legacy).expect("json")).expect("open legacy");
    for name in ["op_auto", "op_mat1", "op_mat0", "op_abs"] {
        let e = model::find_by_name(&legacy_world, name).expect("op");
        let op = model::operation_of(&legacy_world, &e).expect("record");
        assert_eq!(op.params.links().expect("links").approach.feed_from, FeedFrom::Auto);
    }
}

#[test]
fn digest_invariance_command_refusals_and_undo_redo() {
    let (world, mut history) = setup_world();
    let src = SrcRef::Entity("boss".into());

    // Changing feed_from invalidates the generated stamp (Auto -> Material -> Absolute).
    let p_auto = pocket_op_params(FeedFrom::Auto);
    let p_mat = pocket_op_params(FeedFrom::Material { gap: 1.0 });
    let p_abs = pocket_op_params(FeedFrom::Absolute { z: 0.5 });

    history.execute(
        &world,
        CreateOperation::new(&world, "P1", "EM-0600-4FL", src.clone(), p_auto.clone()).expect("p1"),
    );
    history.execute(&world, GenerateOperation::new(&world, "P1").expect("gen p1"));
    let ep1 = model::find_by_name(&world, "P1").expect("P1");
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Ok);

    history.execute(&world, EditOperationParams::new(&world, "P1", p_mat).expect("edit mat"));
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Stale, "Material must invalidate stamp");

    history.execute(&world, GenerateOperation::new(&world, "P1").expect("regen mat"));
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Ok);

    history.execute(&world, EditOperationParams::new(&world, "P1", p_abs).expect("edit abs"));
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Stale, "Absolute must invalidate stamp");

    // Undo / redo restores feed_from cleanly.
    let read_ff = |w: &flecs_ecs::prelude::World| {
        let e = model::find_by_name(w, "P1").expect("P1");
        model::operation_of(w, &e).expect("rec").params.links().expect("links").approach.feed_from
    };
    assert_eq!(read_ff(&world), FeedFrom::Absolute { z: 0.5 });
    history.undo(&world);
    assert_eq!(read_ff(&world), FeedFrom::Material { gap: 1.0 });
    history.redo(&world);
    assert_eq!(read_ff(&world), FeedFrom::Absolute { z: 0.5 });

    // Command creation refusals for negative / NaN / inf gap and non-finite z.
    for bad in [
        FeedFrom::Material { gap: -0.5 },
        FeedFrom::Material { gap: f64::NAN },
        FeedFrom::Absolute { z: f64::INFINITY },
    ] {
        let err = CreateOperation::new(&world, "Bad", "EM-0600-4FL", src.clone(), pocket_op_params(bad))
            .err()
            .expect("bad feed_from must refuse");
        assert!(err.to_string().contains("feed_from"), "{bad:?}: {err}");
    }

    // Non-2D Links op (Pocket3D) refuses non-Auto feed_from by name.
    let mut p3d = seed_params(OpType::Pocket3D.ordinal());
    p3d.links_mut().expect("3d links").approach.feed_from = FeedFrom::Material { gap: 1.0 };
    let err = CreateOperation::new(&world, "Bad3D", "EM-0600-4FL", src, p3d)
        .err()
        .expect("Pocket3D must refuse feed_from");
    assert!(err.to_string().contains("2D milling"), "{err}");
}

#[test]
fn mcp_edit_set_switch_reset_and_refuse_feed_from() {
    let (world, mut history) = setup_world();
    let src = SrcRef::Entity("boss".into());
    history.execute(
        &world,
        CreateOperation::new(&world, "p1", "EM-0600-4FL", src.clone(), pocket_op_params(FeedFrom::Auto)).expect("p1"),
    );
    history.execute(
        &world,
        CreateOperation::new(&world, "p3d", "EM-0600-4FL", src, seed_params(OpType::Pocket3D.ordinal())).expect("p3d"),
    );

    let mut file = std::env::temp_dir();
    file.push(format!("camcraft-s7-mcp-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&file);
    project::save(&world, &file).expect("save");
    let mut server = Server::new();
    let (err, r) = mcp_call(&mut server, 1, "open_project", json!({ "path": file.to_string_lossy() }));
    let _ = std::fs::remove_file(&file);
    assert!(!err, "open_project failed: {r}");

    // Set material { gap: 1.0 } via edit_operation.
    let (err, body) = mcp_call(
        &mut server,
        2,
        "edit_operation",
        json!({
            "name": "p1",
            "params": {
                "links": { "approach": { "feed_from": { "material": { "gap": 1.0 } } } }
            }
        }),
    );
    assert!(!err, "set material gap 1: {body}");

    // Switch directly from material to absolute via edit_operation.
    let (err, body) = mcp_call(
        &mut server,
        3,
        "edit_operation",
        json!({
            "name": "p1",
            "params": {
                "links": { "approach": { "feed_from": { "absolute": { "z": 0.5 } } } }
            }
        }),
    );
    assert!(!err, "switch to absolute: {body}");

    // Reset back to "auto".
    let (err, body) = mcp_call(
        &mut server,
        4,
        "edit_operation",
        json!({
            "name": "p1",
            "params": { "links": { "approach": { "feed_from": "auto" } } }
        }),
    );
    assert!(!err, "reset to auto: {body}");

    // Refuse negative gap.
    let (err, body) = mcp_call(
        &mut server,
        5,
        "edit_operation",
        json!({
            "name": "p1",
            "params": { "links": { "approach": { "feed_from": { "material": { "gap": -2.0 } } } } }
        }),
    );
    assert!(err && body.to_string().contains("feed_from"), "negative gap refused: {body}");

    // Refuse unknown sub-key.
    let (err, body) = mcp_call(
        &mut server,
        6,
        "edit_operation",
        json!({
            "name": "p1",
            "params": { "links": { "approach": { "feed_from": { "material": { "gap": 1.0, "extra": 3.0 } } } } }
        }),
    );
    assert!(err && body.to_string().contains("unknown field"), "unknown sub-key refused: {body}");

    // Refuse feed_from on Pocket3D via MCP edit_operation.
    let (err, body) = mcp_call(
        &mut server,
        7,
        "edit_operation",
        json!({
            "name": "p3d",
            "params": { "links": { "approach": { "feed_from": { "material": { "gap": 1.0 } } } } }
        }),
    );
    assert!(err && body.to_string().contains("2D milling"), "3D refused: {body}");
}

#[test]
fn generate_operation_end_to_end_check4_and_check5_refusal() {
    let (world, mut history) = setup_world();
    let src = SrcRef::Entity("boss".into());

    // Check 4: Pocket with Absolute { z: 0.5 } generates cleanly.
    history.execute(
        &world,
        CreateOperation::new(&world, "OkPocket", "EM-0600-4FL", src.clone(), pocket_op_params(FeedFrom::Absolute { z: 0.5 }))
            .expect("create ok"),
    );
    history.execute(&world, GenerateOperation::new(&world, "OkPocket").expect("generate ok"));
    let ok_e = model::find_by_name(&world, "OkPocket").expect("OkPocket");
    assert_eq!(operation::status_of(&world, &ok_e), OpStatus::Ok);

    // Check 5: Pocket with Absolute { z: -1.0 } (below stock top 0.0) refuses at GenerateOperation by name.
    history.execute(
        &world,
        CreateOperation::new(&world, "BadPocket", "EM-0600-4FL", src.clone(), pocket_op_params(FeedFrom::Absolute { z: -1.0 }))
            .expect("create bad pocket"),
    );
    let err = GenerateOperation::new(&world, "BadPocket")
        .err()
        .expect("generate must refuse rapid into material");
    assert!(err.to_string().contains("descent feed height refused"), "{err}");

    // Contour with Absolute { z: -1.0 } also refuses at GenerateOperation by name.
    let p_bad_contour = OpParams::Contour {
        stock_to_leave: 0.0,
        floor_stock: 0.0,
        clearance_z: Height::fixed(5.0),
        retract_z: Height::fixed(5.0),
        top_z: Height::fixed(0.0),
        bottom_z: Height::fixed(-5.0),
        stepdown: 2.5,
        feed: 500.0,
        plunge_feed: 120.0,
        climb: true,
        side: Side::Outside,
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
            approach: Approach { feed_from: FeedFrom::Absolute { z: -1.0 }, ..Approach::DEFAULT },
            ..Links::PLANAR
        },
        tilt: None,
        chain_overrides: Vec::new(),
        passes: WallPasses::default(),
    };
    history.execute(
        &world,
        CreateOperation::new(&world, "BadContour", "EM-0600-4FL", src, p_bad_contour).expect("create bad contour"),
    );
    let err = GenerateOperation::new(&world, "BadContour")
        .err()
        .expect("contour generate must refuse rapid into material");
    assert!(err.to_string().contains("descent feed height refused"), "{err}");
}
