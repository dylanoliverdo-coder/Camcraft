//! Journeyman spec S3 integration checks (Checks 5 & 6, refusals, MCP reset):
//! posted NC verification, project JSON round-trip & legacy load, command-level
//! by-name refusals, and MCP `edit_operation` reset-to-default on Links + Face.
//!
//! Provenance: Journeyman spec S3 (docs/journeyman/specs/S3-fed-retract.md);
//! no vendor parameters, text or code.

use serde_json::{Value, json};

use camcraft_app::commands::{
    AddTool, CreateOperation, EditOperationParams, GenerateOperation, History, ImportModel,
    SetStock,
};
use camcraft_app::geometry::{Point3, cube};
use camcraft_app::links::{Approach, Link, Links};
use camcraft_app::machining2d::{FaceHoles, PocketStrategy};
use camcraft_app::machining3d::Entry3D;
use camcraft_app::mcp::Server;
use camcraft_app::model;
use camcraft_app::operation::{self, Height, HeightReference, OpParams, OpStatus, OpType, SrcRef};
use camcraft_app::posting;
use camcraft_app::project;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::catalog;
use camcraft_app::world::new_world;

const RF: f64 = 2540.0;

fn tool(name: &str) -> camcraft_app::tools::Tool {
    catalog()
        .into_iter()
        .find(|(n, _)| *n == name)
        .map(|(_, t)| t)
        .expect("catalog tool")
}

fn pocket_40x30(retract_feed: f64) -> OpParams {
    OpParams::Pocket {
        floor_stock: 0.0,
        stepover: 3.0,
        stock_to_leave: 0.0,
        clearance_z: Height::fixed(10.0),
        retract_z: Some(Height::fixed(5.0)),
        top_z: Height::fixed(0.0),
        bottom_z: Height::fixed(-5.0),
        stepdown: 2.5,
        feed: 800.0,
        plunge_feed: 200.0,
        climb: true,
        strategy: PocketStrategy::Zigzag,
        optimal_load: 0.2,
        entry: Entry3D::Plunge,
        helix_radius: 1.5,
        blend_turns: false,
        blend_radius: None,
        links: Links {
            level: Link::Retract,
            pass: Link::Retract,
            approach: Approach { plunge_height: 1.0, retract_feed, ..Approach::DEFAULT },
            ..Links::PLANAR
        },
        tilt: None,
        rest: Default::default(),
        corner_relief: Default::default(),
        pass_start: Default::default(),
        finish: Default::default(),
    }
}

fn face_params(retract_feed: f64) -> OpParams {
    OpParams::Face {
        floor_stock: 0.0,
        stepover: 7.0,
        clearance_z: Height::of(HeightReference::TopOfStock, 10.0),
        cut_z: Height::of(HeightReference::TopOfStock, -2.0),
        top_z: None,
        stepdown: None,
        feed: 800.0,
        plunge_feed: 200.0,
        retract_feed, finish_levels: 0, finish_stepdown: 0.0,
        climb: true,
        blend_turns: false,
        blend_radius: None,
        holes: FaceHoles::Lift,
        tilt: None,
    }
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
fn s3_check_5_posted_nc_emits_g1_fed_lift_followed_by_g0_rapid() {
    let world = new_world();
    let mut history = History::default();
    // 40 x 30 x 10 mm stock (z in [-10, 0]) around a 10 x 10 mm central boss.
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
    history.execute(
        &world,
        CreateOperation::new(&world, "Pocket1", "EM-0600-4FL", SrcRef::Entity("boss".into()), pocket_40x30(RF))
            .expect("create pocket"),
    );
    history.execute(&world, GenerateOperation::new(&world, "Pocket1").expect("gen pocket"));

    let mut path = std::env::temp_dir();
    path.push(format!("camcraft-s3-check5-{}.nc", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let posted = posting::post_setup(&world, &path).expect("post setup");
    assert_eq!(posted.ops, 1);
    let nc = std::fs::read_to_string(&path).expect("read nc");
    let _ = std::fs::remove_file(&path);

    let lines: Vec<&str> = nc.lines().map(str::trim).filter(|l| !l.is_empty()).collect();
    let mut found_pair = false;
    for w in lines.windows(2) {
        let a = w[0];
        let b = w[1];
        let a_is_fed_z = (a.contains("G01") || a.contains("G1 "))
            && a.contains('Z')
            && (a.contains("F2540") || a.contains("F100."));
        let b_is_rapid_z = (b.contains("G00") || b.contains("G0 ")) && b.contains('Z');
        if a_is_fed_z && b_is_rapid_z {
            found_pair = true;
            break;
        }
    }
    assert!(found_pair, "Check 5: expected G1 Z... F2540 immediately followed by G0 Z... in NC:\n{nc}");
}

#[test]
fn s3_check_6_project_roundtrip_legacy_load_and_stamp() {
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        SetStock::new(&world, StockShape::Box { dx: 40.0, dy: 30.0, dz: 10.0 }, Default::default())
            .expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, "EM-0600-4FL", tool("EM-0600-4FL")).expect("tool"));
    history.execute(
        &world,
        ImportModel::from_mesh("box", cube(Point3::ZERO, Point3::new(40.0, 30.0, 10.0))),
    );

    let p0 = pocket_40x30(0.0);
    let pf = pocket_40x30(RF);
    let f0 = face_params(0.0);
    let ff = face_params(RF);
    let p0_json = serde_json::to_string(&project::params_to_json(&p0)).expect("p0 json");
    let f0_json = serde_json::to_string(&project::params_to_json(&f0)).expect("f0 json");
    assert!(!p0_json.contains("retract_feed"), "zero pocket retract_feed omitted: {p0_json}");
    assert!(!f0_json.contains("retract_feed"), "zero face retract_feed omitted: {f0_json}");

    // Legacy JSON without `retract_feed` loads as 0.0 and matches stamp identically.
    assert_eq!(project::params_from_json(&project::params_to_json(&p0)).expect("p0 back"), p0);
    assert_eq!(project::params_from_json(&project::params_to_json(&f0)).expect("f0 back"), f0);

    // Positive retract_feed round-trips identically on both Pocket and Face and changes stamps.
    let pf_val = project::params_to_json(&pf);
    let ff_val = project::params_to_json(&ff);
    assert!(serde_json::to_string(&pf_val).expect("s").contains("2540"));
    assert!(serde_json::to_string(&ff_val).expect("s").contains("2540"));
    assert_eq!(project::params_from_json(&pf_val).expect("pf back"), pf);
    assert_eq!(project::params_from_json(&ff_val).expect("ff back"), ff);

    let src = SrcRef::Entity("box".into());
    // Changing retract_feed invalidates the generated stamp on both Pocket and Face.
    history.execute(
        &world,
        CreateOperation::new(&world, "P1", "EM-0600-4FL", src.clone(), p0.clone()).expect("p1"),
    );
    history.execute(
        &world,
        CreateOperation::new(&world, "F1", "EM-0600-4FL", src, f0.clone()).expect("f1"),
    );
    history.execute(&world, GenerateOperation::new(&world, "P1").expect("gen p1"));
    history.execute(&world, GenerateOperation::new(&world, "F1").expect("gen f1"));
    let ep1 = model::find_by_name(&world, "P1").expect("P1");
    let ef1 = model::find_by_name(&world, "F1").expect("F1");
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Ok);
    assert_eq!(operation::status_of(&world, &ef1), OpStatus::Ok);
    history.execute(&world, EditOperationParams::new(&world, "P1", pf.clone()).expect("edit p1"));
    history.execute(&world, EditOperationParams::new(&world, "F1", ff.clone()).expect("edit f1"));
    assert_eq!(operation::status_of(&world, &ep1), OpStatus::Stale, "Pocket stamp must change with retract_feed");
    assert_eq!(operation::status_of(&world, &ef1), OpStatus::Stale, "Face stamp must change with retract_feed");

    // Full project file save/load round-trip preserves fresh status & retract_feed.
    history.execute(&world, GenerateOperation::new(&world, "F1").expect("regen f1"));
    let mut file = std::env::temp_dir();
    file.push(format!("camcraft-s3-check6-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&file);
    project::save(&world, &file).expect("save project");
    let text = std::fs::read_to_string(&file).expect("read project");
    let _ = std::fs::remove_file(&file);
    let loaded = new_world();
    project::open(&loaded, &text).expect("open project");
    let e_p1 = model::find_by_name(&loaded, "P1").expect("P1");
    let e_f1 = model::find_by_name(&loaded, "F1").expect("F1");
    assert_eq!(model::operation_of(&loaded, &e_p1).expect("P1 op").params, pf);
    assert_eq!(model::operation_of(&loaded, &e_f1).expect("F1 op").params, ff);
    assert_eq!(operation::status_of(&loaded, &e_f1), OpStatus::Ok);
}

#[test]
fn s3_refusals_by_name_and_mcp_reset_to_default() {
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        SetStock::new(&world, StockShape::Box { dx: 40.0, dy: 30.0, dz: 10.0 }, Default::default())
            .expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, "EM-0600-4FL", tool("EM-0600-4FL")).expect("tool"));
    history.execute(
        &world,
        ImportModel::from_mesh("box", cube(Point3::ZERO, Point3::new(40.0, 30.0, 10.0))),
    );
    let src = SrcRef::Entity("box".into());

    // CreateOperation refuses negative / NaN / inf retract_feed by name on both Pocket and Face.
    for bad in [-10.0, f64::NAN, f64::INFINITY] {
        let ep = CreateOperation::new(&world, "BadP", "EM-0600-4FL", src.clone(), pocket_40x30(bad))
            .err()
            .expect("bad pocket must refuse");
        assert!(ep.to_string().contains("retract_feed"), "Pocket create refusal: {ep}");
        let ef = CreateOperation::new(&world, "BadF", "EM-0600-4FL", src.clone(), face_params(bad))
            .err()
            .expect("bad face must refuse");
        assert!(ef.to_string().contains("retract_feed"), "Face create refusal: {ef}");
    }

    // Create valid ops and verify EditOperationParams refuses bad values by name.
    history.execute(
        &world,
        CreateOperation::new(&world, "P1", "EM-0600-4FL", src.clone(), pocket_40x30(0.0)).expect("p1"),
    );
    history.execute(
        &world,
        CreateOperation::new(&world, "F1", "EM-0600-4FL", src, face_params(0.0)).expect("f1"),
    );
    for bad in [-1.0, f64::NAN, f64::INFINITY] {
        let ep = EditOperationParams::new(&world, "P1", pocket_40x30(bad)).err().expect("bad edit P1");
        assert!(ep.to_string().contains("retract_feed"), "Pocket edit refusal: {ep}");
        let ef = EditOperationParams::new(&world, "F1", face_params(bad)).err().expect("bad edit F1");
        assert!(ef.to_string().contains("retract_feed"), "Face edit refusal: {ef}");
    }

    // Save project and load into MCP Server to exercise MCP edit_operation + reset-to-default.
    let mut file = std::env::temp_dir();
    file.push(format!("camcraft-s3-mcp-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&file);
    project::save(&world, &file).expect("save");
    let mut srv = Server::new();
    let (err, r) = mcp_call(&mut srv, 1, "open_project", json!({ "path": file.to_string_lossy() }));
    let _ = std::fs::remove_file(&file);
    assert!(!err, "open_project failed: {r}");

    let (err, r) = mcp_call(
        &mut srv,
        2,
        "edit_operation",
        json!({ "name": "P1", "params": { "links": { "approach": { "retract_feed": 2540.0 } } } }),
    );
    assert!(!err, "MCP set Pocket retract_feed=2540 must succeed: {r}");
    let (err, body) = mcp_call(
        &mut srv,
        3,
        "edit_operation",
        json!({ "name": "P1", "params": { "links": { "approach": { "retract_feed": 0.0 } } } }),
    );
    assert!(!err, "MCP reset Pocket retract_feed=0 must succeed: {body}");
    let (err, _) = mcp_call(
        &mut srv,
        4,
        "edit_operation",
        json!({ "name": "P1", "params": { "links": { "approach": { "retract_feed": 0.0, "bogus_key": 1.0 } } } }),
    );
    assert!(err, "MCP unknown sub-key inside links.approach must still fail");

    let (err, r) = mcp_call(
        &mut srv,
        5,
        "edit_operation",
        json!({ "name": "F1", "params": { "retract_feed": 2540.0 } }),
    );
    assert!(!err, "MCP set Face retract_feed=2540 must succeed: {r}");
    let (err, body) = mcp_call(
        &mut srv,
        6,
        "edit_operation",
        json!({ "name": "F1", "params": { "retract_feed": 0.0 } }),
    );
    assert!(!err, "MCP reset Face retract_feed=0 must succeed: {body}");
    let (err, _) = mcp_call(
        &mut srv,
        7,
        "edit_operation",
        json!({ "name": "F1", "params": { "retract_feed": -5.0 } }),
    );
    assert!(err, "MCP negative Face retract_feed must fail");
}

#[test]
fn s3_non_2d_ops_refuse_retract_feed_in_commands_and_mcp() {
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        SetStock::new(&world, StockShape::Box { dx: 40.0, dy: 30.0, dz: 10.0 }, Default::default()).expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, "EM-0600-4FL", tool("EM-0600-4FL")).expect("tool"));
    history.execute(&world, AddTool::new(&world, "DR-0400", tool("DR-0400")).expect("drill tool"));
    history.execute(&world, ImportModel::from_mesh("box", cube(Point3::ZERO, Point3::new(40.0, 30.0, 10.0))));
    let src = SrcRef::Entity("box".into());

    // All three non-2D Links ops refuse any non-zero / non-finite retract_feed by name.
    for kind in [OpType::Pocket3D, OpType::RotaryOffset, OpType::MultiAxisRough] {
        let mut p = operation::seed_params(kind.ordinal());
        for bad in [2540.0, -10.0, f64::NAN] {
            if let Some(l) = p.links_mut() {
                l.approach.retract_feed = bad;
            }
            let e = CreateOperation::new(&world, "BadNon2D", "EM-0600-4FL", src.clone(), p.clone())
                .err()
                .expect("non-2D op with retract_feed must refuse");
            assert!(e.to_string().contains("retract_feed"), "{kind:?} create refusal: {e}");
        }
    }

    // Valid Pocket3D and Drill: EditOperationParams and MCP edit_operation both reject retract_feed.
    let p3_ok = operation::seed_params(OpType::Pocket3D.ordinal());
    let mut p3_bad = p3_ok.clone();
    if let Some(l) = p3_bad.links_mut() {
        l.approach.retract_feed = RF;
    }
    history.execute(&world, CreateOperation::new(&world, "P3", "EM-0600-4FL", src.clone(), p3_ok).expect("p3"));
    history.execute(
        &world,
        CreateOperation::new(&world, "D1", "DR-0400", src, operation::seed_params(OpType::Drill.ordinal())).expect("d1"),
    );
    let e = EditOperationParams::new(&world, "P3", p3_bad).err().expect("Pocket3D edit must refuse");
    assert!(e.to_string().contains("retract_feed"), "Pocket3D edit refusal: {e}");

    let mut file = std::env::temp_dir();
    file.push(format!("camcraft-s3-non2d-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&file);
    project::save(&world, &file).expect("save");
    let mut srv = Server::new();
    let (err, r) = mcp_call(&mut srv, 1, "open_project", json!({ "path": file.to_string_lossy() }));
    let _ = std::fs::remove_file(&file);
    assert!(!err, "open_project: {r}");

    for (id, name, patch) in [
        (2, "P3", json!({ "links": { "approach": { "retract_feed": 2540.0 } } })),
        (3, "P3", json!({ "links": { "approach": { "retract_feed": 0.0 } } })),
        (4, "D1", json!({ "retract_feed": 2540.0 })),
        (5, "D1", json!({ "retract_feed": 0.0 })),
    ] {
        let (err, body) = mcp_call(&mut srv, id, "edit_operation", json!({ "name": name, "params": patch }));
        assert!(err, "MCP edit_operation on {name} with {patch} must fail, got {body}");
    }
}
