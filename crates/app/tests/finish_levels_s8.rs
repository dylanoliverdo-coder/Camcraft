//! Journeyman spec S8 application integration checks (Checks 7, 8, 9, 10,
//! command & MCP refusals, gating off non-supporting ops, project roundtrip).
//!
//! Gate map: covers camcraft_app::operation (types, generate, floor_stock,
//! params), camcraft_app::project (approach, params), camcraft_app::mcp::edit.
//!
//! Provenance: Journeyman spec S8 (`docs/journeyman/specs/S8-finish-levels.md`);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007;
//! M. P. Groover, *Fundamentals of Modern Manufacturing*, Wiley.
//! No vendor parameters, text or code.

use serde_json::{Value, json};

use camcraft_app::cl::{MoveKind, Toolpath};
use camcraft_app::commands::{
    AddTool, CreateOperation, EditOperationParams, GenerateOperation, History, ImportModel,
    SetStock,
};
use camcraft_app::geometry::{Mesh, Point3, Triangle};
use camcraft_app::links::{Approach, Links};
use camcraft_app::machining2d::{FaceHoles, Lead, PocketStrategy, Side, WallPasses};
use camcraft_app::machining3d::Entry3D;
use camcraft_app::mcp::Server;
use camcraft_app::model;
use camcraft_app::operation::{self, Height, OpParams, OpStatus, OpType, SrcRef};
use camcraft_app::project;
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::catalog;
use camcraft_app::verify::Warning;
use camcraft_app::world::new_world;

const TOOL: &str = "EM-0600-4FL";

fn tool(name: &str) -> camcraft_app::tools::Tool {
    catalog()
        .into_iter()
        .find(|(n, _)| *n == name)
        .map(|(_, t)| t)
        .expect("catalog tool")
}

fn box_facets(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Vec<Triangle> {
    let p = |x, y, z| Point3::new(x, y, z);
    let v = [
        p(x0, y0, z0),
        p(x1, y0, z0),
        p(x1, y1, z0),
        p(x0, y1, z0),
        p(x0, y0, z1),
        p(x1, y0, z1),
        p(x1, y1, z1),
        p(x0, y1, z1),
    ];
    let q = |a: usize, b: usize, c: usize, d: usize| [Triangle::new(v[a], v[b], v[c]), Triangle::new(v[a], v[c], v[d])];
    [
        q(0, 3, 2, 1), // -Z
        q(4, 5, 6, 7), // +Z
        q(0, 1, 5, 4), // -Y
        q(2, 3, 7, 6), // +Y
        q(0, 4, 7, 3), // -X
        q(1, 2, 6, 5), // +X
    ]
    .into_iter()
    .flatten()
    .collect()
}

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

fn assert_levels(path: &Toolpath, expected: &[f64]) {
    let actual = distinct_levels(path);
    assert_eq!(actual.len(), expected.len(), "actual={actual:?} expected={expected:?}");
    for (&a, &e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < 1e-8, "actual={actual:?} expected={expected:?}");
    }
}

fn gouges(w: &[Warning]) -> usize {
    w.iter().filter(|w| matches!(w, Warning::Gouge { .. })).count()
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

fn contour_params(
    floor_stock: f64,
    bottom_z: f64,
    stepdown: f64,
    equal_steps: bool,
    finish_levels: u32,
    finish_stepdown: f64,
) -> OpParams {
    OpParams::Contour {
        floor_stock,
        stock_to_leave: 0.0,
        clearance_z: Height::fixed(10.0),
        retract_z: Height::fixed(5.0),
        top_z: Height::fixed(0.0),
        bottom_z: Height::fixed(bottom_z),
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
        tilt: None,
        chain_overrides: Vec::new(),
        passes: WallPasses::default(),
    }
}

fn pocket_params(
    floor_stock: f64,
    bottom_z: f64,
    stepdown: f64,
    equal_steps: bool,
    finish_levels: u32,
    finish_stepdown: f64,
) -> OpParams {
    OpParams::Pocket {
        floor_stock,
        stepover: 3.0,
        stock_to_leave: 0.0,
        clearance_z: Height::fixed(10.0),
        retract_z: Some(Height::fixed(5.0)),
        top_z: Height::fixed(0.0),
        bottom_z: Height::fixed(bottom_z),
        stepdown,
        feed: 800.0,
        plunge_feed: 200.0,
        climb: true,
        strategy: PocketStrategy::Zigzag,
        entry: Entry3D::Plunge,
        helix_radius: 1.5,
        links: Links {
            approach: Approach {
                equal_steps,
                finish_levels,
                finish_stepdown,
                ..Approach::DEFAULT
            },
            ..Links::PLANAR
        },
        optimal_load: 0.25,
        blend_turns: false,
        blend_radius: None,
        rest: None,
        corner_relief: Default::default(),
        tilt: None,
        pass_start: Default::default(),
        finish: Default::default(),
    }
}

fn face_params(
    floor_stock: f64,
    cut_z: f64,
    stepdown: f64,
    finish_levels: u32,
    finish_stepdown: f64,
) -> OpParams {
    OpParams::Face {
        floor_stock,
        stepover: 4.0,
        clearance_z: Height::fixed(10.0),
        cut_z: Height::fixed(cut_z),
        top_z: Some(Height::fixed(0.0)),
        stepdown: Some(stepdown),
        feed: 1000.0,
        plunge_feed: 250.0,
        retract_feed: 0.0,
        finish_levels,
        finish_stepdown,
        climb: true,
        blend_turns: false,
        blend_radius: None,
        holes: FaceHoles::Lift,
        tilt: None,
    }
}

#[test]
fn s8_check_9_floor_allowance_interaction_on_contour_pocket_and_face() {
    // 30 x 20 x 10 mm boss sitting in [-10, 0] inside an 80 x 60 x 10 mm stock.
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        ImportModel::from_mesh("boss", Mesh::from_facets(box_facets(-15.0, -10.0, -10.0, 15.0, 10.0, 0.0))),
    );
    history.execute(
        &world,
        SetStock::new(
            &world,
            StockShape::Box { dx: 80.0, dy: 60.0, dz: 10.0 },
            Location { pos: Point3::new(0.0, 0.0, -10.0), ..Default::default() },
        )
        .expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, TOOL, tool(TOOL)).expect("tool"));

    let expected = [-2.0, -4.0, -4.95, -5.2, -5.45, -5.7];

    // Contour with S2 floor_stock = 0.3 on Check 4 (top 0, bottom -6, stepdown 2, n=3, s=0.25).
    let cp = contour_params(0.3, -6.0, 2.0, false, 3, 0.25);
    history.execute(&world, CreateOperation::new(&world, "C9", TOOL, SrcRef::Entity("boss".into()), cp).expect("create C9"));
    history.execute(&world, GenerateOperation::new(&world, "C9").expect("gen C9"));
    let ce = model::find_by_name(&world, "C9").expect("C9");
    assert_eq!(operation::status_of(&world, &ce), OpStatus::Ok);
    let ctp = model::toolpath_of(&world, &ce).expect("C9 tp");
    assert_levels(&ctp.path, &expected);
    assert_eq!(gouges(&ctp.warnings), 0, "{:?}", ctp.warnings);

    // Pocket (stock source around boss) with S2 floor_stock = 0.3 on Check 4.
    let pp = pocket_params(0.3, -6.0, 2.0, false, 3, 0.25);
    history.execute(&world, CreateOperation::new(&world, "P9", TOOL, SrcRef::Stock, pp).expect("create P9"));
    history.execute(&world, GenerateOperation::new(&world, "P9").expect("gen P9"));
    let pe = model::find_by_name(&world, "P9").expect("P9");
    let ptp = model::toolpath_of(&world, &pe).expect("P9 tp");
    assert_levels(&ptp.path, &expected);
    assert_eq!(gouges(&ptp.warnings), 0, "{:?}", ptp.warnings);

    // Face on stock-only world with S2 floor_stock = 0.3 on Check 4.
    let w_face = new_world();
    let mut h_face = History::default();
    h_face.execute(
        &w_face,
        SetStock::new(
            &w_face,
            StockShape::Box { dx: 80.0, dy: 60.0, dz: 10.0 },
            Location { pos: Point3::new(0.0, 0.0, -10.0), ..Default::default() },
        )
        .expect("stock"),
    );
    h_face.execute(&w_face, AddTool::new(&w_face, TOOL, tool(TOOL)).expect("tool"));
    let fp = face_params(0.3, -6.0, 2.0, 3, 0.25);
    h_face.execute(&w_face, CreateOperation::new(&w_face, "F9", TOOL, SrcRef::Stock, fp).expect("create F9"));
    h_face.execute(&w_face, GenerateOperation::new(&w_face, "F9").expect("gen F9"));
    let fe = model::find_by_name(&w_face, "F9").expect("F9");
    let ftp = model::toolpath_of(&w_face, &fe).expect("F9 tp");
    assert_levels(&ftp.path, &expected);
    assert_eq!(gouges(&ftp.warnings), 0, "{:?}", ftp.warnings);

    // Check 8 via full app pipeline (80x60 stock, top 0, cut -1, stepdown 0.6, n=1, s=0.3).
    let fp8 = face_params(0.0, -1.0, 0.6, 1, 0.3);
    h_face.execute(&w_face, CreateOperation::new(&w_face, "F8", TOOL, SrcRef::Stock, fp8).expect("create F8"));
    h_face.execute(&w_face, GenerateOperation::new(&w_face, "F8").expect("gen F8"));
    let fe8 = model::find_by_name(&w_face, "F8").expect("F8");
    let ftp8 = model::toolpath_of(&w_face, &fe8).expect("F8 tp");
    assert_levels(&ftp8.path, &[-0.6, -0.7, -1.0]);
    assert_eq!(gouges(&ftp8.warnings), 0, "{:?}", ftp8.warnings);

    // Floor stock reducing effective depth so n*s >= depth refuses by name at generation.
    let fp_ref = face_params(5.5, -6.0, 2.0, 3, 0.25); // effective z_b = -0.5, depth = 0.5, n*s = 0.75 >= 0.5
    h_face.execute(&w_face, CreateOperation::new(&w_face, "F_ref", TOOL, SrcRef::Stock, fp_ref).expect("create F_ref"));
    let err = GenerateOperation::new(&w_face, "F_ref")
        .err()
        .expect("n*s >= effective depth refused")
        .to_string();
    assert!(err.contains("0.750") && err.contains("0.500"), "{err}");
}

#[test]
fn s8_check_10_project_file_legacy_load_and_roundtrip_and_stamp() {
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        SetStock::new(
            &world,
            StockShape::Box { dx: 80.0, dy: 60.0, dz: 10.0 },
            Location { pos: Point3::new(0.0, 0.0, -10.0), ..Default::default() },
        )
        .expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, TOOL, tool(TOOL)).expect("tool"));

    let c_def = contour_params(0.0, -3.81, 1.905, false, 0, 0.0);
    let c_set = contour_params(0.0, -3.81, 1.905, true, 1, 0.508);
    let p_def = pocket_params(0.0, -6.0, 2.0, false, 0, 0.0);
    let p_set = pocket_params(0.0, -6.0, 2.0, false, 3, 0.25);
    let f_def = face_params(0.0, -1.0, 0.6, 0, 0.0);
    let f_set = face_params(0.0, -1.0, 0.6, 1, 0.3);

    // Defaults omit finish_levels / finish_stepdown from JSON so old files stay byte-identical.
    for (k, def) in [("contour", &c_def), ("pocket", &p_def), ("face", &f_def)] {
        let j = project::params_to_json(def);
        let s = serde_json::to_string(&j).expect("json");
        assert!(!s.contains("finish_levels") && !s.contains("finish_stepdown"), "{k}: {s}");
        let back = project::params_from_stored_json(&j).expect("legacy load");
        assert_eq!(back, *def, "legacy load of {k} yields default");
    }

    // Non-defaults serialize and roundtrip identically through full project save/load.
    history.execute(&world, CreateOperation::new(&world, "C_s8", TOOL, SrcRef::Stock, c_set.clone()).expect("C_s8"));
    history.execute(&world, CreateOperation::new(&world, "P_s8", TOOL, SrcRef::Stock, p_set.clone()).expect("P_s8"));
    history.execute(&world, CreateOperation::new(&world, "F_s8", TOOL, SrcRef::Stock, f_set.clone()).expect("F_s8"));

    let path = std::env::temp_dir().join(format!("camcraft-s8-roundtrip-{}.ccproj", std::process::id()));
    let _ = std::fs::remove_file(&path);
    project::save(&world, &path).expect("save project");
    let loaded = new_world();
    project::load(&loaded, &path).expect("load project");
    let _ = std::fs::remove_file(&path);

    for (name, want) in [("C_s8", &c_set), ("P_s8", &p_set), ("F_s8", &f_set)] {
        let e = model::find_by_name(&loaded, name).expect("op present");
        let got = model::operation_of(&loaded, &e).expect("op component").params;
        assert_eq!(&got, want, "project roundtrip mismatch for {name}");
    }

    // Stamp digest changes when finish_levels / finish_stepdown change.
    let s_c0 = operation::current_stamp(&world, TOOL, &SrcRef::Stock, &c_def, "C_s8", None);
    let s_c1 = operation::current_stamp(&world, TOOL, &SrcRef::Stock, &c_set, "C_s8", None);
    assert_ne!(s_c0, s_c1);
    let s_f0 = operation::current_stamp(&world, TOOL, &SrcRef::Stock, &f_def, "F_s8", None);
    let s_f1 = operation::current_stamp(&world, TOOL, &SrcRef::Stock, &f_set, "F_s8", None);
    assert_ne!(s_f0, s_f1);
}

#[test]
fn s8_command_and_mcp_refusals_and_strict_gating() {
    let world = new_world();
    let mut history = History::default();
    history.execute(
        &world,
        SetStock::new(
            &world,
            StockShape::Box { dx: 80.0, dy: 60.0, dz: 10.0 },
            Location { pos: Point3::new(0.0, 0.0, -10.0), ..Default::default() },
        )
        .expect("stock"),
    );
    history.execute(&world, AddTool::new(&world, TOOL, tool(TOOL)).expect("tool"));

    // Negative and non-finite finish_stepdown refused by name at command creation and edit.
    history.execute(
        &world,
        CreateOperation::new(&world, "F_ok", TOOL, SrcRef::Stock, face_params(0.0, -1.0, 0.6, 0, 0.0)).expect("F_ok"),
    );
    history.execute(
        &world,
        CreateOperation::new(&world, "C_ok", TOOL, SrcRef::Stock, contour_params(0.0, -4.0, 2.0, false, 0, 0.0))
            .expect("C_ok"),
    );
    history.execute(
        &world,
        CreateOperation::new(&world, "P_ok", TOOL, SrcRef::Stock, pocket_params(0.0, -4.0, 2.0, false, 0, 0.0))
            .expect("P_ok"),
    );

    for bad in [-0.1, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for (edit_name, p) in [
            ("F_ok", face_params(0.0, -1.0, 0.6, 1, bad)),
            ("C_ok", contour_params(0.0, -4.0, 2.0, false, 1, bad)),
            ("P_ok", pocket_params(0.0, -4.0, 2.0, false, 1, bad)),
        ] {
            let c_err = CreateOperation::new(&world, "bad_new", TOOL, SrcRef::Stock, p.clone())
                .err()
                .expect("create refused")
                .to_string();
            let e_err = EditOperationParams::new(&world, edit_name, p)
                .err()
                .expect("edit refused")
                .to_string();
            assert!(c_err.contains("finish_stepdown"), "{c_err}");
            assert!(e_err.contains("finish_stepdown"), "{e_err}");
        }
    }

    // Strict command gating: EVERY non-supporting Links op refuses non-default finish_levels/finish_stepdown.
    let non_supporting = [
        OpType::Adaptive2D,
        OpType::Offset2D,
        OpType::Circular,
        OpType::Slot,
        OpType::Bore,
        OpType::Pocket3D,
        OpType::RotaryOffset,
        OpType::MultiAxisRough,
    ];
    for kind in non_supporting {
        let mut p = operation::seed_params(kind.ordinal());
        if let Some(l) = p.links_mut() {
            l.approach.finish_levels = 1;
            l.approach.finish_stepdown = 0.25;
        }
        assert!(!p.is_sane(), "{kind:?} must fail is_sane with finish_levels");
        let why = p.named_refusal().expect("named refusal on unsupported kind");
        assert!(why.contains("finish_levels") && why.contains("Contour, Pocket, and Face"), "{kind:?}: {why}");
    }

    // MCP end-to-end: set, reset to 0/0.0, reject negative values, gate off unsupported ops.
    history.execute(&world, AddTool::new(&world, "DR-0400", tool("DR-0400")).expect("drill tool"));
    let mcp_proj = std::env::temp_dir().join(format!("camcraft-s8-mcp-{}.camproj", std::process::id()));
    let _ = std::fs::remove_file(&mcp_proj);
    project::save(&world, &mcp_proj).expect("save mcp project");
    let mut server = Server::new();
    let (err, body) = mcp_call(&mut server, 1, "open_project", json!({ "path": mcp_proj.to_string_lossy() }));
    let _ = std::fs::remove_file(&mcp_proj);
    assert!(!err, "open_project: {body}");

    // Face: set & reset via MCP
    let (err, body) = mcp_call(&mut server, 3, "add_operation", json!({ "kind": "Face", "name": "MFace", "tool": TOOL }));
    assert!(!err, "add MFace: {body}");
    let (err, body) = mcp_call(&mut server, 4, "edit_operation", json!({
        "name": "MFace",
        "params": { "finish_levels": 2, "finish_stepdown": 0.2 }
    }));
    assert!(!err, "face set: {body}");
    let (err, body) = mcp_call(&mut server, 5, "edit_operation", json!({
        "name": "MFace",
        "params": { "finish_levels": 0, "finish_stepdown": 0.0 }
    }));
    assert!(!err, "face reset to default: {body}");
    // Negative finish_levels and negative finish_stepdown refused via MCP
    let (err, body) = mcp_call(&mut server, 6, "edit_operation", json!({
        "name": "MFace",
        "params": { "finish_levels": -1 }
    }));
    assert!(err, "negative finish_levels must refuse: {body}");
    let (err, body) = mcp_call(&mut server, 7, "edit_operation", json!({
        "name": "MFace",
        "params": { "finish_stepdown": -0.25 }
    }));
    assert!(err && body.as_str().unwrap_or("").contains("finish_stepdown"), "negative finish_stepdown: {body}");

    // Contour & Pocket: set & reset via MCP
    for (id_base, kind, name) in [(10, "Contour", "MContour"), (20, "Pocket", "MPocket")] {
        let (err, body) = mcp_call(&mut server, id_base, "add_operation", json!({ "kind": kind, "name": name, "tool": TOOL }));
        assert!(!err, "add {kind}: {body}");
        let (err, body) = mcp_call(&mut server, id_base + 1, "edit_operation", json!({
            "name": name,
            "params": { "links": { "approach": { "finish_levels": 2, "finish_stepdown": 0.3 } } }
        }));
        assert!(!err, "{kind} set: {body}");
        let (err, body) = mcp_call(&mut server, id_base + 2, "edit_operation", json!({
            "name": name,
            "params": { "links": { "approach": { "finish_levels": 0, "finish_stepdown": 0.0 } } }
        }));
        assert!(!err, "{kind} reset: {body}");
        let (err, body) = mcp_call(&mut server, id_base + 3, "edit_operation", json!({
            "name": name,
            "params": { "links": { "approach": { "finish_levels": 0, "bogus_key": 1 } } }
        }));
        assert!(err, "{kind} unknown sub-key must fail: {body}");
    }

    // Unsupported ops via MCP: Adaptive2D, Pocket3D, Drill must all refuse finish_levels/finish_stepdown.
    for (id_base, kind, op_tool, name, patch) in [
        (30, "2D Adaptive", TOOL, "MAdapt", json!({ "links": { "approach": { "finish_levels": 1, "finish_stepdown": 0.2 } } })),
        (40, "2D Adaptive", TOOL, "MAdapt0", json!({ "links": { "approach": { "finish_levels": 0 } } })),
        (50, "3D Pocket", TOOL, "MP3D", json!({ "links": { "approach": { "finish_levels": 1 } } })),
        (60, "Drill", "DR-0400", "MDrill", json!({ "finish_levels": 1 })),
    ] {
        let (err, body) = mcp_call(&mut server, id_base, "add_operation", json!({ "kind": kind, "name": name, "tool": op_tool }));
        assert!(!err, "add {kind}: {body}");
        let (err, body) = mcp_call(&mut server, id_base + 1, "edit_operation", json!({ "name": name, "params": patch }));
        assert!(err, "{kind} must refuse finish_levels via MCP: {body}");
    }
}
