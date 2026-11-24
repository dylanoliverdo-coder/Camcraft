//! Journeyman S6 (docs/journeyman/specs/S6-hole-chamfer-width.md); cone
//! geometry after Smid 2007; no vendor parameters, text or code.
//! Gate map: covers app::operation::generate, app::project, motion::drilling.

use camcraft_app::commands::{
    AddTool, CreateOperation, EditOperationParams, GenerateOperation, History, SetStock,
};
use camcraft_app::geometry::Point3;
use camcraft_app::model;
use camcraft_app::operation::{self, OpParams, SrcRef};
use camcraft_app::project::{params_from_json, params_from_stored_json, params_to_json};
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::{ToolKind, catalog};
use camcraft_app::world::new_world;
use serde_json::json;

fn spot_params() -> serde_json::Value {
    let holes = [(-9.0, -9.0), (9.0, -9.0), (9.0, 9.0), (-9.0, 9.0)].map(|(x, y)| {
        json!({"xy": [x, y], "radius": 3.3, "entry_z": 0.0, "floor_z": -8.0, "through": false})
    });
    json!({
        "drill": {
            "clearance_z": 5.0, "retract_z": 2.0, "top_z": 0.0,
            "cycle": "spot", "peck": 0.0, "dwell": 0.25, "feed": 120.0,
            "plunge_feed": 80.0, "holes": holes
        }
    })
}

fn fixture() -> (flecs_ecs::prelude::World, History) {
    let world = new_world();
    let mut history = History::default();
    let stock = StockShape::Box { dx: 40.0, dy: 40.0, dz: 10.0 };
    let at = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Location::default() };
    history.execute(&world, SetStock::new(&world, stock, at).expect("stock"));
    let mut tool = catalog().into_iter().map(|(_, tool)| tool)
        .find(|tool| tool.kind == ToolKind::Drill).expect("catalog drill");
    tool.diameter = 12.0;
    tool.tip_angle = 90.0;
    history.execute(&world, AddTool::new(&world, "Synthetic spot", tool).expect("spot tool"));
    (world, history)
}

fn add_drill(world: &flecs_ecs::prelude::World, history: &mut History, params: OpParams) {
    let create = CreateOperation::new(world, "Chamfer", "Synthetic spot", SrcRef::Stock, params);
    history.execute(world, create.expect("create spot operation"));
}

fn cycles(params: OpParams) -> Vec<camcraft_app::cl::Cycle> {
    let (world, mut history) = fixture();
    add_drill(&world, &mut history, params);
    history.execute(&world, GenerateOperation::new(&world, "Chamfer").expect("generate spot operation"));
    let op = model::find_by_name(&world, "Chamfer").expect("operation");
    model::toolpath_of(&world, &op).expect("generated path").path.cycles.clone()
}

#[test]
fn chamfer_width_drives_the_spot_tip_at_all_four_holes() {
    let mut params = spot_params();
    params["drill"]["chamfer_width"] = json!(0.5);
    params["drill"]["bottom_z"] = json!(-2.0);

    let cuts = cycles(params_from_json(&params).expect("drill parameters"));
    assert_eq!(cuts.len(), 4);
    for cut in cuts {
        assert!((cut.bottom_z + 3.8).abs() < 1e-9, "expected -3.8, got {}", cut.bottom_z);
        assert_eq!(cut.r_z, 0.0);
        assert_eq!(cut.dwell, 0.25);
    }
}

#[test]
fn legacy_spot_uses_the_original_depth_and_omits_width() {
    let params = params_from_json(&spot_params()).expect("legacy drill parameters");
    assert!(params_to_json(&params)["drill"].get("chamfer_width").is_none());
    for cut in cycles(params) {
        assert_eq!(cut.bottom_z, -2.0);
    }
}

#[test]
fn width_round_trips_and_edit_undo_restores_the_stamp_and_sheet() {
    let (world, mut history) = fixture();
    let legacy = params_from_stored_json(&spot_params()).expect("legacy drill parameters");
    add_drill(&world, &mut history, legacy.clone());
    let op = model::find_by_name(&world, "Chamfer").expect("operation");
    let stamp = || {
        let current = model::operation_of(&world, &op).expect("operation params");
        operation::current_stamp(&world, &current.tool, &current.src, &current.params, "Chamfer", None)
            .expect("stamp")
    };
    let before = stamp();
    let original_sheet = camcraft_app::setup_sheet::render(&world, None).expect("setup sheet");

    let mut edited = legacy;
    let OpParams::Drill { chamfer_width, .. } = &mut edited else { unreachable!() };
    *chamfer_width = 0.5;
    let saved = params_to_json(&edited);
    assert_eq!(saved["drill"]["chamfer_width"], json!(0.5));
    assert_eq!(params_to_json(&params_from_stored_json(&saved).expect("reload width")), saved);
    history.execute(&world, EditOperationParams::new(&world, "Chamfer", edited).expect("edit width"));

    let after = stamp();
    assert_ne!(after, before);
    let width_sheet = camcraft_app::setup_sheet::render(&world, None).expect("setup sheet");
    assert!(width_sheet.contains("Chamfer width 0.500 mm"), "{width_sheet}");
    assert_eq!(width_sheet.matches("bottom -3.800").count(), 4, "{width_sheet}");

    assert!(history.undo(&world));
    assert_eq!(stamp(), before);
    assert_eq!(camcraft_app::setup_sheet::render(&world, None).expect("undo sheet"), original_sheet);
    assert!(history.redo(&world));
    assert_eq!(stamp(), after);
    assert_eq!(camcraft_app::setup_sheet::render(&world, None).expect("redo sheet"), width_sheet);
}

#[test]
fn negative_and_non_finite_widths_are_refused_by_create_and_edit() {
    let (world, mut history) = fixture();
    let legacy = params_from_stored_json(&spot_params()).expect("legacy drill parameters");
    add_drill(&world, &mut history, legacy.clone());
    for width in [-0.5, f64::NAN, f64::INFINITY] {
        let mut invalid = legacy.clone();
        let OpParams::Drill { chamfer_width, .. } = &mut invalid else { unreachable!() };
        *chamfer_width = width;
        assert!(!invalid.is_sane());
        let create = CreateOperation::new(&world, "Invalid", "Synthetic spot", SrcRef::Stock, invalid.clone());
        assert!(create.is_err_and(|why| why.to_string().contains("Chamfer width")));
        let edit = EditOperationParams::new(&world, "Chamfer", invalid);
        assert!(edit.is_err_and(|why| why.to_string().contains("Chamfer width")));
    }
}
