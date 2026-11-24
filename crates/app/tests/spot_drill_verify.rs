//! Provenance: issue #3020 and machine-tool handbook cone geometry on an original synthetic blind bore.
//! Gate map: covers app::operation::generate, app::posting, sim::verify::section.

use flecs_ecs::prelude::World;

use camcraft_app::commands::{AddTool, CreateOperation, GenerateOperation, History, ImportModel, SetStock};
use camcraft_app::drilling::{CycleChoice, Hole};
use camcraft_app::geo2d::Point2;
use camcraft_app::geometry::{Mesh, Point3, Triangle};
use camcraft_app::model;
use camcraft_app::operation::{self, Height, OpParams, OpStatus, SrcRef};
use camcraft_app::posting;
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::{ToolKind, catalog};
use camcraft_app::verify::Warning;
use camcraft_app::world::new_world;

const TOP: f64 = 12.0;
const FLOOR: f64 = 2.0;
const BORE: f64 = 2.5;
const SEGS: usize = 64;
const FANUC: &str = include_str!("../../../assets/posts/fanuc.cps");

/// Revolve an outward-wound meridian into a closed solid with a blind bore.
fn bored_blank(floor: f64, tessellated_top: bool) -> Mesh {
    let mut meridian = vec![(0.0, 0.0), (10.0, 0.0), (10.0, TOP)];
    if tessellated_top {
        meridian.push((BORE + 0.125, TOP));
    }
    meridian.extend([(BORE, TOP), (BORE, floor), (0.0, floor)]);
    let point = |j: usize, radius: f64, z: f64| {
        let angle = std::f64::consts::TAU * j as f64 / SEGS as f64;
        Point3::new(radius * angle.cos(), radius * angle.sin(), z)
    };
    let mut facets = Vec::new();
    for segment in meridian.windows(2) {
        let ((ra, za), (rb, zb)) = (segment[0], segment[1]);
        for j in 0..SEGS {
            let (a1, a2, b1, b2) = (point(j, ra, za), point(j + 1, ra, za), point(j, rb, zb), point(j + 1, rb, zb));
            if ra <= 1e-12 {
                facets.push(Triangle::new(a1, b2, b1));
            } else if rb <= 1e-12 {
                facets.push(Triangle::new(a1, a2, b2));
            } else {
                facets.push(Triangle::new(a1, a2, b2));
                facets.push(Triangle::new(a1, b2, b1));
            }
        }
    }
    Mesh::from_facets(facets)
}

fn generated_spot(depth: f64) -> (World, History) {
    configured_spot(depth, TOP, 6.0, 120.0)
}

fn configured_spot(depth: f64, stock_top: f64, diameter: f64, tip_angle: f64) -> (World, History) {
    configured_spot_with_chamfer(depth, stock_top, diameter, tip_angle, None)
}

#[derive(Clone, Copy)]
struct ChamferFixture {
    width: f64,
    tip_diameter: f64,
    floor: f64,
    center_x: f64,
    tessellated_top: bool,
}

fn configured_spot_with_chamfer(
    depth: f64, stock_top: f64, diameter: f64, tip_angle: f64, chamfer: Option<ChamferFixture>,
) -> (World, History) {
    let world = new_world();
    let mut history = History::default();
    let floor = chamfer.map_or(FLOOR, |c| c.floor);
    history.execute(&world, ImportModel::from_mesh("blank", bored_blank(floor, chamfer.is_some_and(|c| c.tessellated_top))));
    history.execute(
        &world,
        SetStock::new(&world, StockShape::Box { dx: 20.0, dy: 20.0, dz: stock_top }, Location::default()).expect("stock"),
    );
    let mut tool = catalog().into_iter().find(|(name, _)| *name == "DR-0600").map(|(_, tool)| tool).expect("catalog drill");
    tool.diameter = diameter;
    tool.tip_angle = tip_angle;
    if let Some(c) = chamfer {
        tool.kind = ToolKind::Taper;
        tool.tip_diameter = c.tip_diameter;
    }
    let saved_tool = camcraft_app::project::tool_to_json(&tool);
    assert_eq!(saved_tool.get("tip_diameter").is_some(), tool.tip_diameter != 0.0);
    let loaded_tool = camcraft_app::project::tool_from_json(&saved_tool).expect("load spot tool");
    assert_eq!(loaded_tool, tool);
    history.execute(&world, AddTool::new(&world, "Spot6", loaded_tool).expect("spot tool"));
    let params = OpParams::Drill {
        chamfer_width: chamfer.map_or(0.0, |c| c.width),
        break_through: None,
        break_through_from: Default::default(),
        clearance_z: Height::fixed(TOP + 15.0),
        retract_z: None,
        // Raw stock needs an R plane at 13; the part top stays 12 and the safe tip ends at 10.
        top_z: Height::fixed(stock_top),
        bottom_z: Some(Height::fixed(TOP - depth)),
        cycle: CycleChoice::Spot,
        peck: 0.0,
        dwell: 0.0,
        feed: 120.0,
        plunge_feed: 80.0,
        holes: vec![Hole {
            xy: Point2::new(chamfer.map_or(0.0, |c| c.center_x), 0.0),
            radius: BORE, entry_z: TOP, floor_z: floor, through: false, stepped: false,
        }],
        cross: vec![],
    };
    history.execute(
        &world,
        CreateOperation::new(&world, "Spot", "Spot6", SrcRef::Entity("blank".into()), params).expect("create spot"),
    );
    history.execute(&world, GenerateOperation::new(&world, "Spot").expect("generate spot"));
    (world, history)
}

fn nc_path(tag: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("camcraft-spot-3020-{tag}-{}.nc", std::process::id()))
}

#[test]
fn shallow_spot_cone_inside_pilot_verifies_and_posts() {
    // At 1 mm depth, the 120-degree cone reaches radius tan(60) = 1.732 mm.
    // The nominal 3 mm radius remains above the top of the 2.5 mm bore.
    let (world, _history) = generated_spot(1.0);
    let entity = model::find_by_name(&world, "Spot").expect("spot entity");
    let cached = model::toolpath_of(&world, &entity).expect("spot toolpath");
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok, "the shallow cone clears the bore: {:?}", cached.warnings);
    assert!(cached.warnings.is_empty(), "the shallow cone clears the bore: {:?}", cached.warnings);
    assert!((cached.path.cycles[0].bottom_z - 11.0).abs() < 1e-9, "the test cuts only 1 mm deep");
    let nc = nc_path("safe");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let posted = posted.expect("the shallow spot posts");
    assert!(posted.program.contains("G81") || posted.program.contains("G82"), "the spot cycle posts: {}", posted.program);
}

#[test]
fn spot_cone_that_enlarges_pilot_still_blocks_post() {
    // At 2 mm depth the full 3 mm drill radius enters the 2.5 mm bore.
    let (world, _history) = generated_spot(2.0);
    let entity = model::find_by_name(&world, "Spot").expect("spot entity");
    let cached = model::toolpath_of(&world, &entity).expect("spot toolpath");
    assert!(cached.warnings.iter().any(|warning| matches!(warning, Warning::Gouge { .. })), "a real wall cut must have a gouge warning: {:?}", cached.warnings);
    let nc = nc_path("gouge");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let error = posted.expect_err("the enlarging spot must not post");
    assert!(error.contains("gouge") && error.contains("cuts below the model"), "the refusal names the model gouge: {error}");
}

#[test]
fn spot_cone_below_extra_stock_clears_the_part_and_posts() {
    // The cone radius is 2 mm at the part top, but 3 mm at the stock top.
    let (world, _history) = configured_spot(2.0, TOP + 1.0, 8.0, 90.0);
    let entity = model::find_by_name(&world, "Spot").expect("spot entity");
    let cached = model::toolpath_of(&world, &entity).expect("spot toolpath");
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok, "the cone clears the finished bore: {:?}", cached.warnings);
    assert!(!cached.warnings.iter().any(|warning| matches!(warning, Warning::Gouge { .. })), "the stock above the part is not a finished wall: {:?}", cached.warnings);
    assert!((cached.path.cycles[0].bottom_z - 10.0).abs() < 1e-9, "the spot is 2 mm deep");
    let nc = nc_path("extra-stock-safe");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let posted = posted.expect("the spot below extra stock posts");
    assert!(posted.program.contains("G81") || posted.program.contains("G82"), "the spot cycle posts: {}", posted.program);
}

#[test]
fn spot_cone_below_extra_stock_that_cuts_the_part_blocks_post() {
    // At 3 mm depth, the cone radius is 3 mm at the part top.
    let (world, _history) = configured_spot(3.0, TOP + 1.0, 8.0, 90.0);
    let entity = model::find_by_name(&world, "Spot").expect("spot entity");
    let cached = model::toolpath_of(&world, &entity).expect("spot toolpath");
    assert!(cached.warnings.iter().any(|warning| matches!(warning, Warning::Gouge { .. })), "the cone cuts the finished wall: {:?}", cached.warnings);
    let nc = nc_path("extra-stock-gouge");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let error = posted.expect_err("the cone that cuts the finished wall must not post");
    assert!(error.contains("gouge"), "the refusal names the gouge: {error}");
}

const CHAMFER: ChamferFixture = ChamferFixture {
    width: 0.5, tip_diameter: 0.5, floor: FLOOR, center_x: 0.0, tessellated_top: false,
};

#[test]
fn a_truncated_chamfer_mill_cuts_and_posts_the_declared_mouth() {
    let (world, _) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(CHAMFER));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    let cached = model::toolpath_of(&world, &entity).expect("spot path");
    assert!((cached.path.cycles[0].bottom_z - (TOP - 2.75)).abs() < 1e-9);
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok, "{:?}", cached.warnings);
    let nc = nc_path("declared-chamfer");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let posted = posted.expect("the bounded chamfer posts");
    assert!(posted.program.contains("G81") || posted.program.contains("G82"));
}

#[test]
fn a_subdivided_top_and_fitted_bore_do_not_hold_the_chamfer_post() {
    let geometry = ChamferFixture { center_x: 0.0002, tessellated_top: true, ..CHAMFER };
    let (world, _) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(geometry));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    let cached = model::toolpath_of(&world, &entity).expect("spot path");
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok, "{:?}", cached.warnings);
    assert!(!cached.warnings.iter().any(Warning::blocks_post), "{:?}", cached.warnings);
    let nc = nc_path("chamfer-split");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    posted.expect("an annulus subdivided across its width is still the top face");
}

#[test]
fn turning_chamfer_width_off_requires_a_compatible_tool() {
    use camcraft_app::commands::EditOperationParams;

    let (world, _) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(CHAMFER));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    let mut params = model::operation_of(&world, &entity).expect("spot parameters").params;
    let OpParams::Drill { chamfer_width, .. } = &mut params else { panic!("Drill parameters") };
    *chamfer_width = 0.0;
    let refused = EditOperationParams::new(&world, "Spot", params).err().expect("turning width off needs a Drill tool");
    assert!(refused.to_string().contains("positive Chamfer width"), "{refused}");
}

#[test]
fn a_chamfer_tip_that_reaches_through_the_pilot_floor_blocks_post() {
    let shallow = ChamferFixture { floor: TOP - 2.5, ..CHAMFER };
    assert_chamfer_gouge_blocks_post(shallow, "chamfer-floor");
}

#[test]
fn an_off_center_path_cannot_borrow_the_declared_chamfer() {
    let shifted = ChamferFixture { center_x: 0.5, ..CHAMFER };
    assert_chamfer_gouge_blocks_post(shifted, "chamfer-off-center");
}

fn assert_chamfer_gouge_blocks_post(chamfer: ChamferFixture, tag: &str) {
    let (world, _) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(chamfer));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    let cached = model::toolpath_of(&world, &entity).expect("spot path");
    assert!(cached.warnings.iter().any(|warning| matches!(warning, Warning::Gouge { .. })), "{:?}", cached.warnings);
    let nc = nc_path(tag);
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let refusal = posted.expect_err("a chamfer may not post over another surface");
    assert!(refusal.contains("gouge"), "{refusal}");
}

#[test]
fn the_program_check_does_not_credit_a_cached_cut_below_the_declared_chamfer() {
    let (world, _) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(CHAMFER));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    let cached = model::toolpath_of(&world, &entity).expect("spot path");
    assert!(cached.warnings.is_empty(), "{:?}", cached.warnings);
    let mut path = cached.path;
    let tip = path.cycles[0].bottom_z;
    for m in &mut path.moves {
        if m.from.z == tip { m.from.z -= 0.5; }
        if m.to.z == tip { m.to.z -= 0.5; }
    }
    path.cycles[0].bottom_z -= 0.5;
    entity.entity_view(&world).set(camcraft_app::schema::Toolpath::new(path, Vec::new()));

    let simulated = camcraft_app::stock_sim::simulate_document(&world).expect("program verdict");
    assert!(simulated.dev.gouge_cells > 0, "the declaration does not grant the deeper cached cut");
    let nc = nc_path("chamfer-lowered-cache");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let refusal = posted.expect_err("the program check must hold the over-depth cut");
    assert!(refusal.contains("gouge"), "{refusal}");
}

#[test]
fn a_later_part_in_the_tip_flat_blocks_post_without_staling_the_source() {
    let (world, mut history) = configured_spot_with_chamfer(2.0, TOP, 12.0, 90.0, Some(CHAMFER));
    let entity = model::find_by_name(&world, "Spot").expect("spot operation");
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok);
    let block = camcraft_app::geometry::cube(Point3::new(0.17, -0.03, 9.20), Point3::new(0.22, 0.03, 9.31));
    history.execute(&world, ImportModel::from_mesh("late test block", block));
    assert_eq!(operation::status_of(&world, &entity), OpStatus::Ok, "source is unchanged");

    let nc = nc_path("chamfer-tip-flat-keep");
    let posted = posting::post_script(&world, &nc, FANUC);
    let _ = std::fs::remove_file(&nc);
    let refusal = posted.expect_err("the tip flat hits the neighboring block");
    assert!(refusal.contains("late test block"), "{refusal}");
}
