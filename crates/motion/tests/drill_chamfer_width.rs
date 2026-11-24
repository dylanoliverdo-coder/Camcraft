//! Journeyman S6 (docs/journeyman/specs/S6-hole-chamfer-width.md); cone
//! geometry after Smid 2007; no vendor parameters, text or code.
//! Gate map: covers motion::drilling, sim::stock_sim.

use camcraft_motion::cl::{CycleKind, MoveKind};
use camcraft_motion::drilling::{self, CycleChoice, CycleParams, Hole};
use camcraft_motion::geo2d::Point2;
use camcraft_motion::stock_sim::{Backend, Columns, SimOp, Stock};
use camcraft_motion::tools::{Tool, ToolKind, catalog};

fn spot_tool(angle: f64) -> Tool {
    let mut tool = catalog().into_iter().map(|(_, tool)| tool)
        .find(|tool| tool.kind == ToolKind::Drill).expect("catalog drill");
    tool.diameter = 12.0;
    tool.tip_angle = angle;
    tool
}

fn hole(x: f64, y: f64, radius: f64) -> Hole {
    Hole { xy: Point2::new(x, y), radius, entry_z: 0.0, floor_z: -8.0, through: false, stepped: false }
}

fn params<'a>(tool: &'a Tool, holes: &'a [Hole], chamfer_width: f64) -> CycleParams<'a> {
    CycleParams {
        clearance_z: 5.0,
        r_z: 0.0,
        hop_z: Some(2.0),
        bottom_z: Some(-2.0),
        chamfer_width,
        cycle: CycleChoice::Spot,
        peck: 0.0,
        dwell: 0.25,
        feed: 120.0,
        plunge_feed: 80.0,
        tool,
        holes,
        through_clear: drilling::THROUGH_CLEAR,
        break_through_from: Default::default(),
    }
}

#[test]
fn depth_reads_the_included_angle_and_each_hole_mouth() {
    let tool = spot_tool(120.0);
    let holes = [hole(-9.0, -9.0, 3.3), Hole { entry_z: -1.0, ..hole(9.0, 9.0, 3.3) }];
    let path = drilling::drill(&params(&tool, &holes, 0.5)).expect("spot two holes");
    assert_eq!(path.cycles.len(), 2);
    for (cycle, hole) in path.cycles.iter().zip(&holes) {
        assert!((cycle.bottom_z - hole.entry_z + 2.193_931_022_92).abs() < 1e-9, "{cycle:?}");
        assert_eq!(cycle.kind, CycleKind::Spot);
        assert_eq!(cycle.r_z, 0.0);
        assert_eq!(cycle.dwell, 0.25);
        let plunge = &path.moves[cycle.first_move];
        assert_eq!(plunge.kind, MoveKind::Plunge);
        assert_eq!(plunge.to.z, cycle.bottom_z);
    }
}

#[test]
fn chamfer_width_changes_depth_without_rewriting_the_cycle_choice() {
    let tool = spot_tool(90.0);
    let holes = [hole(9.0, 9.0, 3.3)];
    let mut p = params(&tool, &holes, 0.5);
    for (choice, kind) in [(CycleChoice::Auto, CycleKind::Drill), (CycleChoice::Peck, CycleKind::Peck)] {
        p.cycle = choice;
        let path = drilling::drill(&p).expect("the selected drilling cycle");
        let cut = &path.cycles[0];
        assert_eq!(cut.kind, kind);
        assert!((cut.bottom_z + 3.8).abs() < 1e-9);
    }
}

#[test]
fn an_oversize_chamfer_skips_only_that_hole_and_names_its_position() {
    let tool = spot_tool(90.0);
    let holes = [hole(-9.0, -9.0, 5.75), hole(9.0, 9.0, 0.0)];
    let path = drilling::drill(&params(&tool, &holes, 1.0)).expect("the point still spots");
    assert_eq!(path.cycles.len(), 1);
    assert_eq!(path.cycles[0].xy, holes[1].xy);
    assert!((path.cycles[0].bottom_z + 1.0).abs() < 1e-9);
    assert!(path.notes.iter().any(|note| {
        note.contains("(-9.000, -9.000)") && note.contains("exceeds the tool body diameter")
    }), "{:?}", path.notes);
    assert!(path.moves.iter().all(|m| m.to.x == 9.0 && m.to.y == 9.0));

    let holes = [hole(-9.0, -9.0, 5.75), hole(9.0, 9.0, 3.3)];
    let path = drilling::drill(&params(&tool, &holes, 0.5)).expect("the second bore still spots");
    assert_eq!(path.cycles.len(), 1);
    assert!((path.cycles[0].bottom_z + 3.8).abs() < 1e-9);
    assert!(path.notes[0].contains("12.500 mm exceeds the tool body diameter 12.000 mm"));
}

#[test]
fn invalid_cones_and_widths_never_return_an_empty_success() {
    let holes = [hole(-9.0, -9.0, 3.3)];
    let mut tool = spot_tool(180.0);
    let failure = drilling::drill(&params(&tool, &holes, 0.5)).expect_err("flat point cannot cut the chamfer");
    assert!(failure.to_string().contains("hole at (-9.000, -9.000)"));
    assert!(failure.to_string().contains("cone angle"));
    tool.kind = ToolKind::EndMill;
    let failure = drilling::drill(&params(&tool, &holes, 0.5)).expect_err("tool has no cone");
    assert!(failure.to_string().contains("no cutting cone"));
    for width in [-0.5, f64::NAN, f64::INFINITY] {
        let failure = drilling::drill(&params(&tool, &holes, width)).expect_err("width is invalid");
        assert!(failure.to_string().contains("Chamfer width"));
    }
}

#[test]
fn simulator_measures_the_chamfer_mouth_and_deepest_sweep() {
    const CELL: f64 = 0.1;
    let tool = spot_tool(90.0);
    let holes = [hole(-9.0, -9.0, 3.3), hole(9.0, -9.0, 3.3), hole(9.0, 9.0, 3.3), hole(-9.0, 9.0, 3.3)];
    let path = drilling::drill(&params(&tool, &holes, 0.5)).expect("spot the synthetic plate");
    assert_eq!(path.cycles.len(), holes.len());
    let mut stock = Stock::block(Backend::Column, -16.05, -16.05, 16.05, 16.05, 0.0, CELL).expect("stock");
    stock.simulate_drill(&[SimOp {
        moves: &path.moves,
        radius: tool.diameter * 0.5,
        ball: false,
        cone: Some((tool.tip_angle.to_radians() * 0.5).tan().recip()),
        corner: None,
    }]).expect("simulate the pointed tool");

    for hole in holes {
        let touched = (-50..=50).filter(|i| stock.top(hole.xy.x + f64::from(*i) * CELL, hole.xy.y) < -1e-8).count();
        let diameter = touched as f64 * CELL;
        assert!((diameter - 7.6).abs() <= CELL + 1e-9, "simulated mouth diameter {diameter}");
        assert!((stock.top(hole.xy.x, hole.xy.y) + 3.8).abs() <= CELL);
    }
    let columns = Columns::of_grid(stock.column().expect("dexel result"));
    let mut deepest = 0.0_f64;
    for iy in 0..columns.ny() {
        for ix in 0..columns.nx() {
            deepest = deepest.min(columns.top_at(ix, iy));
        }
    }
    assert!((deepest + 3.8).abs() <= CELL && deepest >= -3.8 - 1e-9, "deepest simulated Z {deepest}");
}
