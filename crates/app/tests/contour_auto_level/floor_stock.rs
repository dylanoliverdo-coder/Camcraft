//! Provenance: Journeyman spec S2, synthetic geometry and a nominal floor offset.

use camcraft_app::commands::SetStock;
use camcraft_app::geometry::Point3;
use camcraft_app::machining2d::PocketStrategy;
use camcraft_app::operation::{retype_params, OpType};
use camcraft_app::posting;
use camcraft_app::project::{params_from_json, params_from_stored_json, params_to_json};
use camcraft_app::schema::Location;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::Tool;

use super::*;

fn contour_with_floor(stock: Option<f64>) -> (OpParams, Toolpath) {
    let outline = rect(-30.0, -20.0, 30.0, 20.0);
    let mut facets = Vec::new();
    plate(&outline, None, -10.0, 0.0, &mut facets);
    let (world, mut history) = world_with(facets);
    let src = SrcRef::Selection(top_face(&world, 0.0));
    let mut wire = params_to_json(&auto());
    if let Some(stock) = stock {
        wire["contour"]["floor_stock"] = stock.into();
    }
    let params = params_from_json(&wire).expect("contour parameters");
    let create = CreateOperation::new(&world, "C", TOOL, src, params.clone()).expect("create contour");
    history.execute(&world, create);
    history.execute(&world, GenerateOperation::new(&world, "C").expect("generate contour"));
    let e = model::find_by_name(&world, "C").expect("contour");
    let cached = model::toolpath_of(&world, &e).expect("cached contour");
    (params, cached.path.clone())
}

fn floor(path: &Toolpath) -> f64 {
    path.moves.iter().filter(|m| m.kind != MoveKind::Rapid).map(|m| m.to.z).fold(f64::INFINITY, f64::min)
}

fn assert_levels(path: &Toolpath, want: &[f64]) {
    let mut actual = Vec::<f64>::new();
    for m in &path.moves {
        if m.kind == MoveKind::Feed && (m.from.z - m.to.z).abs() < 1e-9
            && !actual.iter().any(|&z| (m.to.z - z).abs() < 1e-8)
        {
            actual.push(m.to.z);
        }
    }
    assert_eq!(actual.len(), want.len(), "expected {want:?}, found {actual:?}");
    for (&seen, &want) in actual.iter().zip(want) {
        assert!((seen - want).abs() < 1e-8, "expected {want}, found {seen}");
    }
}

fn last_level_xy(path: &Toolpath) -> Vec<[f64; 4]> {
    let z = floor(path);
    path.moves.iter()
        .filter(|m| m.kind == MoveKind::Feed && (m.from.z - z).abs() < 1e-8 && (m.to.z - z).abs() < 1e-8)
        .map(|m| [m.from.x, m.from.y, m.to.x, m.to.y])
        .collect()
}

#[test]
fn contour_floor_stock_moves_the_last_level() {
    let (_, control) = contour_with_floor(None);
    let xy = last_level_xy(&control);
    assert!(!xy.is_empty());
    for stock in [0.5, -0.5] {
        let (params, path) = contour_with_floor(Some(stock));
        assert!((floor(&path) - (-10.0 + stock)).abs() < 1e-8, "floor stock {stock}: actual floor {}", floor(&path));
        assert_eq!(last_level_xy(&path), xy, "contour walls are unchanged in XY");
        assert_wire_stock("contour", &params, stock);
    }
}

fn assert_wire_stock(kind: &str, params: &OpParams, stock: f64) {
    let wire = params_to_json(params);
    assert_eq!(wire[kind]["floor_stock"].as_f64(), Some(stock));
    assert_eq!(params_from_stored_json(&wire).expect("read saved parameters"), *params);
}

fn add_stock(world: &World, history: &mut History, dx: f64, dy: f64) {
    let at = Location { pos: Point3::new(0.0, 0.0, -10.0), ..Default::default() };
    let stock = StockShape::Box { dx, dy, dz: 10.0 };
    history.execute(world, SetStock::new(world, stock, at).expect("stock box"));
}

fn pocket_job(part_bottom: f64) -> (World, History, SrcRef) {
    let outside = rect(-30.0, -25.0, 30.0, 25.0);
    let inside = rect(-20.0, -15.0, 20.0, 15.0);
    let mut facets = Vec::new();
    plate(&outside, Some((&inside, -10.0)), part_bottom, 0.0, &mut facets);
    let (world, mut history) = world_with(facets);
    add_stock(&world, &mut history, 60.0, 50.0);
    let part = model::find_by_name(&world, "part").expect("part");
    let features = model::features_of(&world, &part).expect("features");
    let rim: Vec<_> = feature::chains(&features).into_iter().filter(|c| {
        feature::chain_points(&features, c).iter().all(|p| {
            p[2].abs() < 1e-8 && p[0].abs() <= 20.0 + 1e-8 && p[1].abs() <= 15.0 + 1e-8
        })
    }).map(|c| GeoSel { entity: "part".into(), feature: FeatureSel::Chain(c.id) }).collect();
    assert!(!rim.is_empty(), "40 by 30 pocket rim");
    (world, history, SrcRef::Selection(rim))
}

fn pocket_params(stock: f64, even: bool) -> OpParams {
    let mut params = retype_params(&auto(), OpType::Pocket as usize);
    if let OpParams::Pocket {
        floor_stock, top_z, bottom_z, clearance_z, retract_z, stepdown,
        strategy, entry, helix_radius, stepover, stock_to_leave, links, ..
    } = &mut params {
        *floor_stock = stock;
        *top_z = Height::fixed(0.0);
        *bottom_z = Height::fixed(-10.0);
        *clearance_z = Height::fixed(5.0);
        *retract_z = None;
        *stepdown = 2.5;
        *strategy = PocketStrategy::Zigzag;
        *entry = Entry3D::Helix;
        *helix_radius = 2.0;
        *stepover = 2.0;
        *stock_to_leave = 0.2;
        links.approach.equal_steps = even;
    } else {
        panic!("Pocket seed");
    }
    params
}

fn run(world: &World, history: &mut History, src: SrcRef, params: OpParams) -> (Toolpath, Vec<Warning>) {
    history.execute(world, CreateOperation::new(world, "test cut", TOOL, src, params).expect("create"));
    history.execute(world, GenerateOperation::new(world, "test cut").expect("generate"));
    let e = model::find_by_name(world, "test cut").expect("operation");
    let cached = model::toolpath_of(world, &e).expect("cached path");
    (cached.path.clone(), cached.warnings.clone())
}

fn pocket(stock: f64, even: bool, part_bottom: f64) -> (Toolpath, Vec<Warning>) {
    let (world, mut history, src) = pocket_job(part_bottom);
    let params = pocket_params(stock, even);
    if stock != 0.0 {
        assert_wire_stock("pocket", &params, stock);
    }
    run(&world, &mut history, src, params)
}

#[test]
fn pocket_floor_stock_keeps_walls_and_reuses_each_levels_entry() {
    let (default, _) = pocket(0.0, false, -10.0);
    let xy = last_level_xy(&default);
    assert!(!xy.is_empty());
    for (stock, even, levels) in [
        (0.3, false, vec![-2.5, -5.0, -7.5, -9.7]),
        (0.3, true, vec![-2.425, -4.85, -7.275, -9.7]),
        (-0.2, false, vec![-2.5, -5.0, -7.5, -10.0, -10.2]),
    ] {
        let (path, warnings) = pocket(stock, even, -10.0);
        assert_levels(&path, &levels);
        assert_eq!(last_level_xy(&path), xy, "pocket walls are unchanged in XY");
        assert_eq!(gouges(&warnings), 0, "{warnings:?}");
        for z in levels {
            assert!(path.moves.iter().any(|m| m.arc.is_some() && m.from.z > m.to.z && (m.to.z - z).abs() < 1e-8), "missing helix to {z}");
        }
    }
}

#[test]
fn pocket_floor_stock_refusals_name_the_field_at_creation_and_edit() {
    let (world, mut history, src) = pocket_job(-10.0);
    history.execute(&world, CreateOperation::new(&world, "valid", TOOL, src.clone(), pocket_params(0.0, false)).expect("create"));
    for bad in [10.0, 11.0, f64::NAN, f64::INFINITY] {
        let create = CreateOperation::new(&world, "bad", TOOL, src.clone(), pocket_params(bad, false));
        let edit = camcraft_app::commands::EditOperationParams::new(&world, "valid", pocket_params(bad, false));
        for err in [create.err(), edit.err()] {
            let err = err.expect("bad floor stock refused").to_string();
            assert!(err.contains("Floor stock"), "{err}");
        }
    }
}

#[test]
fn a_negative_pocket_floor_stock_still_checks_the_blind_floor() {
    // Picked blind pocket: pocket_ladder now includes floor_stock (-0.2 -> -10.200), so
    // pocket_region_on_part refuses the blind-floor intrusion before generating any toolpath.
    let (world, mut history, src) = pocket_job(-12.0);
    history.execute(&world, CreateOperation::new(&world, "test cut", TOOL, src, pocket_params(-0.2, false)).expect("create"));
    let gen_err = GenerateOperation::new(&world, "test cut").err().expect("blind floor refused at generate").to_string();
    assert!(gen_err.contains("-10.200") && gen_err.contains("floor"), "{gen_err}");
    let nc = std::env::temp_dir().join(format!("camcraft-s2-blind-{}.nc", std::process::id()));
    let _ = std::fs::remove_file(&nc);
    assert!(posting::post_setup(&world, &nc).is_err(), "un-generated op must not post");
    assert!(!nc.exists(), "the refused post writes no NC");

    // Face with negative floor_stock into a solid part's top (part top at Z=0, cut_z=0, floor_stock=-0.2):
    // verifier flags a blocking gouge and post_setup refuses writing NC.
    let outer = rect(-30.0, -20.0, 30.0, 20.0);
    let mut facets = Vec::new();
    plate(&outer, None, -10.0, 0.0, &mut facets);
    let (w2, mut h2) = world_with(facets);
    add_stock(&w2, &mut h2, 80.0, 60.0);
    let mut fp = retype_params(&auto(), OpType::Face as usize);
    if let OpParams::Face { cut_z, clearance_z, top_z, stepdown, stepover, floor_stock, .. } = &mut fp {
        *cut_z = Height::fixed(0.0);
        *clearance_z = Height::fixed(5.0);
        *top_z = None;
        *stepdown = None;
        *stepover = 3.0;
        *floor_stock = -0.2;
    }
    let (_, warnings) = run(&w2, &mut h2, SrcRef::Stock, fp);
    assert!(gouges(&warnings) > 0, "{warnings:?}");
    let refusal = posting::post_setup(&w2, &nc).err().expect("the gouge blocks posting");
    assert!(refusal.to_string().to_lowercase().contains("gouge"), "{refusal}");
    assert!(!nc.exists(), "the refused post writes no NC");
}

fn facer() -> Tool {
    catalog().into_iter().find(|(name, _)| *name == TOOL).expect("catalog mill").1
}

#[test]
fn face_floor_stock_shifts_staged_and_single_cut_levels() {
    for staged in [true, false] {
        for stock in [0.5, -0.5] {
            let world = new_world();
            let mut history = History::default();
            history.execute(&world, AddTool::new(&world, TOOL, facer()).expect("tool"));
            add_stock(&world, &mut history, 100.0, 80.0);
            let mut params = retype_params(&auto(), OpType::Face as usize);
            if let OpParams::Face { cut_z, clearance_z, top_z, stepdown, stepover, floor_stock, .. } = &mut params {
                *cut_z = Height::fixed(-5.0);
                *clearance_z = Height::fixed(5.0);
                *top_z = staged.then_some(Height::fixed(0.0));
                *stepdown = staged.then_some(2.5);
                *stepover = 3.0;
                *floor_stock = stock;
            } else {
                panic!("Face seed");
            }
            assert_wire_stock("face", &params, stock);
            let (path, warnings) = run(&world, &mut history, SrcRef::Stock, params);
            assert!((floor(&path) - (-5.0 + stock)).abs() < 1e-8, "staged={staged}, allowance={stock}, got {}", floor(&path));
            assert_eq!(gouges(&warnings), 0, "{warnings:?}");
        }
    }
}

#[test]
fn legacy_and_explicit_zero_floor_stock_keep_the_contour() {
    let (legacy, path) = contour_with_floor(None);
    let (zero, control) = contour_with_floor(Some(0.0));
    assert_eq!(floor(&path), -10.0);
    assert_eq!(path, control);
    assert_eq!(legacy, zero);
    assert!(params_to_json(&legacy)["contour"].get("floor_stock").is_none());
}

#[test]
fn positive_floor_stock_clears_a_blind_pocket_floor_and_steps_above_a_submerged_island() {
    // Blind pocket floor at -10.0 (part extends down to -12.0). Raw bottom_z = -10.2 sits below
    // the blind floor, but +0.5 floor stock raises the last cut to -9.7 (safely above -10.0).
    let (world, mut history, src) = pocket_job(-12.0);
    let mut params = pocket_params(0.5, false);
    if let OpParams::Pocket { bottom_z, .. } = &mut params {
        *bottom_z = Height::fixed(-10.2);
    }
    let (path, warnings) = run(&world, &mut history, src, params);
    assert!((floor(&path) - -9.7).abs() < 1e-8, "got floor {}", floor(&path));
    assert_eq!(gouges(&warnings), 0, "{warnings:?}");

    // Whole-model pocket around a 20x20 boss on a 60x50 stock, raw bottom -10.0, floor_stock +2.5 -> -7.5.
    let boss = rect(-10.0, -10.0, 10.0, 10.0);
    let mut boss_facets = Vec::new();
    plate(&boss, None, -10.0, 0.0, &mut boss_facets);
    let (w2, mut h2) = world_with(boss_facets);
    add_stock(&w2, &mut h2, 60.0, 50.0);
    let (path_whole, w_whole) = run(&w2, &mut h2, SrcRef::Entity("part".into()), pocket_params(2.5, false));
    assert!((floor(&path_whole) - -7.5).abs() < 1e-8, "whole-model floor {}", floor(&path_whole));
    assert_eq!(gouges(&w_whole), 0, "{w_whole:?}");

    // Stock-source pocket around the same part with floor_stock +2.5 -> -7.5.
    let mut boss_facets2 = Vec::new();
    plate(&boss, None, -10.0, 0.0, &mut boss_facets2);
    let (w3, mut h3) = world_with(boss_facets2);
    add_stock(&w3, &mut h3, 60.0, 50.0);
    let (path_stock, w_stock) = run(&w3, &mut h3, SrcRef::Stock, pocket_params(2.5, false));
    assert!((floor(&path_stock) - -7.5).abs() < 1e-8, "stock-source floor {}", floor(&path_stock));
    assert_eq!(gouges(&w_stock), 0, "{w_stock:?}");
}

#[test]
fn contour_face_sanity_refusals_and_unresolved_ref_symmetry() {
    // Contour & staged Face refusals at creation and edit, plus is_sane / named_refusal on non-finite.
    let outline = rect(-30.0, -20.0, 30.0, 20.0);
    let mut facets = Vec::new();
    plate(&outline, None, -10.0, 0.0, &mut facets);
    let (world, mut history) = world_with(facets);
    let src = SrcRef::Selection(top_face(&world, 0.0));
    add_stock(&world, &mut history, 100.0, 80.0);

    let mut c_ok = auto();
    if let OpParams::Contour { top_z, bottom_z, .. } = &mut c_ok {
        *top_z = Height::fixed(0.0);
        *bottom_z = Height::fixed(-10.0);
    }
    history.execute(&world, CreateOperation::new(&world, "c_ok", TOOL, src.clone(), c_ok.clone()).expect("create contour"));
    for bad in [10.0, 11.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut p = c_ok.clone();
        if let OpParams::Contour { floor_stock, .. } = &mut p {
            *floor_stock = bad;
        }
        if !bad.is_finite() {
            assert!(!p.is_sane(), "non-finite contour floor_stock must fail is_sane");
            assert!(p.named_refusal().is_some_and(|w| w.contains("Floor stock")));
        } else {
            assert!(!camcraft_app::operation::depth_sane(&world, None, Some(&src), &p));
        }
        let c_err = CreateOperation::new(&world, "c_bad", TOOL, src.clone(), p.clone()).err().expect("contour refused").to_string();
        let e_err = camcraft_app::commands::EditOperationParams::new(&world, "c_ok", p).err().expect("contour edit refused").to_string();
        assert!(c_err.contains("Floor stock") && e_err.contains("Floor stock"), "{c_err} | {e_err}");
    }

    // Staged Face refusal when cut_z + floor_stock >= top_z or non-finite.
    let mut f_ok = retype_params(&auto(), OpType::Face as usize);
    if let OpParams::Face { top_z, cut_z, stepdown, .. } = &mut f_ok {
        *top_z = Some(Height::fixed(0.0));
        *cut_z = Height::fixed(-5.0);
        *stepdown = Some(2.5);
    }
    history.execute(&world, CreateOperation::new(&world, "f_ok", TOOL, SrcRef::Stock, f_ok.clone()).expect("create face"));
    for bad in [5.0, 6.0, f64::NAN, f64::INFINITY] {
        let mut p = f_ok.clone();
        if let OpParams::Face { floor_stock, .. } = &mut p {
            *floor_stock = bad;
        }
        if !bad.is_finite() {
            assert!(!p.is_sane());
            assert!(p.named_refusal().is_some_and(|w| w.contains("Floor stock")));
        } else {
            assert!(!camcraft_app::operation::depth_sane(&world, None, Some(&SrcRef::Stock), &p));
        }
        let c_err = CreateOperation::new(&world, "f_bad", TOOL, SrcRef::Stock, p.clone()).err().expect("face refused").to_string();
        let e_err = camcraft_app::commands::EditOperationParams::new(&world, "f_ok", p).err().expect("face edit refused").to_string();
        assert!(c_err.contains("Floor stock") && e_err.contains("Floor stock"), "{c_err} | {e_err}");
    }

    // Negative radial stock_to_leave stays refused even when floor_stock is negative.
    let mut neg_rad = pocket_params(-0.2, false);
    if let OpParams::Pocket { stock_to_leave, .. } = &mut neg_rad {
        *stock_to_leave = -0.1;
    }
    assert!(!neg_rad.is_sane());

    // Unresolved non-selection height reference (TopOfStock with a part but no stock in world)
    // stays allowed at creation time symmetrically with floor_stock == 0.0.
    let mut bare_facets = Vec::new();
    plate(&outline, None, -10.0, 0.0, &mut bare_facets);
    let (bare, _) = world_with(bare_facets);
    let bare_src = SrcRef::Entity("part".into());
    let mut unres = retype_params(&auto(), OpType::Face as usize);
    if let OpParams::Face { top_z, cut_z, stepdown, floor_stock, .. } = &mut unres {
        *top_z = Some(Height::of(HeightReference::TopOfStock, 0.0));
        *cut_z = Height::of(HeightReference::TopOfStock, -2.0);
        *stepdown = Some(1.0);
        *floor_stock = 0.5;
    }
    assert!(camcraft_app::operation::depth_sane(&bare, None, Some(&bare_src), &unres));
    assert!(CreateOperation::new(&bare, "pending_stock", TOOL, bare_src, unres).is_ok());
}

#[test]
fn region_z_setup_sheet_digest_and_undo_follow_floor_stock() {
    let world = new_world();
    let mut history = History::default();
    history.execute(&world, AddTool::new(&world, TOOL, facer()).expect("tool"));
    add_stock(&world, &mut history, 100.0, 80.0);

    let mut f0 = retype_params(&auto(), OpType::Face as usize);
    if let OpParams::Face { top_z, cut_z, stepdown, floor_stock, .. } = &mut f0 {
        *top_z = None;
        *cut_z = Height::fixed(-5.0);
        *stepdown = None;
        *floor_stock = 0.0;
    }
    let mut f_pos = f0.clone();
    if let OpParams::Face { floor_stock, .. } = &mut f_pos {
        *floor_stock = 0.5;
    }
    // Unstaged Face region_z shifts by floor_stock.
    assert_eq!(camcraft_app::operation::region_z(&world, None, Some(&SrcRef::Stock), &f0), Some(-5.0));
    assert_eq!(camcraft_app::operation::region_z(&world, None, Some(&SrcRef::Stock), &f_pos), Some(-4.5));

    // Parameter stamp changes when floor_stock != 0.0 and stays identical for 0.0 vs -0.0.
    history.execute(&world, CreateOperation::new(&world, "F", TOOL, SrcRef::Stock, f0.clone()).expect("create"));
    let s0 = camcraft_app::operation::current_stamp(&world, TOOL, &SrcRef::Stock, &f0, "F", None);
    let mut f_neg0 = f0.clone();
    if let OpParams::Face { floor_stock, .. } = &mut f_neg0 {
        *floor_stock = -0.0;
    }
    assert_eq!(s0, camcraft_app::operation::current_stamp(&world, TOOL, &SrcRef::Stock, &f_neg0, "F", None));

    let assert_nominal_cut = |sheet: &str| {
        let row = sheet.lines().find(|row| row.split_whitespace().nth(1) == Some("F")).expect("Face row");
        assert_eq!(row.split_whitespace().last(), Some("-5.000"), "{row}");
    };
    // Undo and redo track the allowance while the setup-sheet Cut Z stays nominal.
    history.execute(&world, camcraft_app::commands::EditOperationParams::new(&world, "F", f_pos.clone()).expect("edit +0.5"));
    assert_ne!(s0, camcraft_app::operation::current_stamp(&world, TOOL, &SrcRef::Stock, &f_pos, "F", None));
    let sheet = camcraft_app::setup_sheet::render(&world, None).expect("setup sheet");
    assert_nominal_cut(&sheet);
    assert!(sheet.lines().any(|line| line.trim() == "Floor stock +0.500 mm"), "{sheet}");
    assert!(history.undo(&world));
    let sheet_undone = camcraft_app::setup_sheet::render(&world, None).expect("setup sheet undone");
    assert_nominal_cut(&sheet_undone);
    assert!(!sheet_undone.contains("Floor stock"), "{sheet_undone}");
    assert!(history.redo(&world));
    let sheet_redone = camcraft_app::setup_sheet::render(&world, None).expect("setup sheet redone");
    assert_eq!(sheet_redone, sheet);
}
