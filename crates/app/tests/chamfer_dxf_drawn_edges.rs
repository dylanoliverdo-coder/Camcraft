//! Journeyman spec S10 follow-up: a 2D Chamfer on a DXF profile breaks the
//! edges its chains are drawn on, over a part that is a separate model. Its
//! source is neither the stock nor the part, so it took the stock-source law
//! (#1631: any break into a part is a gouge) at generation, and a
//! whole-profile source counted as "no picks" in the sim (#2596: credit
//! dropped wherever the model carries a modelled chamfer). Now a part on
//! whose edges every break stands (#2435's wall, and no part higher than
//! the drawn edge beside it) is the profile's own part, judged as a part
//! source's own part is: in the program sim, with the break credited; a
//! break off every wall of a part, or drawn below its edge, still gouges
//! it, by name.
//!
//! Gate map: covers app::operation (keep_law, keep_law_drawn, generate, job,
//! kept_rejudge), app::stock_sim (breaks::program_breaks, breaks::auto_edges),
//! sim::stock_sim (wall_anchor::on_drawn_edges), sim::verify (Warning),
//! app::posting (verification_gate).
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! synthetic fixtures over our own commands and stock sim; DXF geometry uses
//! public ASCII DXF group codes; the chamfer mill's cone law is machine-tool
//! handbook geometry; no vendor parameters, text or code.

use std::path::PathBuf;

use flecs_ecs::prelude::World;

use camcraft_app::commands::{
    AddTool, Command, CommandError, CreateOperation, DxfOptions, GenerateOperation, History, ImportDxf, ImportModel,
    SetStock,
};
use camcraft_app::geometry::{Mesh, Point3, Triangle};
use camcraft_app::machining2d::Side;
use camcraft_app::model;
use camcraft_app::operation::{self, FeatureSel, GeoSel, Height, HeightReference, OpParams, OpType, SrcRef};
use camcraft_app::posting;
use camcraft_app::schema::Location;
use camcraft_app::sketch;
use camcraft_app::stock::StockShape;
use camcraft_app::tools::catalog;
use camcraft_app::verify::Warning;
use camcraft_app::world::new_world;

/// A 90-degree chamfer mill, 6 mm across.
const MILL: &str = "CM-0600-90";

type Chain<'a> = (&'a [(f64, f64)], bool);

fn tmp_dir(tag: &str) -> PathBuf {
    let p = std::env::temp_dir().join(format!("camcraft-s10-drawn-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&p);
    std::fs::create_dir_all(&p).expect("temp dir");
    p
}

/// A millimetre DXF holding the given chains as LWPOLYLINEs on layer "c".
fn dxf(chains: &[Chain]) -> String {
    let mut s = String::from("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n");
    for (pts, closed) in chains {
        s.push_str(&format!("0\nLWPOLYLINE\n8\nc\n90\n{}\n70\n{}\n", pts.len(), u8::from(*closed)));
        for (x, y) in *pts {
            s.push_str(&format!("10\n{x}\n20\n{y}\n"));
        }
    }
    s.push_str("0\nENDSEC\n0\nEOF\n");
    s
}

fn run(world: &World, history: &mut History, cmd: Result<impl Command + 'static, CommandError>, what: &str) {
    history.execute(world, cmd.unwrap_or_else(|e| panic!("{what}: {e}")));
}

fn quad(facets: &mut Vec<Triangle>, a: Point3, b: Point3, c: Point3, d: Point3) {
    facets.extend([Triangle::new(a, b, c), Triangle::new(a, c, d)]);
}

/// A sharp prism over the convex CCW `ring`, z `z0` to `z1`.
fn prism(ring: &[(f64, f64)], z0: f64, z1: f64) -> Mesh {
    let p = |i: usize, z| Point3::new(ring[i].0, ring[i].1, z);
    let mut facets = Vec::new();
    for i in 1..ring.len() - 1 {
        facets.push(Triangle::new(p(0, z1), p(i, z1), p(i + 1, z1)));
        facets.push(Triangle::new(p(0, z0), p(i + 1, z0), p(i, z0)));
    }
    for i in 0..ring.len() {
        let j = (i + 1) % ring.len();
        quad(&mut facets, p(i, z0), p(j, z0), p(j, z1), p(i, z1));
    }
    Mesh::from_facets(facets)
}

/// #2596's plate: 60 x 40 x 10 centred in plan, a modelled 1 x 45 chamfer
/// on its four outer top edges (walls x = +-30, y = +-20 up to z 9, top
/// face inset 1 mm), and a SHARP 20 x 10 pocket down to z 5.
fn bevelled_plate_with_sharp_pocket() -> Mesh {
    let ring = |hx: f64, hy: f64, z| [Point3::new(-hx, -hy, z), Point3::new(hx, -hy, z), Point3::new(hx, hy, z), Point3::new(-hx, hy, z)];
    let (bottom, wall, top) = (ring(30.0, 20.0, 0.0), ring(30.0, 20.0, 9.0), ring(29.0, 19.0, 10.0));
    let (rim, floor) = (ring(10.0, 5.0, 10.0), ring(10.0, 5.0, 5.0));
    let mut facets = Vec::new();
    quad(&mut facets, bottom[3], bottom[2], bottom[1], bottom[0]);
    quad(&mut facets, floor[0], floor[1], floor[2], floor[3]);
    for i in 0..4 {
        let j = (i + 1) % 4;
        quad(&mut facets, bottom[i], bottom[j], wall[j], wall[i]);
        quad(&mut facets, wall[i], wall[j], top[j], top[i]);
        quad(&mut facets, top[i], top[j], rim[j], rim[i]);
        quad(&mut facets, rim[i], rim[j], floor[j], floor[i]);
    }
    Mesh::from_facets(facets)
}

/// The 40 x 20 brick's plan, centred on the origin.
const BRICK: [(f64, f64); 4] = [(-20.0, -10.0), (20.0, -10.0), (20.0, 10.0), (-20.0, 10.0)];
/// A loop inside the brick's top face, 10 mm in from every wall.
const INSIDE: [(f64, f64); 4] = [(-10.0, -5.0), (10.0, -5.0), (10.0, 5.0), (-10.0, 5.0)];
/// The brick's plan moved 50 mm along +x, where no part stands.
const SECOND: [(f64, f64); 4] = [(30.0, -10.0), (70.0, -10.0), (70.0, 10.0), (30.0, 10.0)];
/// #2596's plate: its outer edge (modelled chamfer) and its sharp pocket rim.
const PLATE: [(f64, f64); 4] = [(-30.0, -20.0), (30.0, -20.0), (30.0, 20.0), (-30.0, 20.0)];
const RIM: [(f64, f64); 4] = [(-10.0, -5.0), (10.0, -5.0), (10.0, 5.0), (-10.0, 5.0)];

/// A 0.5 x 45 2D Chamfer whose top plane stands `top` from the stock's top.
fn chamfer(top: f64) -> OpParams {
    let mut p = operation::seed_params(OpType::Chamfer2D.ordinal());
    if let OpParams::Chamfer2D { width, angle_deg, clearance_z, retract_z, top_z, climb, side, .. } = &mut p {
        *width = 0.5;
        *angle_deg = 45.0;
        *clearance_z = Height::of(HeightReference::TopOfStock, 10.0);
        *retract_z = None;
        *top_z = Height::of(HeightReference::TopOfStock, top);
        *climb = true;
        *side = Side::Auto;
    }
    p
}

/// The part `part` (a model of its own; the stock is cut from it, as the
/// program ahead leaves it - the #2435 tests' setup), and op "OP": a 2D
/// Chamfer on the DXF profile `tag` holding `chains` - every chain picked,
/// or with `whole` the profile itself as the source. Returns the world and
/// the op's generation warnings.
fn drawn(tag: &str, part: Mesh, chains: &[Chain], top: f64, whole: bool) -> (World, Vec<Warning>) {
    let world = new_world();
    let mut h = History::default();
    h.execute(&world, ImportModel::from_mesh("part", part));
    run(&world, &mut h, SetStock::new(&world, StockShape::FromModel { source: "part".into() }, Location::default()), "stock");
    let tool = catalog().into_iter().find(|(n, _)| *n == MILL).map(|(_, t)| t).expect("catalog chamfer mill");
    run(&world, &mut h, AddTool::new(&world, MILL, tool), "tool");
    let path = tmp_dir(tag).join(format!("{tag}.dxf"));
    std::fs::write(&path, dxf(chains)).expect("write dxf");
    run(&world, &mut h, ImportDxf::new(&world, None, &path, &DxfOptions::default()), "import dxf");
    let (_, sk) = sketch::find(&world, tag).expect("profile");
    let src = if whole {
        SrcRef::Entity(tag.into())
    } else {
        SrcRef::Selection(sk.data.chains().all().map(|c| GeoSel { entity: tag.into(), feature: FeatureSel::SketchChain(c.id()) }).collect())
    };
    run(&world, &mut h, CreateOperation::new(&world, "OP", MILL, src, chamfer(top)), "create");
    run(&world, &mut h, GenerateOperation::new(&world, "OP"), "generate");
    let e = model::find_by_name(&world, "OP").expect("op");
    let t = model::toolpath_of(&world, &e).expect("generated");
    assert!(!t.path.breaks.is_empty(), "{tag}: the chamfer declares its breaks");
    (world, t.warnings)
}

fn blocking(w: &[Warning]) -> Vec<String> {
    w.iter().filter(|w| w.blocks_post()).map(ToString::to_string).collect()
}

fn gouge_line(world: &World) -> Option<String> {
    posting::verification_gate(world).into_iter().find(|l| l.starts_with("gouge:"))
}

/// The post's live re-judge of the kept parts (#1628), on the op as cached.
fn rejudged(world: &World) -> Option<String> {
    let e = model::find_by_name(world, "OP").expect("op");
    let op = model::operation_of(world, &e).expect("operation");
    let t = model::toolpath_of(world, &e).expect("cached");
    operation::kept_rejudge::moved_part_problem(world, "OP", &op, &t.path)
}

#[test]
fn a_profile_chamfer_on_a_sharp_part_edge_breaks_it_with_credit() {
    // The stage-1 case: the profile is the part's outline, the part a model
    // apart from the stock. The 0.5 mm break is the drawn edge's own.
    let (world, warns) = drawn("outline", prism(&BRICK, 0.0, 10.0), &[(&BRICK, true)], 0.0, false);
    assert_eq!(blocking(&warns), Vec::<String>::new(), "the break on the part's edge is no gouge");
    assert_eq!(gouge_line(&world), None, "the sim credits the break on the part's wall");
    assert_eq!(rejudged(&world), None, "the post's live re-judge spares the part whose edge it breaks");
}

#[test]
fn a_whole_profile_chamfer_beside_a_modelled_chamfer_keeps_its_credit() {
    // #2596 drops a no-pick chamfer's credit where the model carries a
    // modelled chamfer: the generator chose those edges. A DXF profile is
    // the user's drawing - its chains are picks - so the sharp pocket rim
    // it is drawn on keeps its break.
    let chains: [Chain; 2] = [(&PLATE, true), (&RIM, true)];
    let (world, warns) = drawn("plate", bevelled_plate_with_sharp_pocket(), &chains, 0.0, true);
    assert_eq!(blocking(&warns), Vec::<String>::new(), "the rim's break is the drawing's own");
    assert_eq!(gouge_line(&world), None, "the sim credits the rim's break beside the modelled chamfer");
}

#[test]
fn a_profile_chain_off_every_wall_still_gouges_the_part_by_name() {
    // A loop drawn inside the part's top face: the cone's tip stands inside
    // the part, on no wall. Nothing there is an edge to break.
    let (world, warns) = drawn("inside", prism(&BRICK, 0.0, 10.0), &[(&INSIDE, true)], 0.0, false);
    assert!(warns.iter().any(|w| matches!(w, Warning::ProfileChamferGouge { .. })), "named: {warns:?}");
    let text = blocking(&warns).join("; ");
    assert!(text.contains("profile chamfer breaks into a part away from its edges"), "{text}");
    assert!(gouge_line(&world).is_some(), "the sim judges the cut sharp");
    let live = rejudged(&world).expect("the post's live re-judge finds the cut");
    assert!(live.contains("part 'part'"), "{live}");
}

#[test]
fn a_two_up_profile_breaks_the_placed_copy_and_cuts_air_over_the_missing_one() {
    // Two copies of the outline, the part placed under one: the other copy
    // stands on nothing and cuts only air.
    let chains: [Chain; 2] = [(&BRICK, true), (&SECOND, true)];
    let (world, warns) = drawn("two-up", prism(&BRICK, 0.0, 10.0), &chains, 0.0, false);
    assert_eq!(blocking(&warns), Vec::<String>::new(), "neither copy gouges the placed part");
    assert_eq!(gouge_line(&world), None, "the placed copy's break is credited");
    assert_eq!(rejudged(&world), None);
}

#[test]
fn a_profile_drawn_below_the_parts_edge_cuts_a_groove_not_a_break() {
    // The outline drawn 2 mm below the part's top: air down to the tip on
    // the outside (#2435's wall holds), but the part stands higher than the
    // drawn edge beside it, so the cone cuts a V into the wall, not a break.
    let (world, warns) = drawn("low", prism(&BRICK, 0.0, 10.0), &[(&BRICK, true)], -2.0, false);
    assert!(warns.iter().any(|w| matches!(w, Warning::ProfileChamferGouge { .. })), "named: {warns:?}");
    assert!(gouge_line(&world).is_some(), "the groove is a gouge in the sim");
    assert!(rejudged(&world).is_some(), "and at the post's live re-judge");
}
