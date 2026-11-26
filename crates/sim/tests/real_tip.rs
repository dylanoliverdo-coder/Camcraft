//! M13.7: the verifier judges a 2D op with the op's real tip, not the
//! square stack. A ball contour pass along a wall whose foot is broken
//! by a 45-degree chamfer: the square end's rim stands in the chamfer
//! (the old verdict convicted it), the ball's dome clears it - and the
//! same ball dipped into the floor is still a gouge. A tip that does
//! not match the context's tool geometry is never trusted to clear.
//!
//! Provenance: our own fixtures, specced from the Prime Directive and
//! the handbook side view of a ball-nose end mill; no vendor material.
//!
//! Gate map (M13.7): covers sim::verify (engine b + b2, probe
//! Cutter::judged / tip_height, VerifyContext::cutter).

use camcraft_sim::cl::{MoveKind, PathBuilder, Toolpath};
use camcraft_sim::geo2d::{Loop, Point2, Region};
use camcraft_sim::geometry::{Point3, Triangle};
use camcraft_sim::verify::{Cutter, Mode, ToolGeom, VerifyContext, Warning, verify};

const R: f64 = 3.0;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// An axis-aligned box, outward-wound.
fn boxf(min: [f64; 3], max: [f64; 3]) -> Vec<Triangle> {
    let v = |i: usize| p([min[0], max[0]][i & 1], [min[1], max[1]][(i >> 1) & 1], [min[2], max[2]][(i >> 2) & 1]);
    let quad = |a: usize, b: usize, c: usize, d: usize| [Triangle::new(v(a), v(b), v(c)), Triangle::new(v(a), v(c), v(d))];
    [quad(4, 5, 7, 6), quad(0, 2, 3, 1), quad(0, 1, 5, 4), quad(2, 6, 7, 3), quad(1, 3, 7, 5), quad(2, 0, 4, 6)]
        .into_iter()
        .flatten()
        .collect()
}

/// A wall at x >= 3 standing on a floor at z 0, its foot broken by a
/// 1 mm 45-degree chamfer prism (x 2..3, z 0..1), all along y -10..10.
fn chamfered_wall() -> Vec<Triangle> {
    let mut f = boxf([3.0, -10.0, 0.0], [10.0, 10.0, 10.0]);
    f.extend(boxf([-10.0, -10.0, -5.0], [10.0, 10.0, 0.0]));
    let tri = Triangle::new;
    let (a, b, c) = (p(2.0, -10.0, 0.0), p(3.0, -10.0, 0.0), p(3.0, -10.0, 1.0));
    let (a2, b2, c2) = (p(2.0, 10.0, 0.0), p(3.0, 10.0, 0.0), p(3.0, 10.0, 1.0));
    f.extend([
        tri(a, b, c),
        tri(a2, c2, b2),
        tri(a, c2, a2),
        tri(a, c, c2),
        tri(a, a2, b2),
        tri(a, b2, b),
        tri(b, b2, c2),
        tri(b, c2, c),
    ]);
    f
}

/// The contour's finish boundary: the wall's own face at x 3.
fn wall_region() -> Region {
    let pts = [(3.0, -10.0), (10.0, -10.0), (10.0, 10.0), (3.0, 10.0)];
    let lp = Loop::new(pts.iter().map(|&(x, y)| Point2::new(x, y)).collect());
    match lp.and_then(|l| Region::new(vec![l])) {
        Ok(r) => r,
        Err(e) => panic!("fixture region: {e}"),
    }
}

/// One contour pass along the wall at x 0, tip at `z`.
fn pass(z: f64) -> Toolpath {
    let mut b = PathBuilder::new(p(0.0, -8.0, 30.0));
    b.push(MoveKind::Rapid, p(0.0, -8.0, z + 5.0), None);
    b.push(MoveKind::Plunge, p(0.0, -8.0, z), Some(100.0));
    b.push(MoveKind::Feed, p(0.0, 8.0, z), Some(500.0));
    b.push(MoveKind::Rapid, p(0.0, 8.0, 30.0), None);
    b.finish()
}

fn geom() -> ToolGeom {
    ToolGeom {
        radius: R,
        flute_len: 20.0,
        shank_radius: R,
        shank_len: 30.0,
        holder_radius: 16.0,
        holder_len: 40.0,
        nose_radius: 0.0,
    }
}

fn ball() -> Cutter {
    Cutter { corner_r: R, ..Cutter::flat(&geom()) }
}

fn judge(t: &Toolpath, cutter: Option<Cutter>) -> Vec<Warning> {
    let (f, region, g) = (chamfered_wall(), wall_region(), geom());
    verify(
        t,
        &VerifyContext {
            region: &region,
            stock: None,
            model: Some(&f),
            tool: &g,
            cutter,
            mode: Mode::Boundary,
            stock_top_z: 10.0,
            clearance_z: 30.0,
            stock_to_leave: 0.0,
            gouge_tol: 0.01,
            z_tol: 1e-6,
            finish_tol: None,
            fixtures: &[],
            spindle: None,
            ..Default::default()
        },
    )
}

fn gouges(w: &[Warning]) -> Vec<usize> {
    w.iter().filter_map(|w| if let Warning::Gouge { at } = w { Some(*at) } else { None }).collect()
}

#[test]
fn a_ball_contour_over_a_chamfered_foot_clears_where_the_square_stack_convicted() {
    let t = pass(0.05);
    let square = judge(&t, None);
    assert!(gouges(&square).contains(&2), "the square end's rim stands in the chamfer: {square:?}");
    let real = judge(&t, Some(ball()));
    assert!(gouges(&real).is_empty(), "the ball's dome clears the chamfer: {real:?}");
}

#[test]
fn the_real_ball_still_convicts_a_true_gouge() {
    // The same pass 0.2 mm into the floor: the ball's bottom is in steel.
    let t = pass(-0.2);
    let real = judge(&t, Some(ball()));
    assert!(gouges(&real).contains(&2), "a ball under the floor is a gouge: {real:?}");
    // And sideways into the wall foot: tip on the floor, axis 1 mm in.
    let mut b = PathBuilder::new(p(1.0, -8.0, 30.0));
    b.push(MoveKind::Rapid, p(1.0, -8.0, 5.05), None);
    b.push(MoveKind::Plunge, p(1.0, -8.0, 0.05), Some(100.0));
    b.push(MoveKind::Feed, p(1.0, 8.0, 0.05), Some(500.0));
    let real = judge(&b.finish(), Some(ball()));
    assert!(gouges(&real).contains(&2), "the dome in the chamfer and wall is a gouge: {real:?}");
}

#[test]
fn a_flat_taper_hits_steel_that_a_sharp_tip_clears() {
    use camcraft_sim::cl::Move;
    use camcraft_sim::geometry::cube;
    use camcraft_sim::verify::{solid_hits, SolidLaw};
    use camcraft_tools::{taper::TaperTip, tools::{Tool, catalog}};

    let (_, sharp) = catalog().into_iter().find(|(name, _)| *name == "CM-0600-90").expect("taper");
    let flat = Tool { tip_form: TaperTip::Flat, tip_diameter: 1.0, ..sharp.clone() };
    let path = Toolpath::new(vec![Move::new(MoveKind::Feed, p(-2.0, 0.0, 0.0), p(2.0, 0.0, 0.0), Some(200.0))]);
    let law = SolidLaw { step: 0.02, allow: 0.001, hole_walls: false };
    let near = cube(p(-0.1, 0.35, 0.025), p(0.1, 0.40, 0.08));
    let clear = cube(p(-0.1, 0.65, 0.025), p(0.1, 0.70, 0.08));
    assert!(solid_hits(&path, &near.facets, &Cutter::of(&sharp), &law).is_empty());
    assert_eq!(gouges(&solid_hits(&path, &near.facets, &Cutter::of(&flat), &law)), [0]);
    assert!(solid_hits(&path, &clear.facets, &Cutter::of(&flat), &law).is_empty());
}

#[test]
fn a_flat_taper_rapid_sees_the_uncut_groove_shoulder() {
    use camcraft_sim::cl::Move;
    use camcraft_tools::{taper::TaperTip, tools::{Tool, catalog}};

    let (_, sharp) = catalog().into_iter().find(|(name, _)| *name == "CM-0600-90").expect("taper");
    let tool = Tool { tip_form: TaperTip::Flat, tip_diameter: 1.0, ..sharp };
    let geometry = tool.geom();
    let stock = Region::new(vec![Loop::new(vec![
        Point2::new(-8.0, -8.0), Point2::new(8.0, -8.0), Point2::new(8.0, 8.0), Point2::new(-8.0, 8.0),
    ]).expect("stock loop")]).expect("stock");
    let empty = Region::empty();
    let cx = VerifyContext {
        region: &empty, stock: Some(&stock), tool: &geometry, cutter: Some(Cutter::of(&tool)), mode: Mode::Engrave,
        stock_top_z: 0.8, clearance_z: 0.2, gouge_tol: 0.01, z_tol: 1e-6, ..Default::default()
    };
    let feed = Move::new(MoveKind::Feed, p(-2.0, 0.0, 0.0), p(0.0, 0.0, 0.0), Some(200.0));
    let rapid = |from: Point3, to: Point3| Move::new(MoveKind::Rapid, from, to, None);
    let across = Toolpath::new(vec![feed, rapid(feed.to, p(0.0, 0.0, 0.2)), rapid(p(0.0, 0.0, 0.2), p(0.0, 0.5, 0.2))]);
    assert!(verify(&across, &cx).contains(&Warning::RapidInStock { at: 2 }));
    let reenter = Toolpath::new(vec![feed, rapid(feed.to, p(0.0, 0.0, 1.0)), rapid(p(0.0, 0.0, 1.0), p(0.0, 0.0, 0.5))]);
    assert!(!verify(&reenter, &cx).contains(&Warning::RapidInStock { at: 2 }));
}

#[test]
fn a_tip_that_disagrees_with_the_tool_is_never_trusted_to_clear() {
    let g = geom();
    // A smaller ball than the tool the other laws read: the square stack
    // stands in, so the chamfer conviction stays.
    let small = Cutter { radius: 2.0, corner_r: 2.0, ..Cutter::flat(&g) };
    assert_eq!(Cutter::judged(&g, Some(small)), Cutter::flat(&g));
    assert_eq!(Cutter::judged(&g, Some(ball())), ball());
    assert_eq!(Cutter::judged(&g, None), Cutter::flat(&g));
    let w = judge(&pass(0.05), Some(small));
    assert!(gouges(&w).contains(&2), "a mismatched tip judges as the square stack: {w:?}");
    assert_eq!(ball().tip_height(), R);
    assert_eq!(Cutter::flat(&g).tip_height(), 0.0);
}
