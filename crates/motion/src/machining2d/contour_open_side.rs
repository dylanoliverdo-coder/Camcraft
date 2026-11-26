//! Contour along an open chain with the cutter offset to a chosen side (S9).
//!
//! Provenance: Journeyman spec S9 (docs/journeyman/specs/S9-open-sketch-chain-side.md);
//! RS-274/NGC cutter radius compensation (NIST IR 6556, Kramer, Proctor, Messina 2000) and
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007; no vendor
//! parameters, text or code.

use crate::cl::Toolpath;
use crate::geo2d::Point2;

use super::contour::ContourParams;
use super::contour_chain::contour_along_chain;
use super::contour_run_side::{TOL, offset_run_side, simple};
use super::face::GenError;

fn dist(a: Point2, b: Point2) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn cross(o: Point2, a: Point2, b: Point2) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

fn foot_dist(p: Point2, a: Point2, b: Point2) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let l2 = dx * dx + dy * dy;
    if l2 <= 0.0 {
        return dist(p, a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0);
    dist(p, Point2::new(a.x + dx * t, a.y + dy * t))
}

fn seg_seg_dist(a: Point2, b: Point2, c: Point2, d: Point2) -> f64 {
    let (d1, d2) = (cross(a, b, c), cross(a, b, d));
    let (d3, d4) = (cross(c, d, a), cross(c, d, b));
    if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
        return 0.0;
    }
    foot_dist(a, c, d)
        .min(foot_dist(b, c, d))
        .min(foot_dist(c, a, b))
        .min(foot_dist(d, a, b))
}

/// True when any two non-adjacent segments cross or touch, or two adjacent
/// segments fold back 180 degrees onto each other.
fn crosses_itself(p: &[Point2]) -> bool {
    let n = p.len();
    for i in 0..n.saturating_sub(2) {
        let (ax, ay) = (p[i + 1].x - p[i].x, p[i + 1].y - p[i].y);
        let (bx, by) = (p[i + 2].x - p[i + 1].x, p[i + 2].y - p[i + 1].y);
        let la = ax.hypot(ay);
        let lb = bx.hypot(by);
        if la > 0.0 && lb > 0.0 && (ax * bx + ay * by) / (la * lb) <= -1.0 + 1e-6 {
            return true;
        }
    }
    for i in 0..n.saturating_sub(1) {
        for j in (i + 2)..n.saturating_sub(1) {
            if seg_seg_dist(p[i], p[i + 1], p[j], p[j + 1]) <= TOL {
                return true;
            }
        }
    }
    false
}

/// Contour along the directed open chain `pts`, placing the tool centre at
/// `d = tool_radius + stock_to_leave` on the `left` (true) or right (false)
/// side of `pts`. With `p.climb` on, travel runs with the material (the side
/// opposite the tool) on the right under M3; `p.climb == false` reverses it.
/// Wall passes (`p.passes`), levels, leads, and links follow `ContourParams`.
pub fn contour_open_side(pts: &[Point2], left: bool, p: &ContourParams) -> Result<Toolpath, GenError> {
    p.validate()?;
    let chain = simple(pts)?;
    if crosses_itself(&chain) {
        return Err(GenError::Invalid(
            "the open chain crosses or touches itself - pick a simple open chain".into(),
        ));
    }
    let d = p.tool_diameter * 0.5 + p.stock_to_leave;
    if !(d.is_finite() && d > TOL) {
        return Err(GenError::Invalid(format!(
            "the open chain's offset (tool radius plus stock to leave) is {d:.3} mm - it must be positive"
        )));
    }
    // `offset_run_side` returns the chosen side along the CCW tube boundary,
    // which keeps the chain (the material side) on its LEFT. Reversing it puts
    // the material on the RIGHT of travel (climb under M3): with the chain when
    // the tool is on the left, against the chain when the tool is on the right.
    let ride = |off: f64| -> Result<Option<Vec<Point2>>, GenError> {
        let Some(mut r) = offset_run_side(&chain, off, left)? else {
            return Ok(None);
        };
        r.reverse();
        Ok(Some(r))
    };
    let mut path = if p.passes.is_default() {
        let r = ride(d)?.ok_or_else(|| {
            GenError::Invalid(format!(
                "the open chain's offset of {d:.3} mm folds onto itself or breaks into pieces - \
                 pick a gentler chain or a smaller tool / offset"
            ))
        })?;
        contour_along_chain(&r, p)?
    } else {
        super::contour_passes::open_pass_runs(p, ride)?
    };
    let side_word = if left { "left" } else { "right" };
    path.notes.push(format!(
        "contour: open chain offset {d:.3} mm to its {side_word} side (tool centre)"
    ));
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cl::MoveKind;
    use crate::machining2d::{Lead, Side, WallPasses};

    fn base_params() -> ContourParams {
        ContourParams {
            tool_diameter: 10.0,
            stock_to_leave: 0.0,
            clearance_z: 10.0,
            retract_z: 5.0,
            top_z: 0.0,
            bottom_z: -2.0,
            stepdown: 2.0,
            feed: 500.0,
            plunge_feed: 100.0,
            climb: true,
            side: Side::Auto,
            lead: Lead::None,
            tolerance: 0.005,
            tab_count: 0,
            tab_width: 0.0,
            tab_height: 0.0,
            entry: crate::entry::Entry3D::Plunge,
            helix_radius: 1.0,
            blend_turns: false,
            blend_radius: None,
            links: crate::links::Links::PLANAR,
            passes: WallPasses::default(),
        }
    }

    fn cut_moves(path: &Toolpath) -> Vec<&crate::cl::Move> {
        path.moves.iter().filter(|m| m.kind == MoveKind::Feed).collect()
    }

    #[test]
    fn check2_straight_left_climb_runs_plus_x_at_plus_radius() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
        let path = contour_open_side(&chain, true, &base_params()).expect("left");
        let feeds = cut_moves(&path);
        assert_eq!(feeds.len(), 1);
        assert!((feeds[0].from.x - 0.0).abs() < 1e-3 && (feeds[0].from.y - 5.0).abs() < 1e-3);
        assert!((feeds[0].to.x - 50.0).abs() < 1e-3 && (feeds[0].to.y - 5.0).abs() < 1e-3);
    }

    #[test]
    fn check3_straight_right_climb_runs_minus_x_at_minus_radius() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
        let path = contour_open_side(&chain, false, &base_params()).expect("right");
        let feeds = cut_moves(&path);
        assert_eq!(feeds.len(), 1);
        assert!((feeds[0].from.x - 50.0).abs() < 1e-3 && (feeds[0].from.y + 5.0).abs() < 1e-3);
        assert!((feeds[0].to.x - 0.0).abs() < 1e-3 && (feeds[0].to.y + 5.0).abs() < 1e-3);
    }

    #[test]
    fn check4_reversed_chain_left_runs_minus_x_at_minus_radius() {
        let chain = [Point2::new(50.0, 0.0), Point2::new(0.0, 0.0)];
        let path = contour_open_side(&chain, true, &base_params()).expect("reversed left");
        let feeds = cut_moves(&path);
        assert_eq!(feeds.len(), 1);
        assert!((feeds[0].from.x - 50.0).abs() < 1e-3 && (feeds[0].from.y + 5.0).abs() < 1e-3);
        assert!((feeds[0].to.x - 0.0).abs() < 1e-3 && (feeds[0].to.y + 5.0).abs() < 1e-3);
    }

    #[test]
    fn check5_stock_to_leave_shifts_offset() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
        let mut p = base_params();
        p.stock_to_leave = 0.5;
        let path = contour_open_side(&chain, true, &p).expect("left + stock");
        let feeds = cut_moves(&path);
        assert!(feeds.iter().all(|m| (m.from.y - 5.5).abs() < 1e-3 && (m.to.y - 5.5).abs() < 1e-3));
    }

    #[test]
    fn check6_l_chain_concave_left_meets_at_single_point() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(40.0, 0.0), Point2::new(40.0, 30.0)];
        let path = contour_open_side(&chain, true, &base_params()).expect("concave L");
        let feeds = cut_moves(&path);
        assert_eq!(feeds.len(), 2, "two straight moves meeting at (35,5): {feeds:?}");
        assert!((feeds[0].from.x - 0.0).abs() < 1e-3 && (feeds[0].from.y - 5.0).abs() < 1e-3);
        assert!((feeds[0].to.x - 35.0).abs() < 1e-3 && (feeds[0].to.y - 5.0).abs() < 1e-3);
        assert!((feeds[1].from.x - 35.0).abs() < 1e-3 && (feeds[1].from.y - 5.0).abs() < 1e-3);
        assert!((feeds[1].to.x - 35.0).abs() < 1e-3 && (feeds[1].to.y - 30.0).abs() < 1e-3);
    }

    #[test]
    fn check7_l_chain_convex_right_rounds_corner_and_travels_back() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(40.0, 0.0), Point2::new(40.0, 30.0)];
        let path = contour_open_side(&chain, false, &base_params()).expect("convex L");
        let feeds = cut_moves(&path);
        assert!(feeds.len() >= 3, "expected vertical + arc + horizontal: {feeds:?}");
        let first = feeds[0];
        let last = feeds[feeds.len() - 1];
        assert!((first.from.x - 45.0).abs() < 1e-3 && (first.from.y - 30.0).abs() < 1e-3, "{first:?}");
        assert!((last.to.x - 0.0).abs() < 1e-3 && (last.to.y + 5.0).abs() < 1e-3, "{last:?}");
        let corner_arc = feeds.iter().filter_map(|m| m.arc.map(|a| (m, a))).find(|(m, a)| {
            let (r, _) = a.radius_sweep(m.from, m.to);
            (a.center.x - 40.0).abs() < 0.02 && (a.center.y - 0.0).abs() < 0.02 && (r - 5.0).abs() < 0.02
        });
        assert!(corner_arc.is_some(), "expected R=5 arc about (40,0): {feeds:?}");
    }

    #[test]
    fn check8_narrow_u_chain_inside_folds_and_is_refused() {
        let u = [
            Point2::new(0.0, 30.0),
            Point2::new(0.0, 0.0),
            Point2::new(8.0, 0.0),
            Point2::new(8.0, 30.0),
        ];
        let err = contour_open_side(&u, true, &base_params()).expect_err("narrow U folds");
        let msg = err.to_string();
        assert!(msg.contains("5.000") && msg.contains("folds"), "{msg}");
    }

    #[test]
    fn check9_self_crossing_chain_is_refused() {
        // Four segments passing through (10, 10).
        let x = [
            Point2::new(0.0, 0.0),
            Point2::new(20.0, 20.0),
            Point2::new(20.0, 0.0),
            Point2::new(0.0, 20.0),
            Point2::new(0.0, 10.0),
        ];
        let err = contour_open_side(&x, true, &base_params()).expect_err("crossing chain");
        assert!(err.to_string().contains("crosses or touches itself"), "{err}");
    }

    #[test]
    fn conventional_milling_reverses_travel_direction() {
        let chain = [Point2::new(0.0, 0.0), Point2::new(50.0, 0.0)];
        let mut p = base_params();
        p.climb = false;
        let path = contour_open_side(&chain, true, &p).expect("conventional left");
        let feeds = cut_moves(&path);
        assert_eq!(feeds.len(), 1);
        assert!((feeds[0].from.x - 50.0).abs() < 1e-3 && (feeds[0].from.y - 5.0).abs() < 1e-3);
        assert!((feeds[0].to.x - 0.0).abs() < 1e-3 && (feeds[0].to.y - 5.0).abs() < 1e-3);
    }
}
