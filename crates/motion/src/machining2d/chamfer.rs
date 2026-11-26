//! 2D chamfer: a cone-ruled ride along the picked edge (M2.4 B, Journeyman spec S10).
//!
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! the chamfer mill's cut law is machine-tool handbook geometry (*Machinery's Handbook*, 31st ed.,
//! Industrial Press 2020; P. Smid, *CNC Programming Handbook*, 3rd ed., 2007) - a conical cutter
//! whose tangent generatrix lies ON the chamfer face. The face joins the point `width` back from
//! the wall (in the top plane) to the point `width * tan(angle)` down the wall; a cone of
//! half-angle from the axis `90 - angle` degrees, riding the wall with its virtual tip at the
//! corner, has exactly that surface. The included angle across such a tool's cone is
//! `180 - 2 * angle` (a 90-degree chamfer mill for a 45-degree chamfer). Written clean-room from
//! that geometry, no vendor parameters, text or code; the CL convention (moves carry TIP
//! positions) is this codebase's own, so the posted Z is the corner depth.
//!
//! The path is one closed circuit per picked loop at one constant Z -
//! `top_z - width * tan(angle_deg)`, the derived floor the operation
//! states instead of carrying - entered by the axial dive trace and
//! the slot post (a chamfer ride has no bounded air beside the start
//! the entry arms could certify against: the wall is the start).
//! The congruence doors (the tool's cone, its reach) stand in the
//! app's dispatch where the Tool is visible; here the law is stated
//! on the face alone. Prime Directive: refused before cut, never
//! approximated.

use crate::cl::{EdgeBreak, MoveKind, PathBuilder, Toolpath};
use crate::geo2d::{Loop, Point2};
use crate::geometry::Point3;

use super::face::GenError;
use super::levels::{Motion, run_levels};

/// One chamfer ride's numbers. `top_z` is the plane the chamfer's wide
/// end opens in (the picked edge's top); the corner depth follows from
/// it and is never an independent input - that is what makes the
/// operation's floor a law, not a second promise.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChamferParams {
    /// Horizontal extent of the chamfer face in the top plane.
    pub width: f64,
    /// The face's angle from that plane, degrees in (0, 90).
    pub angle_deg: f64,
    pub clearance_z: f64,
    /// Hop plane; validated at or above `top_z`, never the corner -
    /// between the corner and the top the wall still stands.
    pub retract_z: Option<f64>,
    pub top_z: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    /// Climb milling under M3 (#1555): the material stays on the RIGHT
    /// of travel - a boss edge rides clockwise, a bore edge
    /// counter-clockwise; conventional reverses each loop.
    pub climb: bool,
    /// The picked region is a bore's air (Side Inside), not the part's
    /// material: which side of each stored loop the material stands on.
    pub inside: bool,
}

impl ChamferParams {
    pub(super) fn validate(&self) -> Result<(), GenError> {
        let mut finite = [
            self.width,
            self.angle_deg,
            self.clearance_z,
            self.top_z,
            self.feed,
            self.plunge_feed,
        ]
        .into_iter()
        .all(f64::is_finite);
        finite &= self.retract_z.is_none_or(f64::is_finite);
        if !finite {
            return Err(GenError::Invalid("parameters must be finite".into()));
        }
        if self.width <= 0.0 {
            return Err(GenError::Invalid("chamfer width must be positive".into()));
        }
        if !(0.0..90.0).contains(&self.angle_deg) {
            return Err(GenError::Invalid(
                "chamfer angle must be between 0 and 90 degrees".into(),
            ));
        }
        if self.feed <= 0.0 || self.plunge_feed <= 0.0 {
            return Err(GenError::Invalid("feeds must be positive".into()));
        }
        if self.clearance_z < self.top_z {
            return Err(GenError::Invalid("clearance plane below the chamfer's top plane".into()));
        }
        if self.retract_z.is_some_and(|rz| rz < self.top_z) {
            return Err(GenError::Invalid("retract plane sits inside the material".into()));
        }
        Ok(())
    }

    /// The corner depth the ride posts - the derived floor, stated in
    /// one place so the panel, the door and the path cannot disagree.
    #[must_use]
    pub fn corner_z(&self) -> f64 {
        self.top_z - self.width * self.angle_deg.to_radians().tan()
    }
}

/// Chamfer every loop of the picked region: one circuit per loop, in
/// `|area|` order for determinism (trace's ordering law), at the
/// corner depth. The centers ride the loops exactly - the tip is on
/// the wall, the generatrix on the face - so no offset exists to get
/// wrong, and a picked edge that is not the chamfer's own wall is
/// caught by the stock simulation, not papered over here.
pub fn chamfer(region: &crate::geo2d::Region, p: &ChamferParams) -> Result<Toolpath, GenError> {
    p.validate()?;
    if region.is_empty() {
        return Err(GenError::Empty("empty chamfer region".into()));
    }
    let mut loops: Vec<&Loop> = region.loops().iter().collect();
    loops.sort_by(|x, y| y.area().abs().total_cmp(&x.area().abs()));
    let z = p.corner_z();
    if !z.is_finite() {
        return Err(GenError::Invalid("corner depth left the finite horizon".into()));
    }
    let mut passes: Vec<Vec<Point2>> = Vec::with_capacity(loops.len());
    for l in &loops {
        // A stored loop keeps its region on the left: the material for a
        // boss edge (climb reverses it), the bore's air for an Inside
        // pick (climb keeps it) - material right of travel either way.
        let mut pass: Vec<Point2> = super::ring::walked(l.points(), p.climb == p.inside, false);
        if let Some(&start) = pass.first() {
            pass.push(start);
        }
        passes.push(pass);
    }

    let mut b = PathBuilder::approached(Point3::new(0.0, 0.0, p.clearance_z));
    // The one level the ride owns: not a ladder - the cone's
    // generatrix meets the wall at exactly one depth.
    let motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed);
    run_levels(&mut b, &passes, &[z], &motion, None, None, None);
    let cur = b.at();
    b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, p.clearance_z), None);

    let mut path = b.finish();
    if !path.all_finite() {
        return Err(GenError::Invalid("path left the finite horizon".into()));
    }
    // The break this ride was asked to cut (#303): each picked loop at
    // the top plane, the face's depth and slope from the two authored
    // numbers - never read back off the moves, so the sim judges the
    // ride against the promise.
    let slope = p.angle_deg.to_radians().tan();
    path.breaks = passes
        .iter()
        .map(|pass| EdgeBreak {
            edge: pass.iter().map(|q| Point3::new(q.x, q.y, p.top_z)).collect(),
            depth: p.top_z - z,
            slope,
            volume: None,
        })
        .filter(EdgeBreak::is_sane)
        .collect();
    Ok(path)
}

/// One closed sketch chain for [`chamfer_chains`] (Journeyman spec S10):
/// its outline loop, whether the material stands outside it (`inside = true`
/// for a bore edge, `false` for a boss edge), and its climb flag.
#[derive(Clone, Debug)]
pub struct ChamferClosedChain {
    pub loop_: Loop,
    pub inside: bool,
    pub climb: bool,
}

/// Chamfer closed loops (largest `|area|` first, matching [`chamfer`]) and
/// then open chains (in caller order, already oriented in travel order), all
/// at the single constant corner depth `p.corner_z()` with the shared
/// retract/rapid/dive link between rides (Journeyman spec S10).
pub fn chamfer_chains(
    closed: &[ChamferClosedChain],
    open: &[Vec<Point2>],
    p: &ChamferParams,
) -> Result<Toolpath, GenError> {
    p.validate()?;
    if closed.is_empty() && open.is_empty() {
        return Err(GenError::Empty("no chamfer chains to cut".into()));
    }
    let z = p.corner_z();
    if !z.is_finite() {
        return Err(GenError::Invalid("corner depth left the finite horizon".into()));
    }

    let mut ordered_closed: Vec<&ChamferClosedChain> = closed.iter().collect();
    ordered_closed.sort_by(|x, y| y.loop_.area().abs().total_cmp(&x.loop_.area().abs()));

    let mut passes: Vec<Vec<Point2>> = Vec::with_capacity(ordered_closed.len() + open.len());
    for c in ordered_closed {
        // Canonicalize each closed loop through `Region::new` (the same
        // representation `chamfer` reads from a picked solid loop) so it is
        // guaranteed CCW (interior on the left) and matches a picked loop's
        // seam and moves verbatim: boss (`inside == false`) rides CW under
        // climb, bore (`inside == true`) rides CCW under climb (#1555).
        let reg = crate::geo2d::Region::new(vec![c.loop_.clone()])?;
        for l in reg.loops() {
            let mut pass: Vec<Point2> = super::ring::walked(l.points(), c.climb == c.inside, false);
            if let Some(&start) = pass.first() {
                pass.push(start);
            }
            passes.push(pass);
        }
    }

    for chain in open {
        if chain.len() < 2 {
            return Err(GenError::Invalid("an open chamfer chain needs at least two points".into()));
        }
        if !chain.iter().all(|q| q.x.is_finite() && q.y.is_finite()) {
            return Err(GenError::Invalid("an open chamfer chain holds non-finite points".into()));
        }
        passes.push(chain.clone());
    }
    if passes.is_empty() {
        return Err(GenError::Empty("no chamfer chains to cut".into()));
    }

    let mut b = PathBuilder::approached(Point3::new(0.0, 0.0, p.clearance_z));
    let motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed);
    run_levels(&mut b, &passes, &[z], &motion, None, None, None);
    let cur = b.at();
    b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, p.clearance_z), None);

    let mut path = b.finish();
    if !path.all_finite() {
        return Err(GenError::Invalid("path left the finite horizon".into()));
    }
    let slope = p.angle_deg.to_radians().tan();
    path.breaks = passes
        .iter()
        .map(|pass| EdgeBreak {
            edge: pass.iter().map(|q| Point3::new(q.x, q.y, p.top_z)).collect(),
            depth: p.top_z - z,
            slope,
            volume: None,
        })
        .filter(EdgeBreak::is_sane)
        .collect();
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo2d::Region;

    fn base_params(width: f64, angle_deg: f64) -> ChamferParams {
        ChamferParams {
            width,
            angle_deg,
            clearance_z: 10.0,
            retract_z: Some(5.0),
            top_z: 0.0,
            feed: 500.0,
            plunge_feed: 100.0,
            climb: true,
            inside: false,
        }
    }

    #[test]
    fn closed_rectangle_matches_chamfer_region_move_for_move() {
        let rect = Loop::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(40.0, 0.0),
            Point2::new(40.0, 20.0),
            Point2::new(0.0, 20.0),
        ])
        .expect("rect");
        let p = base_params(0.5, 45.0);
        let solid_path = chamfer(&Region::new(vec![rect.clone()]).expect("region"), &p).expect("solid chamfer");
        let chain_path = chamfer_chains(
            &[ChamferClosedChain { loop_: rect, inside: false, climb: true }],
            &[],
            &p,
        )
        .expect("chain chamfer");
        assert_eq!(chain_path.moves, solid_path.moves);
        assert!(chain_path.moves.iter().filter(|m| m.kind == MoveKind::Feed).all(|m| (m.to.z + 0.5).abs() < 1e-9));
    }

    #[test]
    fn angle_30_corner_depth_matches_width_tan_30() {
        let p = base_params(0.5, 30.0);
        let open = vec![vec![Point2::new(0.0, 0.0), Point2::new(40.0, 0.0)]];
        let path = chamfer_chains(&[], &open, &p).expect("open chamfer");
        let z = path.moves.iter().find(|m| m.kind == MoveKind::Feed).expect("feed").to.z;
        assert!((z - (-0.5 * 30f64.to_radians().tan())).abs() < 1e-9);
        assert!((z - (-0.2887)).abs() < 1e-4, "z={z}");
    }

    #[test]
    fn nested_boss_and_bore_travel_cw_and_ccw_largest_first() {
        let outer = Loop::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(40.0, 0.0),
            Point2::new(40.0, 40.0),
            Point2::new(0.0, 40.0),
        ])
        .expect("outer");
        let inner = Loop::new(vec![
            Point2::new(10.0, 10.0),
            Point2::new(30.0, 10.0),
            Point2::new(30.0, 30.0),
            Point2::new(10.0, 30.0),
        ])
        .expect("inner");
        let p = base_params(0.5, 45.0);
        // Pass inner first to verify largest-first ordering.
        let path = chamfer_chains(
            &[
                ChamferClosedChain { loop_: inner, inside: true, climb: true },
                ChamferClosedChain { loop_: outer, inside: false, climb: true },
            ],
            &[],
            &p,
        )
        .expect("nested");
        let feeds: Vec<_> = path.moves.iter().filter(|m| m.kind == MoveKind::Feed).collect();
        assert_eq!(feeds.len(), 8);
        // First 4 moves: outer loop (boss -> CW signed area < 0).
        let signed = |ms: &[&crate::cl::Move]| -> f64 {
            ms.iter().map(|m| m.from.x * m.to.y - m.to.x * m.from.y).sum::<f64>() * 0.5
        };
        assert!(signed(&feeds[..4]) < -1000.0, "outer boss rides CW: {:?}", &feeds[..4]);
        assert!(signed(&feeds[4..]) > 300.0, "inner bore rides CCW: {:?}", &feeds[4..]);
    }

    #[test]
    fn empty_chain_list_is_refused() {
        let p = base_params(0.5, 45.0);
        let err = chamfer_chains(&[], &[], &p).expect_err("empty chains");
        assert!(matches!(err, GenError::Empty(_)), "{err}");
    }

    #[test]
    fn degenerate_collinear_closed_loop_is_refused_as_empty() {
        let p = base_params(0.5, 45.0);
        let degen = Loop::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(20.0, 0.0),
        ])
        .expect("collinear loop");
        let err = chamfer_chains(
            &[ChamferClosedChain { loop_: degen, inside: false, climb: true }],
            &[],
            &p,
        )
        .expect_err("degenerate closed loop collapses to empty");
        assert!(matches!(err, GenError::Empty(_)), "{err}");
    }
}
