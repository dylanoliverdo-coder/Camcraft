//! One trochoidal slot level (issue #220): the bore, then each loop's
//! wall lead and front arc, with the microlift or retract return between
//! loops. Moved out of `trochoid_slot` (#2004, the 400-line law).
//! Provenance: Journeyman spec S7 (`feed_from`,
//! docs/journeyman/specs/S7-descent-feed-height.md; Smid 2007, RS-274/ISO 6983);
//! no vendor parameters, text or code.

use crate::cl::{MoveKind, PathBuilder, lattice};
use crate::feed_from::FeedFrom;
use crate::geo2d::Point2;
use crate::geometry::Point3;

use super::face::GenError;
use super::trochoid_bore::bore;
use super::trochoid_loops::{El, Layout};
use super::trochoid_slot::{TrochoidParams, TrochoidReturn};

/// One level at `z`: the bore from `from` (`trochoid_bore`), then each
/// loop's wall lead and cut, the returns between them. `levels` says
/// whether a retract return may hop (the planner's single level lifts
/// in place instead, which reads the same stock).
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_level(
    b: &mut PathBuilder,
    lay: &Layout,
    p: &TrochoidParams,
    z: f64,
    from: f64,
    levels: bool,
    open: bool,
    retract_feed: f64,
    mat_top: f64,
    feed_from: FeedFrom,
) -> Result<(), GenError> {
    let p3 = |q: Point2, zz: f64| Point3::new(lattice(q.x), lattice(q.y), lattice(zz));
    bore(b, lay, p, z, from, open)?;
    let n = lay.pieces.len();
    for (i, pc) in lay.pieces.iter().enumerate() {
        b.push(MoveKind::Feed, p3(pc.start, z), Some(p.feed));
        for el in &pc.cut {
            let El::Arc { center, to, ccw } = *el;
            b.push_arc(MoveKind::Feed, p3(to, z), Some(p.feed), center, ccw);
        }
        if i + 1 == n {
            break;
        }
        let e = pc.end();
        match p.ret {
            TrochoidReturn::Microlift { height } => {
                let zl = z + height;
                b.push(MoveKind::Feed, p3(e, zl), Some(p.return_feed));
                for el in pc.retrace() {
                    let El::Arc { center, to, ccw } = el;
                    b.push_arc(MoveKind::Feed, p3(to, zl), Some(p.return_feed), center, ccw);
                }
                b.push(MoveKind::Feed, p3(pc.start, z), Some(p.plunge_feed));
            }
            TrochoidReturn::Retract => {
                let hop = if levels { p.retract_z.unwrap_or(p.clearance_z) } else { z + 0.5 };
                let near = (z + crate::approach::DEFAULT_PLUNGE_HEIGHT).min(hop);
                crate::fed_retract::lift_up(b, hop, near, retract_feed);
                b.push(MoveKind::Rapid, p3(pc.start, hop), None);
                if feed_from.is_auto() {
                    b.push(MoveKind::Rapid, p3(pc.start, near), None);
                } else {
                    // `pc.start` was already cleared down to `z` on this level.
                    let xy = Point2::new(lattice(pc.start.x), lattice(pc.start.y));
                    crate::feed_from::push_stated_rapid(b, xy, lattice(z), mat_top, feed_from, true);
                }
                b.push(MoveKind::Feed, p3(pc.start, z), Some(p.plunge_feed));
            }
        }
    }
    Ok(())
}
