//! Stated rapid-to-feed switch height for 2D plunge descents (Journeyman spec S7).
//!
//! Provenance: Journeyman spec S7 (docs/journeyman/specs/S7-descent-feed-height.md);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007 (rapid approach
//! to a clearance height above the work, feed from there); RS-274 / ISO 6983 (G0/G1);
//! no vendor parameters, text or code.

use crate::cl::{Move, MoveKind, PathBuilder};
use crate::geo2d::Point2;
use crate::geometry::Point3;

/// Where a 2D plunge descent switches from rapid (G0) to plunge feed (G1)
/// (Journeyman spec S7).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum FeedFrom {
    /// Existing laws unchanged: closed ladders rapid to
    /// `min(mat_top + plunge_height, max(hop_z, mat_top))`; open runs feed the
    /// whole way from the traverse height.
    #[default]
    Auto,
    /// Rapid to `min(mat_top + gap, z_t)` and feed from there; `gap >= 0`
    /// (`gap == 0` switches at `mat_top`).
    Material { gap: f64 },
    /// Rapid to `min(z, z_t)` in the operation's frame and feed from there.
    Absolute { z: f64 },
}

/// Note prefix emitted when a stated `feed_from` descent cannot be honoured safely.
pub const FEED_FROM_REFUSAL: &str = "descent feed height refused:";

/// Height slack for Z comparisons (mm).
const Z_EPS: f64 = 1e-6;
/// Rapid-skip slack when `hz` is already at or above the traverse height (mm).
const TRAV_EPS: f64 = 1e-9;
/// Concentric XY tolerance (mm) for proving an earlier cutting move swept the
/// full cutter disc over `xy` down to `z`.
const FULL_DISC_EPS: f64 = 1e-4;
/// Chord tolerance for sampling arcs/helices during cleared-column proof (mm).
const CHORD_TOL: f64 = 0.01;

impl FeedFrom {
    /// Door sanity at command / operation creation (Journeyman spec S7).
    #[must_use]
    pub fn refusal(self) -> Option<&'static str> {
        match self {
            FeedFrom::Auto => None,
            FeedFrom::Material { gap } => {
                if !(gap.is_finite() && gap >= 0.0) {
                    Some("feed_from material gap must be finite and zero or positive (mm)")
                } else {
                    None
                }
            }
            FeedFrom::Absolute { z } => {
                if !z.is_finite() {
                    Some("feed_from absolute Z must be finite (mm)")
                } else {
                    None
                }
            }
        }
    }

    /// True when this is the default [`FeedFrom::Auto`] mode.
    #[must_use]
    pub fn is_auto(self) -> bool {
        matches!(self, FeedFrom::Auto)
    }
}

/// Distance from `p` to the 2D segment `a -> b`.
fn seg_dist(p: Point2, a: Point2, b: Point2) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let dd = dx * dx + dy * dy;
    let t = if dd > 0.0 && dd.is_finite() {
        (((p.x - a.x) * dx + (p.y - a.y) * dy) / dd).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.x - a.x - t * dx).hypot(p.y - a.y - t * dy)
}

/// Has an earlier non-rapid move in `moves` swept the full cutter disc at `xy`
/// down to `z`? Stricter than `plunge_law::column_cleared` (which allows a
/// feed plunge if only the inner core is clear): a rapid (G0) requires the
/// entire cutter diameter to be clear of solid.
#[must_use]
pub fn full_column_cleared(moves: &[Move], xy: Point2, z: f64) -> bool {
    if !z.is_finite() {
        return false;
    }
    moves.iter().rev().filter(|m| m.kind != MoveKind::Rapid).any(|m| {
        if m.from.z.min(m.to.z) > z + Z_EPS {
            return false;
        }
        m.segments3(CHORD_TOL).iter().any(|(a, b)| {
            a.z.max(b.z) <= z + Z_EPS
                && seg_dist(xy, Point2::new(a.x, a.y), Point2::new(b.x, b.y)) <= FULL_DISC_EPS
        })
    })
}

/// Emit the rapid approach (if any) for a stated [`FeedFrom`] (`Material` or
/// `Absolute`) before a plunge descent from the tool's current traverse height
/// `z_t = b.at().z` down to `z_cut` over `xy`.
///
/// Refuses by recording a [`FEED_FROM_REFUSAL`] note on `b` (which every 2D
/// generator converts into `GenError::Invalid` via `approach::refusal`) when:
/// - `hz <= z_cut` (the rapid would reach or pass the cut level), naming `(x, y)`
///   and both heights; or
/// - `Absolute` would rapid below `mat_top` over a column not proved cleared
///   (neither `proved_clear` nor cleared by an earlier move in `b`), naming
///   `(x, y)` and both heights.
///
/// Does nothing when `feed_from` is [`FeedFrom::Auto`] or when
/// `z_t <= z_cut + TRAV_EPS` (zero-drop stay-down).
pub fn push_stated_rapid(
    b: &mut PathBuilder,
    xy: Point2,
    z_cut: f64,
    mat_top: f64,
    feed_from: FeedFrom,
    proved_clear: bool,
) {
    let z_t = b.at().z;
    let hz = match feed_from {
        FeedFrom::Auto => return,
        FeedFrom::Material { gap } => (mat_top + gap).min(z_t),
        FeedFrom::Absolute { z } => z.min(z_t),
    };
    if z_t <= z_cut + TRAV_EPS {
        return;
    }
    if !hz.is_finite() || hz <= z_cut + TRAV_EPS {
        b.note_once(&format!(
            "{FEED_FROM_REFUSAL} rapid switch height Z {hz:.3} mm is at or below cut level Z {z_cut:.3} mm at ({:.3}, {:.3}) - raise feed_from above Z {z_cut:.3} mm",
            xy.x, xy.y
        ));
        return;
    }
    if matches!(feed_from, FeedFrom::Absolute { .. })
        && hz < z_t - TRAV_EPS
        && hz < mat_top - Z_EPS
        && !proved_clear
        && !full_column_cleared(b.moves(), xy, hz)
    {
        b.note_once(&format!(
            "{FEED_FROM_REFUSAL} rapid height Z {hz:.3} mm is below material top Z {mat_top:.3} mm at ({:.3}, {:.3}) over an uncleared column - raise feed_from to at least Z {mat_top:.3} mm",
            xy.x, xy.y
        ));
        return;
    }
    if hz < z_t - TRAV_EPS {
        b.push(MoveKind::Rapid, Point3::new(xy.x, xy.y, hz), None);
    }
}
