//! Fed vertical retract helper for 2D milling operations.
//!
//! Provenance: Journeyman spec S3 (2D operations, fed retract; RS-274 /
//! ISO 6983 G1/G0 vertical lift); no vendor parameters, text or code.

use crate::approach::Approach;
use crate::cl::{MoveKind, PathBuilder};
use crate::geometry::Point3;

/// Refusal message when `retract_feed` is negative, non-finite, or absurd.
pub const RETRACT_FEED_REFUSAL: &str =
    "retract_feed must be zero (rapid) or a finite positive feed rate in mm/min (at most 1e6)";

/// Refuse an invalid `retract_feed` by name; `None` means valid.
#[must_use]
pub fn retract_feed_refusal(v: f64) -> Option<&'static str> {
    (!(v.is_finite() && (0.0..=1.0e6).contains(&v))).then_some(RETRACT_FEED_REFUSAL)
}

/// Switch height `hz` where a level's descent switches from rapid to feed
/// (and where a fed retract switches from feed back to rapid):
/// `hz = min(mat_top + plunge_height, max(hop_z, mat_top))`.
#[must_use]
pub fn switch_z(mat_top: f64, plunge_height: f64, hop_z: f64) -> f64 {
    let h = if plunge_height.is_finite() && plunge_height > 0.0 { plunge_height } else { 0.0 };
    (mat_top + h).min(hop_z.max(mat_top))
}

/// Straight-up vertical lift from the builder's current position to `target_z`.
///
/// - When `retract_feed <= 0.0` (the default): identical to a single vertical
///   `MoveKind::Rapid` to `target_z`.
/// - When `retract_feed > 0.0` and `target_z > cur.z`:
///   - If `hz > cur.z`: feeds straight up at `retract_feed` to `min(hz, target_z)`,
///     then rapids straight up to `target_z` if `target_z > hz`.
///   - Otherwise (`cur.z >= hz`): rapids straight up to `target_z`.
/// - Never emits a downward or diagonal feed.
pub fn lift_up(b: &mut PathBuilder, target_z: f64, hz: f64, retract_feed: f64) {
    let cur = b.at();
    if !(retract_feed.is_finite() && retract_feed > 0.0) {
        b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, target_z), None);
        return;
    }
    const EPS: f64 = 1e-9;
    if !(target_z.is_finite() && cur.z.is_finite() && target_z > cur.z + EPS) {
        b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, target_z), None);
        return;
    }
    if hz.is_finite() && hz > cur.z + EPS {
        let feed_to = hz.min(target_z);
        b.push(MoveKind::Feed, Point3::new(cur.x, cur.y, feed_to), Some(retract_feed));
        if target_z > feed_to + EPS {
            b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, target_z), None);
        }
    } else {
        b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, target_z), None);
    }
}

/// Straight-up vertical lift using explicit `mat_top`, `plunge_height`, `hop_z`,
/// and `retract_feed`.
pub fn lift_motion(
    b: &mut PathBuilder,
    target_z: f64,
    mat_top: f64,
    plunge_height: f64,
    hop_z: f64,
    retract_feed: f64,
) {
    let hz = switch_z(mat_top, plunge_height, hop_z);
    lift_up(b, target_z, hz, retract_feed);
}

/// Straight-up vertical lift reading `plunge_height` and `retract_feed` from
/// `Approach`.
pub fn lift_approach(b: &mut PathBuilder, target_z: f64, mat_top: f64, hop_z: f64, a: &Approach) {
    lift_motion(b, target_z, mat_top, a.plunge_height, hop_z, a.retract_feed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_retract_feed_is_single_rapid() {
        let mut b = PathBuilder::new(Point3::new(10.0, 20.0, -2.5));
        lift_motion(&mut b, 5.0, 0.0, 1.0, 5.0, 0.0);
        assert_eq!(b.moves().len(), 1);
        assert_eq!(b.moves()[0].kind, MoveKind::Rapid);
        assert_eq!(b.moves()[0].to, Point3::new(10.0, 20.0, 5.0));
    }

    #[test]
    fn positive_retract_feed_splits_at_switch_height() {
        let mut b = PathBuilder::new(Point3::new(10.0, 20.0, -2.5));
        lift_motion(&mut b, 5.0, 0.0, 1.0, 5.0, 2540.0);
        assert_eq!(b.moves().len(), 2);
        assert_eq!(b.moves()[0].kind, MoveKind::Feed);
        assert_eq!(b.moves()[0].feed, Some(2540.0));
        assert_eq!(b.moves()[0].to, Point3::new(10.0, 20.0, 1.0));
        assert_eq!(b.moves()[1].kind, MoveKind::Rapid);
        assert_eq!(b.moves()[1].to, Point3::new(10.0, 20.0, 5.0));
    }

    #[test]
    fn target_at_or_below_switch_height_feeds_straight_to_target() {
        let mut b = PathBuilder::new(Point3::new(1.0, 2.0, -5.0));
        lift_up(&mut b, -2.0, -1.5, 2540.0);
        assert_eq!(b.moves().len(), 1);
        assert_eq!(b.moves()[0].kind, MoveKind::Feed);
        assert_eq!(b.moves()[0].feed, Some(2540.0));
        assert_eq!(b.moves()[0].to, Point3::new(1.0, 2.0, -2.0));
    }

    #[test]
    fn never_emits_downward_feed() {
        let mut b = PathBuilder::new(Point3::new(3.0, 4.0, 10.0));
        lift_up(&mut b, 5.0, 8.0, 2540.0);
        assert_eq!(b.moves().len(), 1);
        assert_eq!(b.moves()[0].kind, MoveKind::Rapid);
        assert_eq!(b.moves()[0].to, Point3::new(3.0, 4.0, 5.0));
    }

    #[test]
    fn refuses_negative_and_non_finite_by_name() {
        assert!(retract_feed_refusal(0.0).is_none());
        assert!(retract_feed_refusal(2540.0).is_none());
        for bad in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 2.0e6] {
            let msg = retract_feed_refusal(bad).expect("invalid retract_feed must refuse");
            assert!(msg.contains("retract_feed"), "refusal must name retract_feed: {msg}");
        }
    }
}
