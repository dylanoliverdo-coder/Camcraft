//! The operation link trailer's approach digest fields.

use std::hash::{DefaultHasher, Hasher};

use super::{Lead, w};
use crate::links::Approach;

/// Hash the link approach only when its fields differ from the defaults.
pub(super) fn write(h: &mut DefaultHasher, a: Approach) {
    if !a.is_default() {
        h.write_u8(0xC3);
        for v in [
            a.ramp_angle, a.ramp_max_len, a.helix_min_radius, a.helix_pitch, a.helix_angle,
            a.arc_radius, a.arc_sweep, a.plunge_height, a.ramp_feed, a.plunge_feed,
            a.lead_feed_pct, a.max_engagement, a.ext_in, a.ext_out, a.lead_height,
            a.overlap, a.axis_tol,
        ] {
            w(h, v);
        }
        for f in [a.ramp_to_depth, a.helix_cw, a.axis_start] {
            h.write_u8(u8::from(f));
        }
        // M11.3: the horizontal arc entry - a byte only when on,
        // so every earlier approach keeps its digest.
        if a.arc_xy {
            h.write_u8(0xC4);
        }
        // #1449: pre-drilled entry - a byte only when on.
        if a.predrilled {
            h.write_u8(0xC7);
        }
        // #3745: the approach distance outside the stock - bytes
        // only when it differs from the default, so every earlier
        // approach keeps its digest.
        if a.approach_distance.to_bits() != crate::links::Approach::DEFAULT.approach_distance.to_bits() {
            h.write_u8(0xCA);
            w(h, a.approach_distance);
        }
        // M13.11: control compensation moves what the post
        // writes - a byte only when on, so every earlier
        // approach keeps its digest.
        if a.compensation == crate::links::Compensation::Control {
            h.write_u8(0xC5);
        }
        // #38: the wear variant posts the centre path under
        // G41/G42 - its own byte, again only when on.
        if a.compensation == crate::links::Compensation::Wear {
            h.write_u8(0xC6);
        }
        // #1448: the user's start point - bytes only when set.
        if let Some(q) = a.start_point {
            h.write_u8(0xC7);
            w(h, q.x);
            w(h, q.y);
        }
        // #1473: the exit point - bytes only when set.
        if let Some(q) = a.exit_point {
            h.write_u8(0xC8);
            w(h, q.x);
            w(h, q.y);
        }
        // #2012: the equal-steps ladder moves every level - a
        // byte only when on, so every earlier approach keeps
        // its digest.
        if a.equal_steps {
            h.write_u8(0xC9);
        }
        // Journeyman spec S3: fed retract rate, written only when positive.
        if a.retract_feed != 0.0 {
            h.write_u8(0xCB);
            w(h, a.retract_feed);
        }
        // Journeyman spec S8: finishing depth levels, written only when non-default.
        if a.finish_levels > 0 || a.finish_stepdown != 0.0 {
            h.write_u8(0xCC);
            h.write_u32(a.finish_levels);
            w(h, a.finish_stepdown);
        }
        match a.feed_from {
            crate::links::FeedFrom::Auto => {}
            crate::links::FeedFrom::Material { gap } => { h.write_u8(0xCD); w(h, gap); }
            crate::links::FeedFrom::Absolute { z } => { h.write_u8(0xCE); w(h, z); }
        }
        match a.lead_out {
            None => h.write_u8(0xFF),
            Some(Lead::None) => h.write_u8(0),
            Some(Lead::Linear { length }) => {
                h.write_u8(1);
                w(h, length);
            }
            Some(Lead::Arc { radius }) => {
                h.write_u8(2);
                w(h, radius);
            }
            Some(Lead::Tangential { radius }) => {
                h.write_u8(3);
                w(h, radius);
            }
            Some(Lead::Perpendicular { length }) => {
                h.write_u8(4);
                w(h, length);
            }
        }
    }
}
