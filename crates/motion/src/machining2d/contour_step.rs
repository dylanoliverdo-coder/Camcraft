//! A contour level's first engagement (moved out of `contour.rs` by
//! M11.2 C): the level link's lift - clearance or the retract plane -
//! and the descent by the entry (first engagement) or the user's
//! transition (every later level): the bore side's certified arms, the
//! tabbed bore's axial dive, the wall side's graze.
//!
//! Provenance: standard multi-depth profile entries of published CAM
//! practice (lead-in from air, plunge/ramp/helix where bounded air
//! certifies them); the certification laws are `levels.rs`'s (M8.1/M8.2).
//! No vendor material consulted.

use crate::cl::{MoveKind, PathBuilder};
use crate::entry::Entry3D;
use crate::geo2d::Point2;
use crate::geometry::Point3;
use crate::links::Link;

use super::arcs::Seg;
use crate::approach::CENTER_CUT_REFUSAL;

use super::contour_bore::{BoreStep, bore_entry};
use super::contour_entry::Engage;
use super::contour_params::Lead;
use super::descent::feed_down;

/// What one level's engagement reads beyond the shared [`Engage`].
pub(super) struct Step {
    /// The motion: the entry on the path's first engagement, the user's
    /// transition after it.
    pub arm: Entry3D,
    pub e2: Point2,
    pub z: f64,
    /// The plane the descent starts from (clearance on the first level).
    pub prev_z: f64,
    pub first: bool,
    /// Did an earlier descent of the same arm open this descent's column
    /// down to `prev_z` (#2004)? The bore side's rings do not clear the
    /// core, so the first transition of a new arm meets material from the
    /// top of the cut down to `prev_z` there.
    pub opened: bool,
    pub inside: bool,
    pub tabbed: bool,
    /// Where an axial dive at the lead-in's start stops: the cut plane,
    /// or the vertical lead-in's height over it (M11.3 C) - the lead then
    /// descends along itself.
    pub lead_top: f64,
    /// Material top of the level being left (Journeyman spec S3).
    pub lift_mat_top: f64,
}

/// Structural refusals, each said once per path.
#[derive(Default)]
pub(super) struct Said {
    refused: bool,
    tabbed: bool,
}

/// The arm's name in a note.
fn word(arm: Entry3D) -> &'static str {
    match arm {
        Entry3D::Ramp => "ramp",
        Entry3D::Helix => "helix",
        Entry3D::Arc => "arc",
        Entry3D::Plunge => "plunge",
    }
}

/// Lift by the level link (never on the first entry - the tool already
/// stands at clearance) and descend onto `(e2, z)`.
pub(super) fn engage_level(b: &mut PathBuilder, eng: &Engage, s: &Step, lead: &[Seg], walk: &[Seg], said: &mut Said) -> Option<Point2> {
    let p = eng.p;
    // The level link's plane. Keep down reaches here only where the
    // stay-down descent could not be planned (a new ring, a tab, a ring
    // too short): retract and re-enter, as before the option.
    let tz = match (s.first, p.links.level) {
        (true, _) => p.clearance_z,
        (false, Link::Retract) => eng.m.hop_z,
        (false, _) => p.clearance_z,
    };
    if !s.first {
        crate::fed_retract::lift_approach(b, tz, s.lift_mat_top, eng.m.hop_z, &p.links.approach);
    }
    let (e2, z) = (s.e2, s.z);
    if s.inside && s.tabbed {
        // M8.2 C: the arm stands down where a tab is material. dive_room
        // and chord_clears certify against the eroded region - the FULL-
        // depth air - but a standing tab is material INSIDE that region
        // at this level: a helix orbit or a ramp chord certified here
        // could run straight through the tab it was told to leave.
        // Tabbed levels therefore take the clearance traverse and the
        // axial dive at the lead-in; the tab gets its cleanup at the
        // level above, where it is air.
        if s.arm != Entry3D::Plunge && !said.tabbed {
            b.note(
                "entry: no arm stands where a tab is material no \
                 certification can see - axial dive at the lead-in on \
                 tabbed levels",
            );
            said.tabbed = true;
        }
        let cur = b.at();
        b.push(MoveKind::Rapid, Point3::new(e2.x, e2.y, cur.z), None);
        graze(b, eng, e2, s.lead_top, s.arm);
        None
    } else if s.inside {
        // The bore side: the eroded region states the bounded air
        // exactly, so every arm certifies against it with the pocket
        // ladders' own machinery - the dive stands in the floor's room,
        // never dwells on the finished wall. A transition's helix pitch
        // rides along; the first entry keeps the entry's own law.
        let pitch = if s.first { 0.0 } else { p.links.helix_pitch };
        let bs = BoreStep { arm: s.arm, pitch, e2, z, prev_z: s.prev_z, tz, first: s.first, opened: s.opened };
        bore_entry(b, eng, &bs, lead, walk)
    } else {
        // The wall side (explicit Outside, and every direct Auto): the
        // graze this generator has always posted, and the only one that
        // can be certified - the grown region holds the part's own
        // interior, so a dive_room or a chord on it would certify
        // descents THROUGH the body (M8.1's rule). The other arms are
        // refused by name - once, because the reason is structural.
        if !matches!(s.arm, Entry3D::Plunge | Entry3D::Arc) && !said.refused {
            b.note(format!(
                "entry: the {} needs bounded air only a bore (Side::Inside) \
                 has - grazing dive at the lead-in, unchanged",
                word(s.arm)
            ));
            said.refused = true;
        }
        let cur = b.at();
        b.push(MoveKind::Rapid, Point3::new(e2.x, e2.y, cur.z), None);
        graze(b, eng, e2, s.lead_top, s.arm);
        None
    }
}

/// The axial dive at the lead-in's start (tabbed bore levels, the wall
/// side): rapid to the plunge height over the top of the cut - air by the
/// generator's contract, nothing is engaged above it (M11.3) - then feed.
/// A tool that is not centre-cutting may not plunge here (the stock
/// beside the wall is not proved cleared): the operation is refused by
/// name.
fn graze(b: &mut PathBuilder, eng: &Engage, e2: Point2, z: f64, arm: Entry3D) {
    let p = eng.p;
    let a = &p.links.approach;
    if !a.center_cutting {
        b.note_once(&format!(
            "{CENTER_CUT_REFUSAL} - the dive at the lead-in ({:.3}, {:.3}) is into material no helix or ramp can be proved beside; {}",
            e2.x,
            e2.y,
            graze_hint(arm, p.lead == Lead::None)
        ));
    }
    feed_down(b, e2, z, p.entry_top(), eng.m, a.plunge(p.plunge_feed));
}

/// Material top that governed the descent into a contour level (Journeyman S3).
#[allow(clippy::too_many_arguments)]
pub(super) fn level_mat_top(
    p: &super::contour_params::ContourParams,
    inside: bool,
    tabbed: bool,
    rzi: usize,
    rz: &[f64],
    first: bool,
    arm: Entry3D,
    hop_z: f64,
) -> f64 {
    if !inside || tabbed || first || rzi == 0 {
        return p.entry_top();
    }
    let prev_z = rz[rzi - 1];
    if prev_z >= p.clearance_z {
        return p.entry_top().min(prev_z);
    }
    let tz = if p.links.level == Link::Retract { hop_z } else { p.clearance_z };
    let opened = rzi >= 2 || (rzi == 1 && arm == p.entry);
    if !opened && tz > prev_z { p.entry_top().max(prev_z) } else { prev_z }
}

/// Lift off a contour cut to `target_z`: when `retract_feed > 0` feeds to switch
/// height then rapids (Journeyman S3); when `0` keeps the pre-S3 feed-to-hop lift.
pub(super) fn lift_over(
    b: &mut PathBuilder,
    p: &super::contour_params::ContourParams,
    target_z: f64,
    mat_top: f64,
    hop_z: f64,
) {
    if p.links.approach.retract_feed > 0.0 {
        crate::fed_retract::lift_approach(b, target_z, mat_top, hop_z, &p.links.approach);
        return;
    }
    let cur = b.at();
    b.push(MoveKind::Feed, Point3::new(cur.x, cur.y, hop_z), Some(p.feed));
    if target_z > hop_z {
        b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, target_z), None);
    }
}

/// What the user can change to lift the refusal. The Arc entry is already
/// chosen when it reaches this dive, so it never says "pick the Arc entry"
/// (#3247): with no lead-in the arc stands down, so the lead-in is the
/// thing to set.
fn graze_hint(arm: Entry3D, no_lead_in: bool) -> &'static str {
    match (arm, no_lead_in) {
        (Entry3D::Arc, true) => {
            "the Arc entry does not run when the lead-in is None; set a Linear or Arc lead-in, or use a centre-cutting tool"
        }
        (Entry3D::Arc, false) => "the Arc entry found no room here (see the entry note); use a centre-cutting tool",
        _ => "pick the Arc entry or a centre-cutting tool",
    }
}
