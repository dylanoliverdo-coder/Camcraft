//! Wall finishing passes with independent direction for 2D Pocket (Journeyman spec S4):
//! contour-parallel finishing and spring passes round every wall (outer boundary and each
//! island) after clearing, with their own climb/conventional direction and per-level or
//! last-level scheduling.
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991
//! (contour-parallel passes round boundary and islands); P. Smid, *CNC Programming Handbook*,
//! 3rd ed., Industrial Press 2007 (climb and conventional milling); no vendor parameters,
//! text or code.

use crate::cl::{PathBuilder, Toolpath};
use crate::geo2d::{Point2, Region};
use crate::geometry::Point3;

use super::arc_cross::post_crossing_free;
use super::arc_proof::ProofTally;
use super::arcs::Seg;
use super::blend_cross::{Bans, Blend, round_walks};
use super::contour_passes::{FinishLevel, MAX_WALL_PASSES};
use super::corner_relief::{CornerRelief, ResolvedRelief, plan_corner_reliefs};
use super::corner_relief_spur::inject_relief_spurs;
use super::equal_steps::cut_levels_for;
use super::face::GenError;
use super::fillet::Legal;
use super::fillet_tally::corner_arc_radius;
use super::levels::{Cert, Motion, WalkState, walk_one_level};
use super::pass_start::{PassStart, StartSide};
use super::pocket::{PocketParams, non_sliver_center, pocket_started_on};
use super::pocket_finish_rings::{
    fit_clear_pass, fit_finish_ring, plan_clearing, plan_pocket_finish_rings, resolve_strategy,
};
use super::predrilled::PreHole;

/// Upper bound on finite finishing spacing (mm), guarding against absurd inputs.
pub const MAX_FINISH_SPACING: f64 = 10_000.0;

/// Milling direction of a 2D Pocket's wall finishing and spring passes (spec S4).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FinishDirection {
    /// Follow the pocket's `climb` flag (opposite turning senses on outer walls and islands).
    #[default]
    Same,
    /// Climb milling on finishing and spring passes only (independent of clearing `climb`).
    Climb,
    /// Conventional milling on finishing and spring passes only (independent of clearing `climb`).
    Conventional,
}

impl FinishDirection {
    /// Effective `climb` flag for finishing and spring passes.
    #[must_use]
    pub fn resolve_climb(self, clearing_climb: bool) -> bool {
        match self {
            Self::Same => clearing_climb,
            Self::Climb => true,
            Self::Conventional => false,
        }
    }
}

/// Wall finishing controls of a 2D Pocket operation (Journeyman spec S4).
/// The default (`finish_passes = 0`, `spring_passes = 0`, `finish_direction = Same`,
/// `finish_level = Every`) is today's clearing alone, byte for byte.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PocketFinish {
    /// Finishing passes round every wall (outer boundary and each island); `0` is none.
    pub finish_passes: u32,
    /// Distance between two finishing passes (mm); unused while `finish_passes == 0`.
    pub finish_spacing: f64,
    /// Repeats of the final wall pass at `w = r + stock_to_leave`.
    pub spring_passes: u32,
    /// Direction of the finishing and spring passes (`Same` follows `climb`).
    pub finish_direction: FinishDirection,
    /// Levels where finishing and spring passes run (`Every` or `Last`).
    pub finish_level: FinishLevel,
}

impl Default for PocketFinish {
    fn default() -> Self {
        Self {
            finish_passes: 0,
            finish_spacing: 0.0,
            spring_passes: 0,
            finish_direction: FinishDirection::Same,
            finish_level: FinishLevel::Every,
        }
    }
}

impl PocketFinish {
    /// True when every field is at its pre-S4 default (omitted from wire and digest).
    #[must_use]
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }

    /// True when at least one finishing or spring pass will be emitted.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.finish_passes > 0 || self.spring_passes > 0
    }

    /// Named refusal for invalid finishing parameters (spec S4 "Refusals").
    #[must_use]
    pub fn refusal(&self) -> Option<String> {
        if !self.finish_spacing.is_finite() {
            return Some("finishing passes: finish_spacing must be finite".into());
        }
        if self.finish_spacing < 0.0 {
            return Some(format!(
                "finishing passes: finish_spacing must not be negative (set to {:.3} mm)",
                self.finish_spacing
            ));
        }
        if self.finish_spacing > MAX_FINISH_SPACING {
            return Some(format!(
                "finishing passes: finish_spacing ({:.3} mm) exceeds the {:.0} mm limit",
                self.finish_spacing, MAX_FINISH_SPACING
            ));
        }
        if self.finish_passes > 0 && self.finish_spacing <= 0.0 {
            return Some(format!(
                "finishing passes: {} passes need a positive spacing (set to {:.3} mm)",
                self.finish_passes, self.finish_spacing
            ));
        }
        if self.finish_passes > MAX_WALL_PASSES || self.spring_passes > MAX_WALL_PASSES {
            return Some(format!(
                "finishing passes: at most {MAX_WALL_PASSES} finishing and {MAX_WALL_PASSES} spring passes are supported"
            ));
        }
        None
    }

    /// Additional radial wall stock left by clearing (`n_f * d_f`, or `0.0`).
    #[must_use]
    pub(crate) fn clearing_allowance(&self) -> f64 {
        if self.finish_passes > 0 { f64::from(self.finish_passes) * self.finish_spacing } else { 0.0 }
    }

    /// Radial wall allowance left by clearing: `n_f * d_f` when `n_f > 0`, or
    /// a tiny 0.01 mm skim gap when `n_f == 0 && spring_passes > 0` so clearing
    /// passes stay strictly separated from the spring ring at `w`.
    #[must_use]
    pub(crate) fn clearing_gap(&self, p: &PocketParams) -> f64 {
        if !self.is_active() {
            0.0
        } else if self.finish_passes > 0 {
            self.clearing_allowance()
        } else {
            0.01_f64.min(p.tiling_step() * 0.25)
        }
    }
}

/// Flute-length law for `finish_level = Last` (spec S4 / S1): a finishing pass
/// at the last level cuts the whole wall height `top_z - bottom_z` with the side
/// of the tool, so the tool's flute length must cover it.
pub fn pocket_flute_covers_wall(top_z: f64, bottom_z: f64, flute_len: f64) -> Result<(), GenError> {
    let wall = top_z - bottom_z;
    if !(wall.is_finite() && flute_len.is_finite() && flute_len > 0.0) {
        return Err(GenError::Invalid(
            "2D Pocket finishing: the tool states no flute length, so the wall height a last-level finishing pass cuts is unknown".into(),
        ));
    }
    if flute_len + 1e-9 < wall {
        return Err(GenError::Invalid(format!(
            "2D Pocket finishing: the wall is {wall:.3} mm tall but the tool's flute is {flute_len:.3} mm long - \
             a finishing pass at the last level cuts the whole wall with the side of the tool; use a tool \
             with a flute at least {wall:.3} mm long"
        )));
    }
    Ok(())
}

/// Whether cut level index `idx` of `count` levels runs finishing/spring passes.
pub(super) fn level_runs_finishing(finish: &PocketFinish, idx: usize, count: usize) -> bool {
    match finish.finish_level {
        FinishLevel::Every => true,
        FinishLevel::Last => idx + 1 == count,
    }
}

/// 2D Pocket with optional wall finishing and spring passes (spec S4).
/// When `finish` is inactive, delegates byte-for-byte to `pocket_started_on`.
pub fn pocket_with_finish(
    region: &Region,
    p: &PocketParams,
    holes: &[PreHole],
    relief: &CornerRelief,
    side: &StartSide,
    finish: &PocketFinish,
) -> Result<Toolpath, GenError> {
    if let Some(why) = finish.refusal() {
        return Err(GenError::Invalid(why));
    }
    if !finish.is_active() {
        return pocket_started_on(region, p, holes, relief, side);
    }
    p.validate()?;
    relief.validate()?;
    if region.is_empty() {
        return Err(GenError::Empty("empty pocket region".into()));
    }
    let w = p.tool_diameter * 0.5 + p.stock_to_leave;
    let center_final = non_sliver_center(region, w)?;
    if center_final.is_empty() {
        return Err(GenError::Invalid("pocket is narrower than the tool".into()));
    }
    let relief_plan = plan_corner_reliefs(region, &center_final, w, relief)?;
    let clear_gap = finish.clearing_gap(p);
    let center_clear = non_sliver_center(region, w + clear_gap)?;
    if center_clear.is_empty() {
        return Err(GenError::Invalid(empty_clear_msg(finish, clear_gap)));
    }
    let strategy = resolve_strategy(region, p);
    let side = &super::ring_side::resolve(&center_clear, strategy, p.tool_diameter * 0.5, side)?;
    let p_clear = PocketParams { stock_to_leave: p.stock_to_leave + clear_gap, ..*p };
    let (clear_passes, clear_notes, clear_strategy, clear_room, clear_joins, clear_ramps) =
        plan_clearing(region, &center_clear, &p_clear, strategy, side, "this pocket")?;
    let clear_passes = if p.links.approach.arc_xy && clear_room.is_none() {
        super::descent_xy::reseat_first(clear_passes)
    } else {
        clear_passes
    };
    let clear_passes = super::predrilled::reseat(
        clear_passes,
        holes,
        &center_clear,
        p.links.approach.predrilled && clear_room.is_none() && side.start == PassStart::Auto,
    );
    let anchor = clear_passes.last().and_then(|v| v.last()).copied().unwrap_or(Point2::new(0.0, 0.0));
    let fin_passes = plan_pocket_finish_rings(region, p, finish, None, anchor)?;
    let fin_climb = finish.finish_direction.resolve_climb(p.climb);
    let zs = cut_levels_for(p.top_z, &p.links.approach, p.bottom_z, p.stepdown)?;
    let mut b = PathBuilder::approached(Point3::new(0.0, 0.0, p.clearance_z));
    clear_notes.into_iter().for_each(|n| b.note(n));
    let mut proof = ProofTally::default();
    let mut clear_runs: Vec<Vec<Seg>> = clear_passes
        .iter()
        .map(|pass| fit_clear_pass(pass, clear_strategy, &center_clear, &mut proof))
        .collect();
    if p.blend_turns {
        let legal = Legal { region: &center_clear, inside: true, air_left: None };
        let rho = corner_arc_radius(p.blend_radius, p.tool_diameter).map_err(GenError::Invalid)?;
        let rounded = round_walks(&clear_passes, &clear_runs, &Blend { rho, legal }, &Bans::default());
        rounded.notes().into_iter().for_each(|n| b.note(n));
        clear_runs = rounded.runs;
    }
    let fin_runs: Vec<Vec<Seg>> = fin_passes.iter().map(|pass| fit_finish_ring(pass, fin_climb, &mut proof)).collect();
    let n_clear = clear_passes.len();
    let mut all_passes = clear_passes.clone();
    all_passes.extend(fin_passes.iter().cloned());
    let mut all_runs = clear_runs;
    all_runs.extend(fin_runs);
    let helix_r = clear_room.as_ref().map_or(p.helix_radius, |e| e.helix_r);
    let clear_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, helix_r).with_arc_radius(p.tool_diameter).with_links(p.links, helix_r)
        .with_top(p.links.approach.entry_top(p.top_z)).with_xy_arc(p.links.approach.arc_xy)
        .with_microlift(clear_joins).with_holes(holes).with_solid_restart()
        .with_cleared_links(side.start != PassStart::Auto).with_ramp_starts(clear_ramps);
    let fin_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, p.helix_radius).with_arc_radius(p.tool_diameter).with_links(p.links, p.helix_radius)
        .with_top(p.links.approach.entry_top(p.top_z)).with_xy_arc(p.links.approach.arc_xy)
        .with_solid_restart().with_cleared_links(true);
    let clear_cert = match &clear_room {
        Some(e) => Cert::Stadium { a: e.seed, b: e.seed, h: e.clear, tess: &e.tess },
        None => Cert::Region(&center_clear),
    };
    let (clear_links_cert, fin_cert) = (Cert::Region(&center_clear), Cert::Region(&center_final));
    let reliefs: &[ResolvedRelief] = &relief_plan.reliefs;
    let post = |b: &mut PathBuilder, runs: Option<&[Vec<Seg>]>| {
        let (c_runs, f_runs) = (runs.and_then(|rs| rs.get(..n_clear)), runs.and_then(|rs| rs.get(n_clear..)));
        let (fin_p_rel, fin_r_rel) = if reliefs.is_empty() {
            (None, None)
        } else {
            let (mut fp, mut fr) = (fin_passes.clone(), f_runs.map(<[Vec<Seg>]>::to_vec));
            inject_relief_spurs(&mut fp, &mut fr, &center_final, reliefs);
            (Some(fp), fr)
        };
        let fp_slice = fin_p_rel.as_deref().unwrap_or(&fin_passes);
        let fr_slice = if reliefs.is_empty() { f_runs } else { fin_r_rel.as_deref() };
        let mut ws = WalkState::new(clear_motion.top_z);
        for (li, &z) in zs.iter().enumerate() {
            ws.next_level();
            walk_one_level(b, z, &clear_passes, c_runs, &clear_motion, Some(&clear_cert), Some(&clear_links_cert), None, &mut ws);
            if level_runs_finishing(finish, li, zs.len()) && !fp_slice.is_empty() {
                if finish.finish_level == FinishLevel::Last && zs.len() > 1 {
                    ws.set_above(fin_motion.top_z);
                }
                ws.add_level_cuts(&clear_passes);
                walk_one_level(b, z, fp_slice, fr_slice, &fin_motion, Some(&fin_cert), Some(&fin_cert), None, &mut ws);
            }
        }
        ws.finish(b, &clear_motion);
        crate::fed_retract::lift_approach(b, p.clearance_z, ws.above(), clear_motion.hop_z, &clear_motion.links.approach);
    };
    let mut path = post_crossing_free(b, &all_passes, all_runs, &mut proof, post);
    if let Some(n) = proof.note() {
        path.notes.push(n);
    }
    if let Some(why) = crate::approach::refusal(&path) {
        return Err(GenError::Invalid(why).with_ghost(&path));
    }
    if !path.all_finite() {
        return Err(GenError::Invalid("path left the finite horizon".into()));
    }
    Ok(path)
}

pub(super) fn empty_clear_msg(finish: &PocketFinish, clear_gap: f64) -> String {
    if finish.finish_passes > 0 {
        format!(
            "2D Pocket finishing: with {} finishing passes of {:.3} mm ({:.3} mm wall allowance), \
             no room remains for clearing - reduce finishing passes or spacing",
            finish.finish_passes, finish.finish_spacing, finish.clearing_allowance()
        )
    } else {
        format!(
            "2D Pocket finishing: no clearing room remains inside the {clear_gap:.3} mm spring-pass wall skim - \
             use a smaller tool or set spring passes to 0"
        )
    }
}
