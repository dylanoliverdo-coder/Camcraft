//! Wall finishing passes after 2D Adaptive clearing (Journeyman spec S5):
//! contour-parallel finishing and spring passes round every wall the area
//! touches, turning by `climb` (opposite senses on outer walls and islands/bosses).
//! Provenance: Journeyman spec S5
//! (docs/journeyman/specs/S5-adaptive-finish-passes.md); M. Held, *On the
//! Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991;
//! M. Stori and P. K. Wright, "Constant engagement tool path generation for
//! convex geometries", J. Manuf. Systems 19 (2000) 172-184; no vendor
//! parameters, text or code.

use camcraft_sim::verify::AvoidSpec;

use crate::cl::{MoveKind, PathBuilder, Toolpath};
use crate::geo2d::{Point2, Region};
use crate::geometry::Point3;
use crate::machining2d::adaptive::finish_rings::plan_finish_rings;
use crate::machining2d::adaptive_avoid::{finish as finish_avoid, keepout_for};
use crate::machining2d::adaptive_troch::{AdaptiveTrochParams, adaptive_trochoidal_with, trochoidal_inside_finish};
use crate::machining2d::arc_cross::post_crossing_free;
use crate::machining2d::arc_proof::{ProofTally, fit_proved};
use crate::machining2d::arcs::{ARC_FIT_TOL, Seg, fit_arcs};
use crate::machining2d::blend_cross::{Bans, Blend, round_walks};
use crate::machining2d::contour_passes::{FinishLevel, MAX_WALL_PASSES};
use crate::machining2d::corner_relief::{CornerRelief, ResolvedRelief, capsule_region, plan_corner_reliefs};
use crate::machining2d::corner_relief_spur::inject_relief_spurs;
use crate::machining2d::equal_steps::cut_levels_for;
use crate::machining2d::face::GenError;
use crate::machining2d::fillet::Legal;
use crate::machining2d::fillet_tally::corner_arc_radius;
use crate::machining2d::levels::{Cert, Motion, WalkState, walk_one_level};
use crate::machining2d::pass_start::{PassStart, StartSide};
use crate::machining2d::pocket::{PocketParams, PocketStrategy, non_sliver_center, pocket_started_on};
use crate::machining2d::pocket_hsm_start::adaptive_passes_from;

/// Wall finishing controls of a 2D Adaptive operation (Journeyman spec S5).
/// The default (`finish_passes = 0`, `spring_passes = 0`) is today's clearing
/// alone, byte for byte.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveFinish {
    /// Finishing passes round every wall after clearing; `0` is none.
    pub finish_passes: u32,
    /// Distance between finishing passes (mm); unused while `finish_passes == 0`.
    pub finish_spacing: f64,
    /// Repeats of the final wall pass at `w = r + s`.
    pub spring_passes: u32,
    /// The levels where finishing and spring passes run (`Every` or `Last`).
    pub finish_level: FinishLevel,
}

impl Default for AdaptiveFinish {
    fn default() -> Self {
        Self { finish_passes: 0, finish_spacing: 0.0, spring_passes: 0, finish_level: FinishLevel::Every }
    }
}

impl AdaptiveFinish {
    /// True when every field is at its pre-S5 default (off wire and digest).
    #[must_use]
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }

    /// True when at least one finishing or spring pass will be emitted.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.finish_passes > 0 || self.spring_passes > 0
    }

    /// Named refusal for invalid finishing parameters (spec S5 "Refusals").
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
    /// a tiny 0.01 mm skim gap when `n_f == 0 && spring_passes > 0` (same law
    /// as `pocket_zigzag.rs` so clearing chords never cross the spring ring at `w`).
    #[must_use]
    pub(crate) fn clearing_gap(&self, p: &PocketParams) -> f64 {
        if !self.is_active() {
            0.0
        } else if self.finish_passes > 0 {
            self.clearing_allowance()
        } else {
            0.01_f64.min(crate::load::load_step(p.tool_diameter, p.optimal_load) * 0.25)
        }
    }
}

/// Flute-length law for `finish_level = Last` (spec S5): a finishing pass at
/// the last level cuts the whole wall height `top_z - bottom_z` with the side
/// of the tool, so the flute length must cover it.
pub fn adaptive_flute_covers_wall(top_z: f64, bottom_z: f64, flute_len: f64) -> Result<(), GenError> {
    let wall = top_z - bottom_z;
    if !(wall.is_finite() && flute_len.is_finite() && flute_len > 0.0) {
        return Err(GenError::Invalid(
            "2D Adaptive finishing: the tool states no flute length, so the wall height a last-level finishing pass cuts is unknown".into(),
        ));
    }
    if flute_len + 1e-9 < wall {
        return Err(GenError::Invalid(format!(
            "2D Adaptive finishing: the wall is {wall:.3} mm tall but the tool's flute is {flute_len:.3} mm long - \
             a finishing pass at the last level cuts the whole wall with the side of the tool; use a tool \
             with a flute at least {wall:.3} mm long"
        )));
    }
    Ok(())
}

/// Whether cut level `z` of `levels` (descending) runs finishing/spring passes.
pub(crate) fn level_runs_finishing(finish: &AdaptiveFinish, levels: &[f64], z: f64) -> bool {
    match finish.finish_level {
        FinishLevel::Every => true,
        FinishLevel::Last => levels.last().is_some_and(|&last| z.to_bits() == last.to_bits()),
    }
}

/// 2D Adaptive (Spiral) with optional wall finishing passes (spec S5).
/// When `finish` is inactive, delegates byte-for-byte to `pocket_started_on`.
pub fn adaptive_with_finish(
    region: &Region,
    p: &PocketParams,
    relief: &CornerRelief,
    side: &StartSide,
    finish: &AdaptiveFinish,
) -> Result<Toolpath, GenError> {
    if let Some(why) = finish.refusal() {
        return Err(GenError::Invalid(why));
    }
    if !finish.is_active() {
        return pocket_started_on(region, p, &[], relief, side);
    }
    adaptive_finish_inside(region, p, relief, side, finish, None)
}

/// 2D Adaptive (Spiral) with "Do not touch" keep-outs and wall finishing passes.
pub fn adaptive_avoid_with_finish(
    region: &Region,
    p: &PocketParams,
    avoid: &AvoidSpec,
    relief: &CornerRelief,
    side: &StartSide,
    finish: &AdaptiveFinish,
) -> Result<Toolpath, GenError> {
    if let Some(why) = finish.refusal() {
        return Err(GenError::Invalid(why));
    }
    if !finish.is_active() {
        return crate::machining2d::adaptive_spiral_avoid_on(region, p, avoid, relief, side);
    }
    let Some(ko) = keepout_for(region, p, avoid)? else {
        return adaptive_finish_inside(region, p, relief, side, finish, None);
    };
    let names = avoid.names_list();
    let path = adaptive_finish_inside(region, p, relief, side, finish, Some((&ko, &names)))
        .map_err(|e| e.map_msg(|m| format!("{m} - the 'Do not touch' keep-out of {names} takes part of the region")))?;
    finish_avoid(path, avoid, "the passes")
}

/// 2D Adaptive (Trochoidal) with optional keep-outs and wall finishing passes.
pub fn adaptive_trochoidal_with_finish(
    region: &Region,
    p: &PocketParams,
    t: &AdaptiveTrochParams,
    avoid: Option<&AvoidSpec>,
    finish: &AdaptiveFinish,
) -> Result<Toolpath, GenError> {
    if let Some(why) = finish.refusal() {
        return Err(GenError::Invalid(why));
    }
    if !finish.is_active() {
        return match avoid {
            Some(spec) => crate::machining2d::adaptive_trochoidal_avoid(region, p, t, spec),
            None => adaptive_trochoidal_with(region, p, t),
        };
    }
    let ko = match avoid {
        Some(spec) => keepout_for(region, p, spec)?,
        None => None,
    };
    let path = trochoidal_inside_finish(region, p, t, ko.as_ref(), Some(finish))?;
    match (avoid, ko) {
        (Some(spec), Some(_)) => finish_avoid(path, spec, "the trochoidal lanes"),
        _ => Ok(path),
    }
}

/// Core spiral-clearing + contour-parallel wall finishing generator (spec S5).
fn adaptive_finish_inside(
    region: &Region,
    p: &PocketParams,
    relief: &CornerRelief,
    side: &StartSide,
    finish: &AdaptiveFinish,
    keepout: Option<(&Region, &str)>,
) -> Result<Toolpath, GenError> {
    p.validate()?;
    relief.validate()?;
    if region.is_empty() {
        return Err(GenError::Empty("empty pocket region".into()));
    }
    let w = p.tool_diameter * 0.5 + p.stock_to_leave;
    let extra = finish.clearing_allowance();
    let mut center_final = non_sliver_center(region, w)?;
    if center_final.is_empty() {
        return Err(GenError::Invalid("pocket is narrower than the tool".into()));
    }
    let relief_plan = plan_corner_reliefs(region, &center_final, w, relief)?;
    if let Some((ko, names)) = keepout {
        for cr in &relief_plan.reliefs {
            if !capsule_region(cr.start, cr.end, cr.radius)?.intersect(ko)?.is_empty() {
                return Err(GenError::Invalid(format!(
                    "corner relief at ({:.3}, {:.3}) enters protected 'Do not touch' zone ({names})",
                    cr.corner.x, cr.corner.y
                )));
            }
        }
        center_final = center_final.difference(ko)?;
        if center_final.is_empty() {
            return Err(GenError::Invalid(format!(
                "cannot cut clear of protected 'Do not touch' {names}: no safe cutting room remains"
            )));
        }
    }
    let clear_gap = finish.clearing_gap(p);
    let mut center_clear = non_sliver_center(region, w + clear_gap)?;
    if let Some((ko, _)) = keepout {
        center_clear = center_clear.difference(ko)?;
    }
    if center_clear.is_empty() {
        return Err(GenError::Invalid(format!(
            "2D Adaptive finishing: with {} finishing passes of {:.3} mm ({extra:.3} mm wall allowance), \
             no room remains for clearing - reduce finishing passes or spacing",
            finish.finish_passes, finish.finish_spacing
        )));
    }
    let p_clear = PocketParams { stock_to_leave: p.stock_to_leave + clear_gap, ..*p };
    let (clear_passes, clear_notes, clear_strategy, clear_room, clear_joins) =
        adaptive_passes_from(region, &center_clear, &p_clear, "this pocket", side)?;
    let clear_passes = if p.links.approach.arc_xy && clear_room.is_none() {
        crate::machining2d::descent_xy::reseat_first(clear_passes)
    } else {
        clear_passes
    };
    let anchor = clear_passes.last().and_then(|v| v.last()).copied().unwrap_or(Point2::new(0.0, 0.0));
    let fin_passes = plan_finish_rings(region, p, finish, keepout.map(|(ko, _)| ko), anchor)?;
    let zs = cut_levels_for(p.top_z, &p.links.approach, p.bottom_z, p.stepdown)?;
    let mut b = PathBuilder::approached(Point3::new(0.0, 0.0, p.clearance_z));
    clear_notes.into_iter().for_each(|n| b.note(n));
    let mut proof = ProofTally::default();
    let clear_tol = if clear_strategy == PocketStrategy::Adaptive { ARC_FIT_TOL * 0.25 } else { ARC_FIT_TOL };
    let mut clear_runs: Vec<Vec<Seg>> = clear_passes
        .iter()
        .map(|pass| {
            if clear_strategy == PocketStrategy::Adaptive {
                fit_proved(pass, clear_tol, &center_clear, &mut proof)
            } else {
                let segs = fit_arcs(pass, clear_tol);
                proof.arcs += segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
                segs
            }
        })
        .collect();
    if p.blend_turns {
        let legal = Legal { region: &center_clear, inside: true, air_left: None };
        let rho = corner_arc_radius(p.blend_radius, p.tool_diameter).map_err(GenError::Invalid)?;
        let rounded = round_walks(&clear_passes, &clear_runs, &Blend { rho, legal }, &Bans::default());
        rounded.notes().into_iter().for_each(|n| b.note(n));
        clear_runs = rounded.runs;
    }
    let fin_runs: Vec<Vec<Seg>> = fin_passes
        .iter()
        .map(|pass| {
            let segs = fit_arcs(pass, ARC_FIT_TOL);
            proof.arcs += segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
            segs
        })
        .collect();
    let n_clear = clear_passes.len();
    let mut all_passes = clear_passes.clone();
    all_passes.extend(fin_passes.iter().cloned());
    let mut all_runs = clear_runs;
    all_runs.extend(fin_runs);
    let helix_r = clear_room.as_ref().map_or(p.helix_radius, |e| e.helix_r);
    let clear_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, helix_r).with_arc_radius(p.tool_diameter).with_links(p.links, helix_r)
        .with_top(p.links.approach.entry_top(p.top_z)).with_xy_arc(p.links.approach.arc_xy)
        .with_microlift(clear_joins).with_solid_restart().with_cleared_links(side.start != PassStart::Auto);
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
        for &z in &zs {
            walk_one_level(b, z, &clear_passes, c_runs, &clear_motion, Some(&clear_cert), Some(&clear_links_cert), None, &mut ws);
            if level_runs_finishing(finish, &zs, z) && !fp_slice.is_empty() {
                ws.add_level_cuts(&clear_passes);
                walk_one_level(b, z, fp_slice, fr_slice, &fin_motion, Some(&fin_cert), Some(&fin_cert), None, &mut ws);
            }
        }
        ws.finish(b, &clear_motion);
        let cur = b.at();
        b.push(MoveKind::Rapid, Point3::new(cur.x, cur.y, p.clearance_z), None);
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

/// Emit wall finishing rings at level `z` after trochoidal clearing/cleanup.
pub(crate) fn emit_troch_finish_level(
    bld: &mut PathBuilder,
    fin_passes: &[Vec<Point2>],
    fin_runs: &[Vec<Seg>],
    center_final: &Region,
    p: &PocketParams,
    z: f64,
    above: f64,
) {
    if fin_passes.is_empty() {
        return;
    }
    let fin_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, p.helix_radius).with_arc_radius(p.tool_diameter)
        .with_links(p.links, p.helix_radius).with_top(above).with_solid_restart().with_cleared_links(true);
    let cert = Cert::Region(center_final);
    let prior_cuts: Vec<Vec<Point2>> = bld
        .moves()
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && (m.from.z - z).abs() < 1e-6 && (m.to.z - z).abs() < 1e-6)
        .map(|m| vec![Point2::new(m.from.x, m.from.y), Point2::new(m.to.x, m.to.y)])
        .collect();
    let mut ws = WalkState::resumed_at(z, above);
    ws.add_level_cuts(&prior_cuts);
    walk_one_level(bld, z, fin_passes, Some(fin_runs), &fin_motion, Some(&cert), Some(&cert), None, &mut ws);
    ws.finish(bld, &fin_motion);
}
