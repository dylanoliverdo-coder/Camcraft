//! Per-level "Do not touch" keep-out adapter for 2D Pocket wall finishing (Journeyman spec S4).
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991;
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007; no vendor parameters,
//! text or code.

use camcraft_sim::verify::{AVOID_PAD, AvoidSpec};

use crate::cl::{PathBuilder, Toolpath};
use crate::geo2d::{Point2, Region};
use crate::geometry::Point3;

use super::arc_proof::ProofTally;
use super::arcs::{ARC_FIT_TOL, Seg};
use super::contour_passes::FinishLevel;
use super::corner_relief::{CornerRelief, plan_corner_reliefs};
use super::corner_relief_spur::inject_relief_spurs;
use super::equal_steps::cut_levels_for;
use super::face::GenError;
use super::fillet::{Legal, fillet_corners};
use super::fillet_tally::{FilletTally, corner_arc_radius, fillet_floor};
use super::levels::{Cert, Motion, WalkState, walk_one_level};
use super::pass_start::{PassStart, StartSide};
use super::pocket::{PocketParams, PocketStrategy, non_sliver_center};
use super::pocket_avoid::pocket_avoid_entered_on;
use super::pocket_avoid_level::{check_reliefs_avoid, enforce_level_crossing_free};
use super::pocket_finish::{
    PocketFinish, check_spacing_vs_tool, empty_clear_msg, pocket_with_finish,
};
use super::pocket_finish_rings::{
    fit_clear_pass, fit_finish_ring, plan_clearing, plan_pocket_finish_rings, resolve_strategy,
};
use super::predrilled::PreHole;

/// 2D Pocket with "Do not touch" keep-outs and wall finishing passes (spec S4).
/// When `avoid` is empty, delegates to `pocket_with_finish`; when `finish` is
/// inactive, delegates byte-for-byte to `pocket_avoid_entered_on`.
pub fn pocket_avoid_with_finish(
    region: &Region,
    p: &PocketParams,
    holes: &[PreHole],
    avoid: &AvoidSpec,
    relief: &CornerRelief,
    side: &StartSide,
    finish: &PocketFinish,
) -> Result<Toolpath, GenError> {
    if let Some(why) = finish.refusal() {
        return Err(GenError::Invalid(why));
    }
    if avoid.is_empty() {
        return pocket_with_finish(region, p, holes, relief, side, finish);
    }
    if !finish.is_active() {
        return pocket_avoid_entered_on(region, p, holes, avoid, relief, side);
    }
    p.validate()?;
    check_spacing_vs_tool(finish, p)?;
    relief.validate()?;
    if region.is_empty() {
        return Err(GenError::Empty("empty pocket region".into()));
    }
    let w = p.tool_diameter * 0.5 + p.stock_to_leave;
    let base_final = non_sliver_center(region, w)?;
    if base_final.is_empty() {
        return Err(GenError::Invalid("pocket is narrower than the tool".into()));
    }
    let relief_plan = plan_corner_reliefs(region, &base_final, w, relief)?;
    let clear_gap = finish.clearing_gap(p);
    let base_clear = non_sliver_center(region, w + clear_gap)?;
    if base_clear.is_empty() {
        return Err(GenError::Invalid(empty_clear_msg(finish, clear_gap)));
    }
    let strategy = resolve_strategy(region, p);
    let fin_climb = finish.finish_direction.resolve_climb(p.climb);
    let zs = cut_levels_for(p.top_z, &p.links.approach, p.bottom_z, p.stepdown)?;
    let mut b = PathBuilder::approached(Point3::new(0.0, 0.0, p.clearance_z));
    let mut ws = WalkState::new(p.links.approach.entry_top(p.top_z));
    let mut prev_center: Option<Region> = None;
    let mut blocked_levels: Vec<f64> = Vec::new();
    let mut notes: Vec<String> = Vec::new();
    let mut tally = FilletTally::default();
    let mut proof = ProofTally::default();

    // Pre-compute per-level keep-outs so `FinishLevel::Last` knows the deepest
    // reachable level when an `avoid` obstacle blocks the bottom level.
    let mut prepared: Vec<Option<(Region, Region, Region)>> = Vec::with_capacity(zs.len());
    for &z in &zs {
        let ko = avoid.keepout_region(z, z, AVOID_PAD).map_err(GenError::Invalid)?;
        let ko_entry = avoid.keepout_region(z, p.clearance_z.max(z), AVOID_PAD).map_err(GenError::Invalid)?;
        check_reliefs_avoid(&relief_plan.reliefs, &ko, &ko_entry, avoid)?;
        let ko_all = match (ko.is_empty(), ko_entry.is_empty()) {
            (true, true) => Region::empty(),
            (false, true) => ko,
            (true, false) => ko_entry,
            (false, false) => ko.union(&ko_entry)?,
        };
        let level_final = if ko_all.is_empty() { base_final.clone() } else { base_final.difference(&ko_all)? };
        let level_clear = if ko_all.is_empty() {
            base_clear.clone()
        } else {
            let ko_g = if clear_gap > 0.0 { ko_all.offset(clear_gap)? } else { ko_all.clone() };
            base_clear.difference(&ko_g)?
        };
        if level_final.is_empty() || level_clear.is_empty() {
            prepared.push(None);
        } else {
            prepared.push(Some((ko_all, level_final, level_clear)));
        }
    }
    let last_active_li = prepared.iter().rposition(Option::is_some);
    let mut ran_finish = false;

    for (li, (&z, slot)) in zs.iter().zip(prepared).enumerate() {
        let Some((ko_all, level_final, level_clear)) = slot else {
            blocked_levels.push(z);
            continue;
        };
        let level_side = super::ring_side::resolve(&level_clear, strategy, p.tool_diameter * 0.5, side)?;
        let p_clear = PocketParams { stock_to_leave: p.stock_to_leave + clear_gap, ..*p };
        let what = format!("level at Z {z:.2}");
        let (clear_passes, cnotes, clear_strategy, clear_room, clear_joins, clear_ramps) =
            plan_clearing(region, &level_clear, &p_clear, strategy, &level_side, &what)?;
        for n in cnotes {
            if !notes.contains(&n) {
                notes.push(n);
            }
        }
        let clear_passes = if p.links.approach.arc_xy && clear_room.is_none() && clear_strategy != PocketStrategy::Adaptive {
            super::descent_xy::reseat_first(clear_passes)
        } else {
            clear_passes
        };
        let clear_passes = super::predrilled::reseat(
            clear_passes,
            holes,
            &level_clear,
            p.links.approach.predrilled && clear_room.is_none() && level_side.start == PassStart::Auto,
        );
        if clear_passes.iter().all(|ps| ps.len() < 2) {
            blocked_levels.push(z);
            continue;
        }
        let mut clear_runs: Vec<Vec<Seg>> = if p.blend_turns && clear_strategy != PocketStrategy::Adaptive {
            let legal = Legal { region: &level_clear, inside: true, air_left: None };
            let rho = corner_arc_radius(p.blend_radius, p.tool_diameter).map_err(GenError::Invalid)?;
            clear_passes
                .iter()
                .map(|pass| {
                    let Some((first, _)) = pass.split_first() else {
                        return Vec::new();
                    };
                    let walk = fit_clear_pass(pass, clear_strategy, &level_clear, &mut proof);
                    let (walk, t) = fillet_corners(&walk, *first, rho, ARC_FIT_TOL, &legal);
                    tally.add(t);
                    walk
                })
                .collect()
        } else {
            clear_passes
                .iter()
                .map(|pass| fit_clear_pass(pass, clear_strategy, &level_clear, &mut proof))
                .collect()
        };
        let hr = clear_room.as_ref().map_or(p.helix_radius, |rm| p.helix_radius.min(rm.helix_r));
        let clear_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
            .with_entry(p.entry, hr).with_arc_radius(p.tool_diameter).with_links(p.links, hr)
            .with_top(p.links.approach.entry_top(p.top_z)).with_xy_arc(p.links.approach.arc_xy)
            .with_microlift(clear_joins).with_holes(holes).with_solid_restart()
            .with_cleared_links(level_side.start != PassStart::Auto).with_ramp_starts(clear_ramps);
        let cert_entry = clear_room.as_ref().map(|rm| Cert::Region(&rm.tess));
        let cert_clear = Cert::Region(&level_clear);
        ws.next_level();
        if !p.blend_turns {
            enforce_level_crossing_free(
                &b, &ws, z, &clear_passes, &mut clear_runs, &clear_motion,
                cert_entry.as_ref().unwrap_or(&cert_clear), &cert_clear, prev_center.as_ref(), &mut proof,
            );
        }
        walk_one_level(
            &mut b, z, &clear_passes, Some(&clear_runs), &clear_motion,
            Some(cert_entry.as_ref().unwrap_or(&cert_clear)), Some(&cert_clear), prev_center.as_ref(), &mut ws,
        );
        let run_fin_here = match finish.finish_level {
            FinishLevel::Every => true,
            FinishLevel::Last => last_active_li == Some(li),
        };
        if run_fin_here {
            let anchor = clear_passes.last().and_then(|v| v.last()).copied().unwrap_or(Point2::new(0.0, 0.0));
            let ko_opt = (!ko_all.is_empty()).then_some(&ko_all);
            let mut fin_passes = plan_pocket_finish_rings(region, p, finish, ko_opt, anchor)?;
            if !fin_passes.is_empty() {
                let mut fin_runs: Vec<Vec<Seg>> = fin_passes
                    .iter()
                    .map(|pass| fit_finish_ring(pass, fin_climb, &mut proof))
                    .collect();
                let fin_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
                    .with_entry(p.entry, p.helix_radius).with_arc_radius(p.tool_diameter)
                    .with_links(p.links, p.helix_radius).with_top(p.links.approach.entry_top(p.top_z))
                    .with_xy_arc(p.links.approach.arc_xy).with_solid_restart().with_cleared_links(true);
                if finish.finish_level == FinishLevel::Last && zs.len() > 1 {
                    ws.set_above(fin_motion.top_z);
                }
                ws.add_level_cuts(&clear_passes);
                let cert_fin = Cert::Region(&level_final);
                enforce_level_crossing_free(
                    &b, &ws, z, &fin_passes, &mut fin_runs, &fin_motion,
                    &cert_fin, &cert_fin, None, &mut proof,
                );
                if !relief_plan.reliefs.is_empty() {
                    let mut wrapped = Some(fin_runs);
                    inject_relief_spurs(&mut fin_passes, &mut wrapped, &level_final, &relief_plan.reliefs);
                    fin_runs = wrapped.unwrap_or_default();
                }
                walk_one_level(
                    &mut b, z, &fin_passes, Some(&fin_runs), &fin_motion,
                    Some(&cert_fin), Some(&cert_fin), None, &mut ws,
                );
                ran_finish = true;
                if finish.finish_level == FinishLevel::Last && li + 1 < zs.len() {
                    notes.push(format!(
                        "2D Pocket finishing: the bottom level at Z {:.3} was blocked by 'Do not touch' ({}); \
                         last-level finishing ran at the lowest clear level Z {:.3}",
                        zs[zs.len() - 1], avoid.names_list(), z
                    ));
                }
            }
        }
        prev_center = Some(level_clear);
    }

    if ws.last_cut_z.is_none() {
        return Err(GenError::Invalid(format!(
            "cannot cut clear of protected 'Do not touch' {}: no safe cutting room remains",
            avoid.names_list()
        )));
    }
    if !ran_finish {
        notes.push(format!(
            "2D Pocket finishing: wall finishing passes could not run because 'Do not touch' ({}) blocked the finishing level",
            avoid.names_list()
        ));
    }
    for n in notes {
        b.note(n);
    }
    for n in tally.notes("pass", fillet_floor(ARC_FIT_TOL)) {
        b.note(n);
    }
    if let Some(n) = proof.note() {
        b.note(n);
    }
    let end_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, p.helix_radius).with_arc_radius(p.tool_diameter)
        .with_links(p.links, p.helix_radius).with_top(p.links.approach.entry_top(p.top_z))
        .with_holes(holes);
    ws.finish(&mut b, &end_motion);
    crate::fed_retract::lift_approach(&mut b, p.clearance_z, ws.above(), end_motion.hop_z, &end_motion.links.approach);
    if !blocked_levels.is_empty() {
        b.note(format!(
            "Do not touch ({}) blocked {} cut level(s); remaining levels stayed clear",
            avoid.names_list(),
            blocked_levels.len()
        ));
    }
    let mut path = b.finish();
    let _ = camcraft_sim::verify::elevate_rapid_links(&mut path, avoid, None, AVOID_PAD);
    if let Some(why) = crate::approach::refusal(&path) {
        return Err(GenError::Invalid(why).with_ghost(&path));
    }
    if !path.all_finite() {
        return Err(GenError::Invalid("path left the finite horizon".into()));
    }
    Ok(path)
}
