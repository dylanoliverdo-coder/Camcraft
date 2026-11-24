//! Contour-parallel wall finishing rings for 2D Adaptive (Journeyman spec S5):
//! extracts oriented rings at `w + (n_f - j) * d_f` and `w` round every wall
//! the area touches (outer boundary, boss outline, and islands).
//! Provenance: Journeyman spec S5
//! (docs/journeyman/specs/S5-adaptive-finish-passes.md); M. Held, *On the
//! Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991;
//! M. Stori and P. K. Wright, "Constant engagement tool path generation for
//! convex geometries", J. Manuf. Systems 19 (2000) 172-184; no vendor
//! parameters, text or code.

use crate::cl::{MoveKind, PathBuilder};
use crate::geo2d::{Loop, Point2, Region};
use crate::machining2d::arc_cross::crossing_free_on;
use crate::machining2d::arc_proof::ProofTally;
use crate::machining2d::arcs::{ARC_FIT_TOL, Seg, fit_arcs};
use crate::machining2d::face::GenError;
use crate::machining2d::levels::{Cert, Motion, WalkState, walk_one_level};
use crate::machining2d::pocket::{PocketParams, non_sliver_center};

use super::finish_passes::AdaptiveFinish;

/// Contour-parallel finishing and spring rings round every wall of `region` at
/// offsets `w + (n_f - j) * d_f` (`j = 1..=n_f`) and `w` for spring passes,
/// oriented by `p.climb` (opposite turning senses on outer walls vs holes/bosses).
pub(crate) fn plan_finish_rings(
    region: &Region,
    p: &PocketParams,
    finish: &AdaptiveFinish,
    keepout: Option<&Region>,
    anchor: Point2,
) -> Result<Vec<Vec<Point2>>, GenError> {
    let w = p.tool_diameter * 0.5 + p.stock_to_leave;
    let nf = finish.finish_passes;
    let df = if nf > 0 { finish.finish_spacing } else { 0.0 };
    let mut passes = Vec::new();
    let mut cur = anchor;
    let mut final_rings: Vec<Vec<Point2>> = Vec::new();
    for j in 1..=nf {
        let extra_off = f64::from(nf - j) * df;
        let off = w + extra_off;
        let rings = rings_at_offset(region, off, extra_off, p.climb, keepout, &mut cur)?;
        if j == nf {
            final_rings = rings.clone();
        }
        passes.extend(rings);
    }
    if finish.spring_passes > 0 {
        if final_rings.is_empty() {
            final_rings = rings_at_offset(region, w, 0.0, p.climb, keepout, &mut cur)?;
        }
        for _ in 0..finish.spring_passes {
            passes.extend(final_rings.iter().cloned());
        }
    }
    Ok(passes)
}

/// Oriented closed wall rings at offset `off`, greedily nearest `cur`.
/// When `keepout` is present, grows it by `extra_ko` (`off - w`) so each
/// finishing ring stays strictly separated from the next ring along keep-out
/// boundaries too (never sharing a coincident edge that could cross).
fn rings_at_offset(
    region: &Region,
    off: f64,
    extra_ko: f64,
    climb: bool,
    keepout: Option<&Region>,
    cur: &mut Point2,
) -> Result<Vec<Vec<Point2>>, GenError> {
    let mut band = non_sliver_center(region, off)?;
    if let Some(ko) = keepout
        && !ko.is_empty()
    {
        let ko_d = if extra_ko > 0.0 { ko.offset(extra_ko)? } else { ko.clone() };
        band = band.difference(&ko_d)?;
    }
    if band.is_empty() {
        return Ok(Vec::new());
    }
    let mut rem: Vec<&Loop> = band.loops().iter().filter(|l| l.points().len() >= 2).collect();
    let mut out = Vec::with_capacity(rem.len());
    while !rem.is_empty() {
        let best = rem
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                nearest_on_loop(a.points(), *cur).0.total_cmp(&nearest_on_loop(b.points(), *cur).0)
            })
            .map_or(0, |(i, _)| i);
        let lp = rem.swap_remove(best);
        let oriented: Vec<Point2> = if climb {
            lp.points().to_vec()
        } else {
            lp.points().iter().rev().copied().collect()
        };
        let ring = start_ring_near(&oriented, *cur);
        if let Some(&s) = ring.first() {
            *cur = s;
        }
        if ring.len() >= 2 {
            out.push(ring);
        }
    }
    Ok(out)
}

/// Shortest distance from `q` to closed polygon `pts`, plus `(edge_idx, t, foot)`.
fn nearest_on_loop(pts: &[Point2], q: Point2) -> (f64, usize, f64, Point2) {
    let n = pts.len();
    let mut best = (f64::INFINITY, 0usize, 0.0f64, pts.first().copied().unwrap_or(q));
    for i in 0..n {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let dd = dx * dx + dy * dy;
        let t = if dd > 0.0 && dd.is_finite() {
            (((q.x - a.x) * dx + (q.y - a.y) * dy) / dd).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let foot = Point2::new(a.x + t * dx, a.y + t * dy);
        let d = (q.x - foot.x).hypot(q.y - foot.y);
        if d < best.0 {
            best = (d, i, t, foot);
        }
    }
    best
}

/// Rotate closed loop `pts` to start nearest `anchor` (projecting onto straight
/// edges >= 1 mm, snapping to existing vertices on fine curves), then close.
fn start_ring_near(pts: &[Point2], anchor: Point2) -> Vec<Point2> {
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let (_, ei, t, foot) = nearest_on_loop(pts, anchor);
    let (a, b) = (pts[ei], pts[(ei + 1) % n]);
    let seg_len = (b.x - a.x).hypot(b.y - a.y);
    let (da, db) = ((foot.x - a.x).hypot(foot.y - a.y), (foot.x - b.x).hypot(foot.y - b.y));
    let mut ring = Vec::with_capacity(n + 2);
    if seg_len >= 1.0 && da > 0.05 && db > 0.05 {
        ring.push(foot);
        for k in 1..=n {
            ring.push(pts[(ei + k) % n]);
        }
        ring.push(foot);
    } else {
        let start = if t <= 0.5 { ei } else { (ei + 1) % n };
        for k in 0..n {
            ring.push(pts[(start + k) % n]);
        }
        ring.push(pts[start]);
    }
    ring
}

/// Emit wall finishing rings at level `z` after trochoidal clearing/cleanup.
/// Checks fitted finishing arcs with `crossing_free_on` against all prior
/// moves in `bld`, expands curved prior arcs with `chords(ARC_FIT_TOL)`, and
/// enters from retract whenever `bld.at().z` is not already at cut plane `z`.
pub(crate) fn emit_troch_finish_level(
    bld: &mut PathBuilder,
    fin_passes: &[Vec<Point2>],
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
    let entry_links = crate::links::Links { level: crate::links::Link::Retract, ..p.links };
    let entry_motion = Motion::new(p.clearance_z, p.retract_z, p.feed, p.plunge_feed)
        .with_entry(p.entry, p.helix_radius).with_arc_radius(p.tool_diameter)
        .with_links(entry_links, p.helix_radius).with_top(above).with_solid_restart().with_cleared_links(true);
    let cert = Cert::Region(center_final);
    let prior_cuts: Vec<Vec<Point2>> = bld
        .moves()
        .iter()
        .filter(|m| m.kind == MoveKind::Feed && (m.from.z - z).abs() < 1e-6 && (m.to.z - z).abs() < 1e-6)
        .flat_map(|m| m.chords(ARC_FIT_TOL).into_iter().map(|(a, b)| vec![a, b]))
        .collect();
    let at_cut_plane = (bld.at().z - z).abs() < 1e-6;

    let emit = |bb: &mut PathBuilder, runs: &[Vec<Seg>]| {
        if at_cut_plane {
            let mut ws = WalkState::resumed_at(z, above);
            ws.add_level_cuts(&prior_cuts);
            walk_one_level(bb, z, fin_passes, Some(runs), &fin_motion, Some(&cert), Some(&cert), None, &mut ws);
            ws.finish(bb, &fin_motion);
        } else {
            let mut ws = WalkState::retracted(above);
            walk_one_level(bb, z, &fin_passes[..1], runs.get(..1), &entry_motion, Some(&cert), Some(&cert), None, &mut ws);
            if fin_passes.len() > 1 {
                ws.add_level_cuts(&prior_cuts);
                ws.add_level_cuts(&fin_passes[..1]);
                walk_one_level(bb, z, &fin_passes[1..], runs.get(1..), &fin_motion, Some(&cert), Some(&cert), None, &mut ws);
            }
            ws.finish(bb, &fin_motion);
        }
    };

    let mut proof = ProofTally::default();
    let raw_runs: Vec<Vec<Seg>> = fin_passes
        .iter()
        .map(|ring| {
            let segs = fit_arcs(ring, ARC_FIT_TOL);
            proof.arcs += segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
            segs
        })
        .collect();
    let safe_runs = crossing_free_on(bld, fin_passes, raw_runs, &mut proof, emit);
    emit(bld, &safe_runs);
    if let Some(n) = proof.note() {
        bld.note(n);
    }
}
