//! Contour-parallel wall finishing ring planner and strategy helpers for 2D Pocket
//! (Journeyman spec S4).
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991
//! (contour-parallel passes round boundary and islands); P. Smid, *CNC Programming Handbook*,
//! 3rd ed., Industrial Press 2007 (climb and conventional milling); no vendor parameters,
//! text or code.

use crate::geo2d::{Loop, Point2, Region};

use super::arc_proof::{ProofTally, fit_proved};
use super::arcs::{ARC_FIT_TOL, Seg, fit_arcs, fit_arcs_sided};
use super::face::GenError;
use super::level_rings::ring_plan_ramped;
use super::pass_start::{PassStart, StartSide};
use super::pocket::{PocketParams, PocketStrategy, non_sliver_center};
use super::pocket_finish::PocketFinish;
use super::pocket_hsm::EntryRoom;
use super::pocket_hsm_start::adaptive_passes_from;
use super::pocket_zigzag::rest_passes;

/// Clearing pass plan output: passes, notes, resolved strategy, entry room,
/// microlift joins, and dead-end ramp starts.
pub(super) type ClearPlan = (
    Vec<Vec<Point2>>,
    Vec<String>,
    PocketStrategy,
    Option<EntryRoom>,
    Vec<Option<super::adaptive::Return>>,
    Vec<Point2>,
);

/// Resolve `PocketStrategy::Auto` from `region`'s outermost loop (matching `pocket.rs`).
pub(super) fn resolve_strategy(region: &Region, p: &PocketParams) -> PocketStrategy {
    match p.strategy {
        PocketStrategy::Auto => {
            let outer = region
                .loops()
                .iter()
                .max_by(|a, b| a.area().abs().total_cmp(&b.area().abs()))
                .map(|l| l.points())
                .unwrap_or(&[]);
            p.strategy.resolve(outer, p.stepover, p.tool_diameter, ARC_FIT_TOL)
        }
        s => s,
    }
}

/// Contour-parallel finishing and spring rings round every wall of `region` at
/// offsets `w + (n_f - j) * d_f` (`j = 1..=n_f`) and `w` for spring passes,
/// oriented by `finish.finish_direction.resolve_climb(p.climb)`.
pub(super) fn plan_pocket_finish_rings(
    region: &Region,
    p: &PocketParams,
    finish: &PocketFinish,
    keepout: Option<&Region>,
    anchor: Point2,
) -> Result<Vec<Vec<Point2>>, GenError> {
    let w = p.tool_diameter * 0.5 + p.stock_to_leave;
    let nf = finish.finish_passes;
    let df = if nf > 0 { finish.finish_spacing } else { 0.0 };
    let fin_climb = finish.finish_direction.resolve_climb(p.climb);
    let mut passes = Vec::new();
    let mut cur = anchor;
    let mut final_rings: Vec<Vec<Point2>> = Vec::new();
    for j in 1..=nf {
        let extra_off = f64::from(nf - j) * df;
        let rings = rings_at_offset(region, w + extra_off, extra_off, fin_climb, keepout, &mut cur)?;
        if j == nf {
            final_rings = rings.clone();
        }
        passes.extend(rings);
    }
    if finish.spring_passes > 0 {
        if final_rings.is_empty() {
            final_rings = rings_at_offset(region, w, 0.0, fin_climb, keepout, &mut cur)?;
        }
        for _ in 0..finish.spring_passes {
            passes.extend(final_rings.iter().cloned());
        }
    }
    Ok(passes)
}

/// Oriented closed wall rings at offset `off`, greedily nearest `cur`.
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

/// True when edge `(pts[ei], pts[(ei+1)%n])` is a genuine straight wall segment
/// (corners turn >= ~15 deg at its ends) rather than a facet of a curved circle.
fn is_straight_wall_edge(pts: &[Point2], ei: usize) -> bool {
    let n = pts.len();
    if n < 3 {
        return true;
    }
    let (prev, a, b, next) = (pts[(ei + n - 1) % n], pts[ei], pts[(ei + 1) % n], pts[(ei + 2) % n]);
    let cos_turn = |u: Point2, v: Point2| -> f64 {
        let (lu, lv) = (u.x.hypot(u.y), v.x.hypot(v.y));
        if lu > 1e-12 && lv > 1e-12 { (u.x * v.x + u.y * v.y) / (lu * lv) } else { 1.0 }
    };
    let e = Point2::new(b.x - a.x, b.y - a.y);
    let c0 = cos_turn(Point2::new(a.x - prev.x, a.y - prev.y), e);
    let c1 = cos_turn(e, Point2::new(next.x - b.x, next.y - b.y));
    c0 < 0.966 || c1 < 0.966
}

/// Rotate closed loop `pts` to start nearest `anchor` (projecting onto true
/// straight edges >= 1 mm, snapping to existing vertices on curved facets so
/// circle arc fitting stays exact), then close.
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
    if seg_len >= 1.0 && da > 0.05 && db > 0.05 && is_straight_wall_edge(pts, ei) {
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

/// Fit one clearing pass according to strategy.
pub(super) fn fit_clear_pass(
    pass: &[Point2],
    strategy: PocketStrategy,
    center_clear: &Region,
    proof: &mut ProofTally,
) -> Vec<Seg> {
    match strategy {
        PocketStrategy::Adaptive => fit_proved(pass, ARC_FIT_TOL * 0.25, center_clear, proof),
        PocketStrategy::Waterline | PocketStrategy::OffsetRings => {
            let segs = fit_arcs(pass, ARC_FIT_TOL);
            proof.arcs += segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
            segs
        }
        PocketStrategy::Zigzag | PocketStrategy::Auto => {
            vec![Seg::Line(pass.get(1..).unwrap_or(&[]).to_vec())]
        }
    }
}

/// Fit one finishing or spring ring on the air side (`air_left = Some(fin_climb)`),
/// preserving full G2/G3 arcs on circular walls while never cutting into material.
pub(super) fn fit_finish_ring(pass: &[Point2], fin_climb: bool, proof: &mut ProofTally) -> Vec<Seg> {
    let segs = fit_arcs_sided(pass, ARC_FIT_TOL, Some(fin_climb));
    proof.arcs += segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
    segs
}

/// Plan clearing passes inside `center_clear` using `p_clear` (which keeps `p.climb`).
pub(super) fn plan_clearing(
    region: &Region,
    center_clear: &Region,
    p_clear: &PocketParams,
    strategy: PocketStrategy,
    side: &StartSide,
    what: &str,
) -> Result<ClearPlan, GenError> {
    match strategy {
        PocketStrategy::Adaptive => {
            let (ps, notes, s, room, joins) = adaptive_passes_from(region, center_clear, p_clear, what, side)?;
            Ok((ps, notes, s, room, joins, Vec::new()))
        }
        PocketStrategy::Zigzag => {
            let ps = rest_passes(center_clear, p_clear.tiling_step(), p_clear.climb)?;
            let notes = if side.start != PassStart::Auto {
                vec![super::pass_start::PASS_START_ZIGZAG_NOTE.to_owned()]
            } else {
                Vec::new()
            };
            Ok((ps, notes, strategy, None, Vec::new(), Vec::new()))
        }
        s => {
            let (ps, notes, ramps) = ring_plan_ramped(region, center_clear, p_clear, s, side, true)?;
            Ok((ps, notes, s, None, Vec::new(), ramps))
        }
    }
}
