//! The wall passes of one level (Journeyman spec S1): a contour rides
//! several offsets of its chain per level, the farthest first and the
//! final offset `w` last, with the finishing and spring passes beyond.
//! Provenance: Journeyman spec S1
//! (docs/journeyman/specs/S1-wall-passes.md); offset curves after
//! M. Held, *On the Computational Geometry of Pocket Machining*,
//! LNCS 500, Springer 1991, and R. T. Farouki and C. A. Neff, "Analytic
//! properties of plane offset curves", CAGD 7 (1990) 83-99; no vendor
//! parameters, text or code.

use crate::geo2d::{Loop, Point2, Region};

use super::contour::PreparedBand;
use super::contour_params::{ContourParams, Side};
use super::face::GenError;

/// The wall-pass controls of one 2D Contour: the roughing, finishing and
/// spring passes and the levels they run on (spec S1). The default is
/// today's single pass at the final offset, byte for byte.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallPasses {
    /// Roughing passes at each level; `1` is today's single pass.
    pub rough_passes: u32,
    /// Distance between two roughing passes (mm); read only when
    /// `rough_passes` > 1.
    pub rough_spacing: f64,
    /// Finishing passes after the roughing ones; `0` is none.
    pub finish_passes: u32,
    /// Distance between two finishing passes, and between the last
    /// roughing pass and the first finishing one (mm); read only when
    /// `finish_passes` > 0.
    pub finish_spacing: f64,
    /// Repeats of the final pass, at the final offset `w`.
    pub spring_passes: u32,
    /// The levels the finishing and spring passes run on.
    pub finish_level: FinishLevel,
}

impl Default for WallPasses {
    /// The single roughing pass every file before the option held.
    fn default() -> Self {
        WallPasses {
            rough_passes: 1,
            rough_spacing: 0.0,
            finish_passes: 0,
            finish_spacing: 0.0,
            spring_passes: 0,
            finish_level: FinishLevel::Every,
        }
    }
}

impl WallPasses {
    /// Is this the single roughing pass every file before the option
    /// held (spec S1, review round 4)? The default path is the pre-S1
    /// code, byte for byte.
    #[must_use]
    pub fn is_default(&self) -> bool {
        *self == WallPasses::default()
    }

    /// Why these wall passes cannot be honoured, by name; `None` when
    /// they can (spec S1, "Refusals").
    #[must_use]
    pub fn refusal(&self) -> Option<String> {
        if !(self.rough_spacing.is_finite() && self.finish_spacing.is_finite()) {
            return Some("parameters must be finite".into());
        }
        if self.rough_passes > 1 && self.rough_spacing <= 0.0 {
            return Some(format!(
                "roughing passes: {} passes need a positive spacing (set to {:.3} mm)",
                self.rough_passes, self.rough_spacing
            ));
        }
        if self.finish_passes > 0 && self.finish_spacing <= 0.0 {
            return Some(format!(
                "finishing passes: {} passes need a positive spacing (set to {:.3} mm)",
                self.finish_passes, self.finish_spacing
            ));
        }
        if self.rough_passes > MAX_WALL_PASSES
            || self.finish_passes > MAX_WALL_PASSES
            || self.spring_passes > MAX_WALL_PASSES
        {
            return Some(format!(
                "wall passes: at most {MAX_WALL_PASSES} roughing, {MAX_WALL_PASSES} finishing and \
                 {MAX_WALL_PASSES} spring passes are supported"
            ));
        }
        None
    }
}

/// The most passes of one kind a level may ask for: more cuts nothing a
/// smaller count did not.
pub const MAX_WALL_PASSES: u32 = 64;

/// The levels a contour's finishing and spring passes run on (spec S1).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FinishLevel {
    /// At every level: the roughing passes, then the finishing and
    /// spring passes, on every level. Every file before the option.
    #[default]
    Every,
    /// Only at the last level: the levels above it carry only the
    /// roughing passes.
    Last,
}

/// The last (lowest) cut level of `p`'s ladder, for the flute law.
pub fn contour_last_level(p: &ContourParams) -> Result<f64, super::face::GenError> {
    let zs = p.levels()?;
    zs.last()
        .copied()
        .ok_or_else(|| super::face::GenError::Invalid("the contour has no cut level".into()))
}

/// The finishing pass's flute law (spec S1, "Order and levels"): a
/// finishing pass cuts the whole wall height with the side of the tool,
/// so the tool's flute length must cover `top_z - bottom_z`. `Err`
/// names the shortfall.
pub fn contour_flute_covers_wall(p: &ContourParams, flute_len: f64) -> Result<(), super::face::GenError> {
    let wall = p.top_z - p.bottom_z;
    if !(flute_len.is_finite() && flute_len > 0.0) {
        return Err(super::face::GenError::Invalid(
            "the tool states no flute length, so the wall height a finishing pass cuts is unknown".into(),
        ));
    }
    if flute_len + 1e-9 < wall {
        return Err(super::face::GenError::Invalid(format!(
            "finishing pass: the wall is {wall:.3} mm tall but the tool's flute is {flute_len:.3} mm long - \
             a finishing pass at the last level cuts the whole wall with the side of the tool; use a tool \
             with a flute at least {wall:.3} mm long"
        )));
    }
    Ok(())
}

/// One pass of a level, in cut order: its offset from the wall (the
/// tool centre's distance) and whether it is a finishing pass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WallPass {
    /// The tool centre's distance from the wall, mm (`w + d`).
    pub offset: f64,
    /// `true` for a finishing or spring pass: it runs only where
    /// `finish_level` says, and reads the flute law.
    pub finishing: bool,
}

/// The tool centre's distance from the wall at the final offset `w`:
/// the tool radius plus the stock to leave.
pub(crate) fn final_offset(p: &ContourParams) -> f64 {
    p.tool_diameter * 0.5 + p.stock_to_leave
}

/// Every pass of one level, in cut order (spec S1 "Offsets"): the
/// roughing passes farthest first, then the finishing passes, then the
/// spring passes at the final offset. The level filter is applied by the
/// caller from [`level_runs_finishing`].
pub(crate) fn level_passes(p: &ContourParams) -> Vec<WallPass> {
    let w = final_offset(p);
    let wp = p.passes;
    let (nr, dr, nf, df, ns) = (
        wp.rough_passes,
        wp.rough_spacing,
        wp.finish_passes,
        wp.finish_spacing,
        wp.spring_passes,
    );
    let mut out = Vec::with_capacity(nr as usize + nf as usize + ns as usize);
    for i in 1..=nr {
        out.push(WallPass { offset: w + nf as f64 * df + (nr - i) as f64 * dr, finishing: false });
    }
    for j in 1..=nf {
        out.push(WallPass { offset: w + (nf - j) as f64 * df, finishing: true });
    }
    for _ in 0..ns {
        out.push(WallPass { offset: w, finishing: true });
    }
    out
}

/// Whether `z` (one of `levels`, descending) runs the finishing and
/// spring passes: every level, or only the last one (the lowest).
pub(crate) fn level_runs_finishing(p: &ContourParams, levels: &[f64], z: f64) -> bool {
    match p.passes.finish_level {
        FinishLevel::Every => true,
        FinishLevel::Last => levels.last().is_some_and(|&last| z.to_bits() == last.to_bits()),
    }
}

/// The band one pass rides (Journeyman spec S1): `band` itself for
/// the final offset, the base region re-offset for any other pass. A
/// pass whose offset collapses the region names itself in `notes` and
/// yields an empty band - except the final offset, which keeps the
/// existing empty-contour refusal.
pub(crate) fn prepare_pass(
    band: &PreparedBand,
    p: &ContourParams,
    pass: WallPass,
    notes: &mut Vec<String>,
) -> Result<PreparedBand, GenError> {
    if (pass.offset - band.offset).abs() <= 1e-12 {
        return Ok(PreparedBand {
            base: band.base.clone(),
            offset: band.offset,
            center: band.center.clone(),
            prog: band.prog.clone(),
            loops: band.loops.clone(),
            plan: band.plan.clone(),
            room: band.room.clone(),
        });
    }
    let inside = p.side == Side::Inside;
    let d = pass.offset;
    let center = band.base.offset(if inside { -d } else { d })?;
    if center.is_empty() {
        if (d - final_offset(p)).abs() <= 1e-12 {
            return Err(GenError::Empty("offset boundary collapsed".into()));
        }
        notes.push(format!(
            "contour: a wall pass at {:.3} mm from the wall does not fit this chain - the material there is narrower than the pass; the pass is dropped",
            d
        ));
        return Ok(PreparedBand {
            base: band.base.clone(),
            offset: d,
            center: Region::empty(),
            prog: None,
            loops: Vec::new(),
            plan: Vec::new(),
            room: None,
        });
    }
    let loops: Vec<Loop> = {
        let mut v: Vec<Loop> = center.loops().to_vec();
        v.sort_by(|a, b| b.area().abs().total_cmp(&a.area().abs()));
        v
    };
    let plan = loops
        .iter()
        .map(|l| {
            let mut v: Vec<Point2> = super::ring::walked(l.points(), p.climb == inside, inside);
            if v.len() > 1 {
                let (first, last) = (v[0], v[v.len() - 1]);
                if first.x != last.x || first.y != last.y {
                    v.push(first);
                }
            }
            v
        })
        .collect();
    Ok(PreparedBand { base: band.base.clone(), offset: d, center, prog: None, loops, plan, room: None })
}

/// Map each base band loop to the pass band loop that carries it
/// (Journeyman spec S1, review round 3): a pass offset can merge or
/// split loops, so a schedule index built on the base band may not
/// index the pass band. For an Outside contour (the region grows) the
/// base loop sits inside the pass loop that contains it; for an Inside
/// contour (the region erodes) the pass loop sits inside the base loop.
/// Returns, for each base loop index, the pass loop index that carries
/// it, or `None` when no pass loop does (the pass drops that loop).
pub(crate) fn map_loops(base: &[Loop], pass: &[Loop], inside: bool) -> Vec<Option<usize>> {
    let inside_loop = |outer: &Loop, q: Point2| -> bool {
        // Ray parity to +x with the half-open edge rule (the same law
        // `Loop::contains_xy` uses), on one loop.
        let p = outer.points();
        let mut hit = false;
        let n = p.len();
        for i in 0..n {
            let (a, b) = (p[i], p[(i + 1) % n]);
            let (ylo, yhi) = if a.y <= b.y { (a.y, b.y) } else { (b.y, a.y) };
            if q.y >= ylo && q.y < yhi {
                let x_at = a.x + (q.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if q.x < x_at {
                    hit = !hit;
                }
            }
        }
        hit
    };
    // A loop's representative interior point: its centroid nudged from
    // the first vertex toward the centroid, so it is strictly inside
    // the loop (the centroid itself may fall in a notch).
    let inner = |l: &Loop| -> Point2 {
        let p = l.points();
        let (mut cx, mut cy) = (0.0, 0.0);
        for q in p {
            cx += q.x;
            cy += q.y;
        }
        let n = p.len().max(1) as f64;
        let c = Point2::new(cx / n, cy / n);
        let v = p[0];
        Point2::new(v.x + (c.x - v.x) * 0.5, v.y + (c.y - v.y) * 0.5)
    };
    base.iter()
        .map(|b| {
            // A base outer loop (CCW) is carried by the pass loop that
            // contains it when the region grows (Outside), or that it
            // contains when the region erodes (Inside); a base hole loop
            // (CW) is the other way round.
            let b_outer = b.is_ccw();
            let a = inner(b);
            pass.iter()
                .position(|p| {
                    let grows = b_outer != inside;
                    if grows {
                        inside_loop(p, a)
                    } else {
                        inside_loop(b, inner(p))
                    }
                })
        })
        .collect()
}

/// The note naming a merged pass loop (spec S1, review round 3): the
/// pass loop `pli` carries `carriers` base loops; `None` when it does
/// not merge.
pub(crate) fn merged_note(map: &[Option<usize>], pli: usize, offset: f64) -> Option<String> {
    let carriers = map.iter().filter(|&&m| m == Some(pli)).count();
    (carriers > 1).then(|| {
        format!(
            "contour: a wall pass at {:.3} mm from the wall merges {} rings into one loop - the merged loop is cut once",
            offset, carriers
        )
    })
}

pub(crate) use super::contour_open_passes::open_pass_runs;

/// The passes of level `z` of `levels`, after the level filter.
pub(crate) fn passes_at_level(p: &ContourParams, levels: &[f64], z: f64) -> Vec<WallPass> {
    if level_runs_finishing(p, levels, z) {
        level_passes(p)
    } else {
        let w = final_offset(p);
        let wp = p.passes;
        (1..=wp.rough_passes)
            .map(|i| WallPass {
                offset: w + wp.finish_passes as f64 * wp.finish_spacing + (wp.rough_passes - i) as f64 * wp.rough_spacing,
                finishing: false,
            })
            .collect()
    }
}
