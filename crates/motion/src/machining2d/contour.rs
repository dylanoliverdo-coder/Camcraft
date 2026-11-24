//! Offset contouring (split from `machining2d.rs` by M5.0.1); M11.1 B
//! adds the optional corner fillets - provenance in `fillet.rs`
//! (tangent circle to two lines, RS-274 / ISO 6983 G2/G3); M11.2 C/D the
//! user's level transitions and links (`contour_link`, `links.rs`).

use crate::cl::{PathBuilder, Toolpath};
use crate::geo2d::{Loop, Point2, Region};

use super::contour_comp;
use super::contour_depth::Ladder;
use super::contour_emit::emit_passes;
use super::contour_step::Said;
use super::face::GenError;
use super::fillet_tally::{FilletTally, fillet_floor};

pub use super::contour_params::{ContourParams, Lead, Side};
pub use super::contour_region::{contour, contour_on_bodies, contour_on_part, contour_on_picked};

/// Offset loops, programmed region and walking polylines prepared for one
/// contour band; shared with `contour_whole`.
pub(super) struct PreparedBand {
    /// The material cross-section this band was cut from: the base every
    /// pass's own offset is taken from (spec S1).
    pub base: Region,
    /// The tool-centre distance from the wall this band's loops ride
    /// (spec S1): the final offset `w`, or one pass's own offset.
    pub offset: f64,
    pub center: Region,
    pub prog: Option<Region>,
    pub loops: Vec<Loop>,
    pub plan: Vec<Vec<Point2>>,
    /// The lead room's region when it is not `center`: `center` with the
    /// rings that another band cuts on a shared level fenced off (#2677).
    pub room: Option<Region>,
}

pub(super) fn prepare_band(region: &Region, p: &ContourParams) -> Result<PreparedBand, GenError> {
    if region.is_empty() {
        return Err(GenError::Empty("empty footprint".into()));
    }
    let r = p.tool_diameter * 0.5;
    let inside = p.side == Side::Inside;
    let d = r + p.stock_to_leave;
    let center = region.offset(if inside { -d } else { d })?;
    if center.is_empty() {
        return Err(GenError::Empty(if inside {
            "the bore is narrower than the tool - switch Side to Outside or pick a smaller tool"
                .into()
        } else {
            "offset boundary collapsed".into()
        }));
    }
    // The contour the post programs under G41/G42 (M13.11): the part
    // at the stock to leave; `None` for every path cut in computer.
    let prog = contour_comp::programmed_region(region, p)?;
    let mut loops: Vec<Loop> = center.loops().to_vec();
    loops.sort_by(|a, b| b.area().abs().total_cmp(&a.area().abs()));
    // The rings every level plans to walk, as closed polylines - the
    // link chords of M8.1's plan law get proven against these. Tabbed
    // levels walk a subset, which errs conservative: an unproven chord
    // hops rather than dragging across a ring line.
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
    Ok(PreparedBand { base: region.clone(), offset: d, center, prog, loops, plan, room: None })
}

/// Walk one band: every ring at every level, and at each level the
/// wall passes of spec S1 in cut order. The loop itself is
/// `contour_passes::emit_passes`; this is the one call that keeps
/// `contour.rs` at the band's own concern.
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_band(
    b: &mut PathBuilder,
    band: &PreparedBand,
    p: &ContourParams,
    lad: &Ladder,
    pick: Option<(usize, Point2)>,
    exit: Option<(usize, Point2)>,
    said: &mut Said,
    lead_notes: &mut Vec<String>,
    tally: &mut FilletTally,
) -> Result<(), GenError> {
    emit_passes(b, band, p, lad, pick, exit, said, lead_notes, tally)
}

pub(super) fn finish_contour(
    mut b: PathBuilder,
    lead_notes: Vec<String>,
    tally: &FilletTally,
    p: &ContourParams,
) -> Result<Toolpath, GenError> {
    for n in lead_notes {
        b.note(n);
    }
    for n in tally.notes("walked", fillet_floor(p.tolerance)) {
        b.note(n);
    }

    let path = b.finish();
    if let Some(why) = crate::approach::refusal(&path) {
        // #261 part 3: the moves built so far ride the refusal, display-only.
        return Err(GenError::Invalid(why).with_ghost(&path));
    }
    if !path.all_finite() {
        return Err(GenError::Invalid("path left the finite horizon".into()));
    }
    Ok(path)
}


/// Which side of travel is air: climb travel (M3, #1555) keeps the
/// material on the right, so air is left; conventional flips both.
pub(super) fn air_side(t: Point2, climb: bool) -> Point2 {
    if climb {
        Point2::new(-t.y, t.x)
    } else {
        Point2::new(t.y, -t.x)
    }
}

pub(super) fn unit(dx: f64, dy: f64) -> Point2 {
    let m = (dx * dx + dy * dy).sqrt();
    if m == 0.0 || !m.is_finite() {
        Point2::new(1.0, 0.0)
    } else {
        Point2::new(dx / m, dy / m)
    }
}
