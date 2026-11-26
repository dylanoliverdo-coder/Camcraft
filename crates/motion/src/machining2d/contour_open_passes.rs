//! Wall-pass orchestration for open contour runs (Journeyman specs S1 and S9).
//!
//! Provenance: Journeyman specs S1 (docs/journeyman/specs/S1-wall-passes.md)
//! and S9 (docs/journeyman/specs/S9-open-sketch-chain-side.md); split from
//! `contour_passes.rs` to stay under the 400-line law; no vendor parameters,
//! text or code.

use crate::geo2d::Point2;

use super::contour_params::ContourParams;
use super::contour_passes::{final_offset, passes_at_level};
use super::face::GenError;

/// The wall passes of an open run, level by level (Journeyman spec
/// S1, review round 3): the passes of each level in cut order, then the
/// next level, finishing at the last level when `finish_level` is Last.
/// `offset` maps a pass's tool-centre distance to its ride: `Ok(None)`
/// when the offset curve does not exist (the pass is named and
/// dropped), `Err` for every other refusal (propagated). A level whose
/// every pass is dropped is an error, never an empty `Ok`.
pub(crate) fn open_pass_runs(
    p: &ContourParams,
    mut offset: impl FnMut(f64) -> Result<Option<Vec<Point2>>, GenError>,
) -> Result<crate::cl::Toolpath, GenError> {
    let zs = p.levels()?;
    let mut path = crate::cl::Toolpath::default();
    let mut first = true;
    let mut level_cut = false;
    for &z in &zs {
        let mut level_any = false;
        for pass in passes_at_level(p, &zs, z) {
            let d = pass.offset;
            if !(d.is_finite() && d > 1e-3) {
                return Err(GenError::Invalid(format!(
                    "the open run's offset (tool radius plus stock to leave) is {d:.3} mm - it must be positive"
                )));
            }
            let ride = match offset(d)? {
                Some(r) => r,
                None => {
                    // The final pass (w) failing refuses the run with the
                    // pre-S1 offset_run fold error; any other pass is
                    // named and dropped.
                    if (d - final_offset(p)).abs() <= 1e-9 {
                        return Err(GenError::Invalid(format!(
                            "the open run's offset of {d:.3} mm folds onto itself (the run comes back within twice the offset of \
                             itself) - pick a gentler run or a smaller tool"
                        )));
                    }
                    path.notes.push(format!(
                        "contour: a wall pass at {:.3} mm from the open run does not fit - the pass is dropped",
                        d
                    ));
                    continue;
                }
            };
            let seg = super::contour_along_chain_zs(&ride, p, std::slice::from_ref(&z))?;
            if first {
                // Keep the notes naming passes dropped before the first one
                // that fits (Journeyman spec S9 review round 2).
                let dropped = std::mem::take(&mut path.notes);
                path = seg;
                path.notes.splice(0..0, dropped);
                first = false;
            } else {
                // The segment opens with a rapid from the seed (0, 0,
                // clearance); re-anchor it on the actual current
                // position so the concatenated path stays linked.
                let mut moves = seg.moves;
                if let Some(first_move) = moves.first_mut() {
                    let cur = path.moves.last().map(|m| m.to).unwrap_or(first_move.from);
                    first_move.from = cur;
                }
                path.moves.extend(moves);
                path.notes.extend(seg.notes);
            }
            level_any = true;
        }
        if !level_any {
            // Every pass of this level was dropped: nothing is cut here.
            return Err(GenError::Empty(format!(
                "the open run has no wall pass that fits at Z {z:.3} - every pass was dropped"
            )));
        }
        level_cut = true;
    }
    if !level_cut {
        return Err(GenError::Empty("the open run has no cut level".into()));
    }
    Ok(path)
}
