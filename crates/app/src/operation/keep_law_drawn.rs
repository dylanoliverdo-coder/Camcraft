//! Drawn-edge keep-law helpers for DXF-profile 2D Chamfers (Journeyman S10 follow-up).
//!
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! clean-room partition of kept parts for DXF-profile chamfers over separate solid parts,
//! delegating edge checks to `camcraft_sim::stock_sim::{beside_model, on_drawn_edges}`;
//! no vendor parameters, text or code.

use std::borrow::Cow;

use crate::geometry::Triangle;

use super::{OpParams, SrcRef};

/// Journeyman S10 follow-up: for a 2D Chamfer on a DXF profile (its
/// source a sketch, so not part-solid), the triangle count of each part
/// [`super::keep_law::parts_in_stock`] lays out, part by part, in its order
/// ([`super::keep_law::kept_parts_named`] reads the same models the same way);
/// `None` for every other operation.
#[must_use]
pub(super) fn drawn_part_sizes(
    world: &flecs_ecs::prelude::World,
    setup: Option<&str>,
    params: &OpParams,
    src: &SrcRef,
    part_solid: bool,
) -> Option<Vec<usize>> {
    if part_solid
        || !matches!(params, OpParams::Chamfer2D { .. })
        || crate::sketch_model::source_profiles(world, src).is_empty()
    {
        return None;
    }
    Some(
        super::keep_law::kept_parts_named(world, setup, params, src, part_solid)
            .iter()
            .map(|(_, f)| f.len())
            .collect(),
    )
}

/// Journeyman S10 follow-up: whether `part` is a profile chamfer's OWN
/// part - every one of its `breaks` stands on an edge of `part`
/// (`stock_sim::on_drawn_edges`: on a wall, and no part higher than the
/// drawn edge beside the tip; a record over air forgives nothing) and at
/// least one of them is drawn beside it. Such a part is judged as a part
/// source's own part is (`generate::depth_law_of` reads `None` for a 2D
/// Chamfer): in the program sim, its breaks credited (#2435). Every other
/// part keeps the solid law: a part the drawing never meets, as a part
/// source's neighbour does (#1621); a part a chain stands inside, or is
/// drawn below the edge of.
#[must_use]
pub(super) fn drawn_own_part(
    part: &[Triangle],
    breaks: &[crate::cl::EdgeBreak],
    excess: f64,
    file_tolerance: bool,
) -> bool {
    // The solid law's own tolerance (`generate::contain_into`): the sim's
    // `DEV_TOL` less the model's excess, the same way.
    let tol = crate::verify::excess_floor_for(super::material::GOUGE_TOL, excess, file_tolerance)
        .unwrap_or(super::material::GOUGE_TOL);
    !breaks.is_empty()
        && camcraft_sim::stock_sim::beside_model(part, breaks)
        && camcraft_sim::stock_sim::on_drawn_edges(part, tol, breaks.to_vec()).len() == breaks.len()
}

/// Journeyman S10 follow-up: of the kept parts `kept` - laid out part by
/// part, `sizes` triangles each ([`drawn_part_sizes`]) - the ones a 2D
/// Chamfer on a DXF profile still answers to the solid law for: all but
/// its own parts ([`drawn_own_part`]). `None` sizes (any other op), or
/// sizes that do not lay `kept` out exactly, spare nothing (the stricter
/// reading).
#[must_use]
pub(super) fn drawn_kept<'a>(
    kept: &'a [Triangle],
    sizes: Option<&[usize]>,
    breaks: &[crate::cl::EdgeBreak],
    excess: f64,
    file_tolerance: bool,
) -> Cow<'a, [Triangle]> {
    let Some(sizes) = sizes.filter(|s| s.iter().sum::<usize>() == kept.len()) else {
        return Cow::Borrowed(kept);
    };
    let (mut out, mut at, mut spared) = (Vec::new(), 0, false);
    for &n in sizes {
        let part = &kept[at..at + n];
        at += n;
        if drawn_own_part(part, breaks, excess, file_tolerance) {
            spared = true;
        } else {
            out.extend_from_slice(part);
        }
    }
    if spared { Cow::Owned(out) } else { Cow::Borrowed(kept) }
}
