//! Journeyman spec S10: 2D Chamfer on the chains of a DXF sketch profile
//! (closed and open).
//!
//! Provenance: Journeyman spec S10 (docs/journeyman/specs/S10-chamfer-on-sketch-chains.md);
//! machine-tool handbook chamfer-mill cone geometry (*Machinery's Handbook*, 31st ed.,
//! Industrial Press 2020) and P. Smid, *CNC Programming Handbook*, 3rd ed., 2007;
//! no vendor parameters, text or code.

use flecs_ecs::prelude::World;

use crate::cl::Toolpath;
use crate::geo2d::{Loop, Point2, Region};
use crate::machining2d::{self, ChamferClosedChain, ChamferParams, GenError, Side};

use super::contour_sketch::{ResolvedSketchChain, SketchContourPlan};
use super::contour_sketch_pick::collect_chains;
use super::{OpParams, OpenChainSide, SrcRef};

/// True when `params` is a `2D Chamfer` and `src` names at least one DXF sketch profile.
#[must_use]
pub(crate) fn is_sketch_chamfer(world: &World, src: &SrcRef, params: &OpParams) -> bool {
    matches!(params, OpParams::Chamfer2D { .. })
        && !crate::sketch_model::source_profiles(world, src).is_empty()
}

/// Resolve the DXF chains selected for a `2D Chamfer` operation.
///
/// Closed chains use Contour's side rules (per-chain `side` override, else
/// operation `Side`, else `Auto` nesting where Outer -> boss and Hole -> bore).
/// Open chains require `open_side` (`left` or `right`); `none` is refused by name.
pub(super) fn resolve_plan(
    world: &World,
    src: &SrcRef,
    params: &OpParams,
) -> Result<SketchContourPlan, GenError> {
    let OpParams::Chamfer2D {
        tolerance,
        side,
        climb,
        chain_overrides,
        ..
    } = params
    else {
        return Err(GenError::Invalid("not a 2D Chamfer operation".into()));
    };
    let mut items = collect_chains(world, src, *tolerance, *side, *climb, chain_overrides)
        .map_err(|e| match e {
            GenError::Empty(_) => {
                let profiles = crate::sketch_model::source_profiles(world, src);
                let who = profiles
                    .first()
                    .map_or_else(|| "the DXF profile".to_owned(), |n| format!("DXF profile '{n}'"));
                GenError::Empty(format!("{who} contains no cuttable closed or open chains to chamfer"))
            }
            other => other,
        })?;

    let mut region_loops = Vec::new();
    let mut closed_boss = 0usize;
    let mut closed_bore = 0usize;
    let mut open_count = 0usize;
    for item in &mut items {
        if item.closed {
            region_loops.push(Loop::new(item.pts.clone())?);
            match item.eff_side {
                Some(Side::Inside) => closed_bore += 1,
                _ => closed_boss += 1,
            }
        } else {
            // Journeyman spec S10: an open chain on a 2D Chamfer always needs
            // `open_side` (left or right) to know which side holds the material.
            // Override Contour's `op_side != Auto` wording unless a per-chain
            // closed-chain `side` override was explicitly set on this open chain.
            let has_closed_side_ov = chain_overrides
                .iter()
                .any(|o| o.entity == item.entity && o.chain == item.chain_id && o.side.is_some());
            if item.open_side == OpenChainSide::None && !has_closed_side_ov {
                item.open_side_err = Some(format!(
                    "{} is an open sketch chain with open_side 'none' - a 2D Chamfer on an open edge needs the side of the material; set open_side to 'left' or 'right'",
                    item.label
                ));
            }
            open_count += 1;
        }
    }

    let mut notes = Vec::new();
    if closed_boss + closed_bore + open_count > 0 {
        notes.push(format!(
            "DXF sketch chamfer: {closed_boss} boss, {closed_bore} bore, {open_count} open"
        ));
    }
    Ok(SketchContourPlan {
        items,
        region_loops,
        notes,
    })
}

fn validate_plan(plan: &SketchContourPlan) -> Result<(), GenError> {
    for item in &plan.items {
        if let Some(why) = &item.open_side_err {
            return Err(GenError::Invalid(why.clone()));
        }
        if item.closed
            && item.eff_side.is_none()
            && let Some(why) = &item.auto_role_err
        {
            return Err(GenError::Invalid(why.clone()));
        }
    }
    Ok(())
}

/// Summary notes + early-door refusal for UI property pages (`selection_memo`).
pub(super) fn account(
    world: &World,
    src: &SrcRef,
    params: &OpParams,
) -> (Vec<String>, Option<String>, Option<bool>) {
    match resolve_plan(world, src, params) {
        Ok(plan) => {
            let refusal = validate_plan(&plan).err().map(|e| e.to_string());
            let mut closed = plan.items.iter().filter(|i| i.closed);
            let inside = closed.next().and_then(|first| {
                let s = first.eff_side?;
                closed.all(|i| i.eff_side == Some(s)).then_some(s == Side::Inside)
            });
            (plan.notes, refusal, inside)
        }
        Err(e) => (Vec::new(), Some(e.to_string()), None),
    }
}

fn orient_open_for_chamfer(item: &ResolvedSketchChain, op_climb: bool) -> Result<Vec<Point2>, GenError> {
    let dir_pts: Vec<Point2> = if item.reversed {
        item.pts.iter().copied().rev().collect()
    } else {
        item.pts.clone()
    };
    let left = match item.open_side {
        OpenChainSide::Left => true,
        OpenChainSide::Right => false,
        OpenChainSide::None => {
            return Err(GenError::Invalid(format!(
                "{} is an open sketch chain with open_side 'none' - set open_side to 'left' or 'right'",
                item.label
            )));
        }
    };
    // Cutter body stands on `open_side`, so material stands on the opposite
    // side: right of `dir_pts` when cutter is `left`, left of `dir_pts` when
    // cutter is `right`. Climb under M3 keeps material on the right of travel;
    // conventional reverses it.
    let forward = left == op_climb;
    if forward {
        Ok(dir_pts)
    } else {
        Ok(dir_pts.into_iter().rev().collect())
    }
}

/// Generate the `2D Chamfer` toolpath for a validated DXF sketch profile plan.
pub(super) fn generate_plan(
    plan: SketchContourPlan,
    cp: &ChamferParams,
    op_side: Side,
    op_climb: bool,
) -> Result<(Region, Toolpath), GenError> {
    validate_plan(&plan)?;
    let verify_region = if plan.region_loops.is_empty() {
        Region::empty()
    } else {
        Region::new(plan.region_loops.clone())?
    };

    let mut closed = Vec::new();
    let mut open = Vec::new();
    for item in &plan.items {
        if item.closed {
            let eff_side = item.eff_side.unwrap_or(match op_side {
                Side::Inside => Side::Inside,
                _ => Side::Outside,
            });
            closed.push(ChamferClosedChain {
                loop_: Loop::new(item.pts.clone())?,
                inside: eff_side == Side::Inside,
                climb: item.eff_climb,
            });
        } else {
            open.push(orient_open_for_chamfer(item, op_climb)?);
        }
    }

    let mut path = machining2d::chamfer_chains(&closed, &open, cp)?;
    for n in plan.notes {
        if !path.notes.contains(&n) {
            path.notes.push(n);
        }
    }
    Ok((verify_region, path))
}
