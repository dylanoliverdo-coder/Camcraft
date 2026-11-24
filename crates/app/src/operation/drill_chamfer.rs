//! Journeyman S6 (docs/journeyman/specs/S6-hole-chamfer-width.md); cone
//! geometry after Smid 2007; no vendor parameters, text or code.

use crate::cl::lattice;
use crate::drilling;
use crate::geo2d::Point2;
use crate::tools::{Tool, ToolKind};
use crate::verify::HoleChamfer;

use super::OpParams;

pub(super) struct DrillProbe {
    pub tip_radius: f64,
    pub chamfers: Vec<HoleChamfer>,
}

impl DrillProbe {
    pub(super) fn of(params: &OpParams, tool: &Tool) -> Option<Self> {
        let OpParams::Drill { chamfer_width, holes, cross, .. } = params else { return None };
        if !cross.is_empty() || !matches!(tool.kind, ToolKind::Drill | ToolKind::Taper)
            || (*chamfer_width == 0.0 && tool.tip_diameter == 0.0)
        {
            return None;
        }
        let chamfers = if *chamfer_width > 0.0 {
            holes.iter().filter_map(|hole| {
                // Match the drill kernel's geometry lattice, not the
                // actual path under verification.
                let hole = snapped(*hole);
                let bottom = drilling::chamfer_tip_z(&hole, tool, *chamfer_width).ok()?;
                HoleChamfer::new(hole.xy, hole.entry_z, hole.radius, *chamfer_width, bottom)
            }).collect()
        } else {
            Vec::new()
        };
        Some(Self { tip_radius: tool.tip_diameter * 0.5, chamfers })
    }

    pub(super) fn verdict(&self) -> (f64, &[HoleChamfer]) {
        (self.tip_radius, &self.chamfers)
    }
}


fn snapped(hole: drilling::Hole) -> drilling::Hole {
    drilling::Hole {
        xy: Point2::new(lattice(hole.xy.x), lattice(hole.xy.y)),
        radius: lattice(hole.radius),
        entry_z: lattice(hole.entry_z),
        ..hole
    }
}

/// The document sim reuses its bounded mark sweep. Rebuild the nominal
/// plunges from the declaration, never from the cache that is judged.
/// Only the source part gets this credit, capped at the chamfer's own
/// axial depth, and the ordinary mark law keeps the part's underside.
pub(crate) fn sim_mark(
    world: &flecs_ecs::prelude::World,
    op: &crate::schema::Operation,
    tool: &Tool,
    warnings: &[crate::verify::Warning],
) -> Option<super::mark_law::SimMark> {
    let OpParams::Drill { chamfer_width, holes, cross, .. } = &op.params else { return None };
    if *chamfer_width <= 0.0 || !cross.is_empty()
        || warnings.iter().any(crate::verify::Warning::blocks_post)
    {
        return None;
    }
    // A stock-sourced chamfer gets no credit over an independent kept
    // part, just like the existing stock-edge break law.
    let target = super::mark_law::target_facets(world, op)?;
    let budget = *chamfer_width / (tool.tip_angle.to_radians() * 0.5).tan();
    if target.is_empty() || !(budget.is_finite() && budget > 0.0) {
        return None;
    }
    let moves: Vec<_> = holes.iter().copied().map(snapped).filter_map(|hole| {
        use crate::{cl::{Move, MoveKind}, geometry::Point3};
        let bottom = drilling::chamfer_tip_z(&hole, tool, *chamfer_width).ok()?;
        Some(Move::new(
            MoveKind::Plunge,
            Point3::new(hole.xy.x, hole.xy.y, hole.entry_z),
            Point3::new(hole.xy.x, hole.xy.y, bottom),
            None,
        ))
    }).collect();
    (!moves.is_empty()).then_some(super::mark_law::SimMark {
        budget, target: Some(target), nominal_moves: Some(moves),
    })
}
