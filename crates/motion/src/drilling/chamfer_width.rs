//! Journeyman S6 (docs/journeyman/specs/S6-hole-chamfer-width.md); cone
//! geometry after Smid 2007; no vendor parameters, text or code.

use super::hole::{CycleParams, Hole};
use crate::tools::{Tool, ToolKind};

/// The width's command-time rule, also checked by the motion entry point.
#[must_use]
pub fn chamfer_width_refusal(width: f64) -> Option<&'static str> {
    (!(width.is_finite() && width >= 0.0)).then_some("Chamfer width must be finite and non-negative")
}

impl Hole {
    /// A chamfer may also start from a point location with no bore diameter.
    #[must_use]
    pub fn is_usable_for_chamfer(&self) -> bool {
        self.all_finite() && self.radius >= 0.0 && self.floor_z < self.entry_z
    }
}

fn cone_depth(
    hole_diameter: f64,
    width: f64,
    body_diameter: f64,
    angle: f64,
    tip_diameter: f64,
) -> Result<f64, String> {
    if !angle.is_finite() || angle <= 0.0 || angle >= 180.0 {
        return Err(format!("the tool's included cone angle {angle} must be between 0 and 180 degrees"));
    }

    if !tip_diameter.is_finite() || tip_diameter < 0.0 || tip_diameter >= body_diameter {
        return Err(format!("tip diameter {tip_diameter} mm must be non-negative and smaller than the tool body"));
    }
    let diameter = hole_diameter + 2.0 * width;
    if diameter > body_diameter {
        return Err(format!("chamfer diameter {diameter:.3} mm exceeds the tool body diameter {body_diameter:.3} mm"));
    }
    if diameter <= tip_diameter {
        return Err(format!("chamfer diameter {diameter:.3} mm does not exceed the tip diameter {tip_diameter:.3} mm"));
    }

    let depth = (diameter - tip_diameter) / (2.0 * (angle.to_radians() * 0.5).tan());
    if !depth.is_finite() || depth <= 0.0 {
        return Err("the tool cone gives no usable tip depth".into());
    }
    Ok(depth)
}

/// The programmed tip position for a hole with a positive chamfer width.
pub fn chamfer_tip_z(hole: &Hole, tool: &Tool, width: f64) -> Result<f64, String> {
    if !matches!(tool.kind, ToolKind::Drill | ToolKind::Taper) {
        return Err("the selected tool has no cutting cone".into());
    }
    let depth = cone_depth(2.0 * hole.radius, width, tool.diameter, tool.tip_angle, tool.tip_diameter)?;
    let bottom = hole.entry_z - depth;
    if !bottom.is_finite() || bottom >= hole.entry_z {
        return Err("the calculated tip depth is unusable".into());
    }
    Ok(bottom)
}

pub(super) fn plan_hole(hole: Hole, params: &CycleParams, notes: &mut Vec<String>) -> Option<(Hole, Option<f64>)> {
    if params.chamfer_width == 0.0 {
        return Some((hole, None));
    }
    match chamfer_tip_z(&hole, params.tool, params.chamfer_width) {
        Ok(z) => Some((hole, Some(z))),
        Err(why) => {
            notes.push(format!("Chamfer width: hole at ({:.3}, {:.3}) skipped: {why}", hole.xy.x, hole.xy.y));
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::cone_depth;

    #[test]
    fn depth_accounts_for_a_truncated_tip() {
        let depth = cone_depth(6.6, 0.5, 12.0, 90.0, 0.5).expect("a 7.6 mm chamfer fits");
        assert!((depth - 3.55).abs() < 1e-9);
    }

    #[test]
    fn cone_that_cannot_cut_the_requested_diameter_is_refused() {
        assert!(cone_depth(11.5, 0.5, 12.0, 90.0, 0.0).expect_err("oversize")
            .contains("exceeds the tool body diameter"));
        assert!(cone_depth(0.0, 0.25, 12.0, 90.0, 0.5).expect_err("too narrow")
            .contains("does not exceed the tip diameter"));
        for angle in [0.0, 180.0, f64::NAN] {
            assert!(cone_depth(6.6, 0.5, 12.0, angle, 0.0).expect_err("bad angle")
                .contains("included cone angle"));
        }
    }
}
