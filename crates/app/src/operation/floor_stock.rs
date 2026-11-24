//! Provenance: Journeyman spec S2 (`docs/journeyman/specs/S2-floor-allowance.md`);
//! P. Smid, *CNC Programming Handbook* (3rd ed., Industrial Press 2007);
//! no vendor parameters, text or code.

use std::hash::Hasher;

use flecs_ecs::prelude::World;

use super::{fold_height, OpFrame, OpParams, SrcRef};

impl OpParams {
    /// Signed axial allowance on the floor, mm (`Face`, `Contour`, `Pocket`).
    #[must_use]
    pub fn floor_stock(&self) -> Option<f64> {
        match self {
            Self::Face { floor_stock, .. }
            | Self::Contour { floor_stock, .. }
            | Self::Pocket { floor_stock, .. } => Some(*floor_stock),
            _ => None,
        }
    }

    pub(crate) fn floor_stock_refusal(&self) -> Option<String> {
        self.floor_stock()
            .filter(|stock| !stock.is_finite())
            .map(|_| "Floor stock must be finite".into())
    }
}

/// Shift a resolved bottom/cut coordinate along the operation's tool axis by `floor_stock`.
pub(super) fn shift_bottom(params: &OpParams, bottom: f64) -> Option<f64> {
    let Some(stock) = params.floor_stock().filter(|stock| *stock != 0.0) else {
        return Some(bottom);
    };
    if !stock.is_finite() {
        return None;
    }
    let cut = bottom + stock;
    cut.is_finite().then_some(cut)
}

pub(super) fn offset(params: &OpParams, bottom: Option<f64>, top: Option<f64>) -> Result<Option<f64>, String> {
    let Some(stock) = params.floor_stock().filter(|stock| *stock != 0.0) else { return Ok(bottom) };
    if !stock.is_finite() {
        return Err("Floor stock must be finite".into());
    }
    let cut = bottom.map(|z| z + stock).filter(|z| z.is_finite())
        .ok_or_else(|| "Floor stock must give a finite cut level".to_owned())?;
    if top.is_some_and(|z| cut >= z) {
        return Err("Floor stock puts the last cut at or above Top Z".into());
    }
    Ok(Some(cut))
}

/// Command-door and `depth_sane` refusal for `floor_stock`. Unresolved non-selection height
/// references stand down here (matching `floor_stock == 0.0`) so generation refuses them by name.
pub(crate) fn floor_stock_depth_refusal(
    world: &World,
    setup: Option<&str>,
    src: Option<&SrcRef>,
    params: &OpParams,
) -> Option<String> {
    let stock = params.floor_stock().filter(|s| *s != 0.0)?;
    if !stock.is_finite() {
        return Some("Floor stock must be finite".into());
    }
    let Ok(frame) = OpFrame::new(setup.map(str::to_owned), params.tilt()).checked_frame() else {
        return None;
    };
    let (top_h, bottom_h) = match params {
        OpParams::Face { top_z, cut_z, .. } => (*top_z, *cut_z),
        OpParams::Contour { top_z, bottom_z, .. } | OpParams::Pocket { top_z, bottom_z, .. } => {
            (Some(*top_z), *bottom_z)
        }
        _ => return None,
    };
    let bottom = fold_height(world, setup, src, frame.as_ref(), bottom_h)?;
    let cut = bottom + stock;
    if !cut.is_finite() {
        return Some("Floor stock must give a finite cut level".into());
    }
    let top = top_h.and_then(|h| fold_height(world, setup, src, frame.as_ref(), h));
    if top.is_some_and(|z| cut >= z) {
        return Some("Floor stock puts the last cut at or above Top Z".into());
    }
    None
}

/// Write nonzero `floor_stock` into the operation digest; zero leaves legacy stamps unchanged.
pub(super) fn write_stamp(params: &OpParams, h: &mut std::hash::DefaultHasher) {
    if let Some(stock) = params.floor_stock().filter(|stock| *stock != 0.0) {
        h.write(b"floor_stock");
        h.write_u64(stock.to_bits());
    }
}
