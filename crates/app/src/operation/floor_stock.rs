//! Provenance: Journeyman spec S2, signed offset along the operation's tool axis.

use super::OpParams;

impl OpParams {
    pub(crate) fn floor_stock(&self) -> Option<f64> {
        match self {
            Self::Face { floor_stock, .. } | Self::Contour { floor_stock, .. } | Self::Pocket { floor_stock, .. } => Some(*floor_stock),
            _ => None,
        }
    }
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
