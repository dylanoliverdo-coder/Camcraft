//! Resolve and protect an authored straight entry on a DXF contour.
//! Provenance: our existing sketch selection and compensated contour laws.

use super::contour_sketch::ResolvedSketchChain;
use super::contour_sketch_pick::transform_chain_pts;
use super::wcs_yaw::Rot2;
use crate::cl::{MoveKind, Toolpath};
use crate::geo2d::{Loop, Point2, Region};
use crate::machining2d::{
    ContourParams, GenError, Side, StraightContourRamp, straight_contour_ramp,
};
use crate::schema::Location;

pub(super) fn from_sketch(
    mut ramp: StraightContourRamp,
    pose: Location,
    label: &str,
) -> Result<StraightContourRamp, GenError> {
    let [x, y] = ramp.start_xy;
    let [dx, dy] = ramp.direction_xy;
    let points = transform_chain_pts(
        pose,
        &[
            Point2::new(x, y),
            Point2::new(x + dx, y + dy),
            Point2::new(x - dy, y + dx),
        ],
    )?;
    let x = points[1].x - points[0].x;
    let y = points[1].y - points[0].y;
    let u = points[2].x - points[0].x;
    let v = points[2].y - points[0].y;
    if (x * v - y * u - 1.0).abs() > 1e-8 {
        return Err(GenError::Invalid(format!(
            "{label}: a straight ramp needs an upright sketch"
        )));
    }
    ramp.start_xy = [points[0].x, points[0].y];
    ramp.direction_xy = [x, y];
    Ok(ramp)
}

pub(super) fn rotate(ramp: &mut StraightContourRamp, rot: &Rot2) {
    let [x, y] = ramp.start_xy;
    let [dx, dy] = ramp.direction_xy;
    let start = rot.p2(Point2::new(x, y));
    let ahead = rot.p2(Point2::new(x + dx, y + dy));
    ramp.start_xy = [start.x, start.y];
    ramp.direction_xy = [ahead.x - start.x, ahead.y - start.y];
}

pub(super) fn protection(
    items: &[&ResolvedSketchChain],
    cp: &ContourParams,
) -> Result<Option<Region>, GenError> {
    if !items.iter().any(|item| item.straight_ramp.is_some()) {
        return Ok(None);
    }
    let mut protected = Region::empty();
    let clearance = cp.tool_diameter + cp.stock_to_leave + cp.tolerance + 0.01;
    for item in items
        .iter()
        .filter(|item| item.eff_side != Some(Side::Inside))
    {
        let outline = Region::new(vec![Loop::new(item.pts.clone())?])?;
        if !protected.is_empty() && !protected.intersect(&outline.offset(clearance)?)?.is_empty() {
            return Err(GenError::Invalid(format!(
                "{}: authored straight ramps need outside outlines separated by the swept cutter clearance",
                item.label
            )));
        }
        protected = protected.union(&outline)?;
    }
    for item in items
        .iter()
        .filter(|item| item.eff_side == Some(Side::Inside))
    {
        let hole = Region::new(vec![Loop::new(item.pts.clone())?])?;
        if !hole.difference(&protected)?.is_empty() {
            return Err(GenError::Invalid(format!(
                "{}: a bore selected with an authored straight ramp needs its containing outline selected",
                item.label
            )));
        }
    }
    if items
        .iter()
        .any(|item| item.eff_side != Some(Side::Inside) && item.straight_ramp.is_none())
    {
        return Err(GenError::Invalid(
            "all outside outlines in an authored straight-ramp operation need their own ramp"
                .into(),
        ));
    }
    Ok(Some(protected))
}

pub(super) fn generate(
    item: &ResolvedSketchChain,
    ramp: StraightContourRamp,
    region: &Region,
    protected: &Region,
    params: &ContourParams,
    carried_top: &mut dyn FnMut(&Region, &Toolpath, bool) -> Option<f64>,
) -> Result<Toolpath, GenError> {
    let path = straight_contour_ramp(region, protected, ramp, params)?;
    let plunge = path
        .moves
        .iter()
        .find(|m| m.kind == MoveKind::Plunge)
        .ok_or_else(|| GenError::Invalid("the straight ramp has no vertical approach".into()))?;
    let disc = super::top_stock::disc(
        Point2::new(plunge.to.x, plunge.to.y),
        params.tool_diameter * 0.5,
    )
    .ok_or_else(|| GenError::Invalid("cannot judge stock at the straight ramp entry".into()))?;
    if carried_top(&disc, &path, true)
        .is_some_and(|top| top - params.bottom_z > params.stepdown + 1e-6)
    {
        return Err(GenError::Invalid(format!(
            "{}: standing stock at the straight ramp entry exceeds one stepdown",
            item.label
        )));
    }
    Ok(path)
}
