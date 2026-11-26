//! An outside contour that descends on its first straight wall segment.
//! Provenance: original implementation over the existing contour generator
//! and published linear interpolation; exercised with synthetic outlines.

use crate::cl::{MoveKind, PathBuilder, Toolpath};
use crate::entry::Entry3D;
use crate::geo2d::{Point2, Region};
use crate::geometry::Point3;
use crate::links::Links;
use serde::{Deserialize, Serialize};
use std::hash::{Hash, Hasher};

use super::{ContourParams, GenError, Lead, Side, contour};

/// Cutter-center seam and unit travel direction in the sketch plane.
/// The lead is normal to the selected segment, into the air side.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StraightContourRamp {
    pub start_xy: [f64; 2],
    pub direction_xy: [f64; 2],
    pub drop_mm: f64,
    pub run_mm: f64,
    pub normal_lead_mm: f64,
}

// Match the exact persisted words and the generation digest, including a signed zero.
impl PartialEq for StraightContourRamp {
    fn eq(&self, other: &Self) -> bool {
        self.words() == other.words()
    }
}

impl Eq for StraightContourRamp {}

impl Hash for StraightContourRamp {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.words().hash(state);
    }
}

impl StraightContourRamp {
    #[must_use]
    pub fn words(self) -> [u64; 7] {
        let [x, y] = self.start_xy;
        let [dx, dy] = self.direction_xy;
        [x, y, dx, dy, self.drop_mm, self.run_mm, self.normal_lead_mm].map(f64::to_bits)
    }

    #[must_use]
    pub fn is_sane(self) -> bool {
        let [x, y] = self.direction_xy;
        self.start_xy.iter().all(|v| v.is_finite())
            && (x.hypot(y) - 1.0).abs() < 1e-8
            && [self.drop_mm, self.run_mm, self.normal_lead_mm]
                .iter()
                .all(|v| v.is_finite() && *v > 0.0)
    }
}

/// Reuse the compensated contour's floor geometry and put the measured
/// descent in its first straight segment. The ring makes only one lap.
pub fn straight_contour_ramp(
    region: &Region,
    protected: &Region,
    ramp: StraightContourRamp,
    p: &ContourParams,
) -> Result<Toolpath, GenError> {
    p.validate()?;
    if !ramp.is_sane()
        || p.side != Side::Outside
        || p.tab_count > 0
        || p.links.approach.compensation.in_control()
        || !p.links.approach.center_cutting
        || !p.passes.is_default()
        || region.loops().len() != 1
    {
        return Err(GenError::Invalid(
            "a straight contour ramp needs one outside outline, a centre-cutting tool, and computer compensation without tabs or extra wall passes".into(),
        ));
    }
    let ap = &p.links.approach;
    if ap.feed_from != crate::approach::FeedFrom::Auto
        || ap.finish_levels != 0
        || ap.retract_feed > 0.0
        || ap.max_engagement > 0.0
        || ap.start_point.is_some()
        || ap.exit_point.is_some()
    {
        return Err(GenError::Invalid(
            "a straight ramp cannot combine with separate start/exit picks, feed-from, finishing levels, a fed retract, or an entry engagement limit".into(),
        ));
    }
    let floor = p.bottom_z;
    let start_z = floor + ramp.drop_mm;
    let maximum_angle = if ap.ramp_angle > 0.0 {
        ap.ramp_angle.min(15.0)
    } else {
        15.0
    };
    if ap.ramp_max_len > 0.0 && ramp.run_mm > ap.ramp_max_len {
        return Err(GenError::Invalid("the straight ramp exceeds the configured maximum run length".into()));
    }
    if start_z > p.top_z + 1e-8
        || p.top_z - floor > p.stepdown + 1e-8
        || ramp.run_mm <= 0.0254 * std::f64::consts::SQRT_2
        || ((ramp.drop_mm + 0.0254) / (ramp.run_mm - 0.0254 * std::f64::consts::SQRT_2))
            .atan()
            .to_degrees()
            > maximum_angle
    {
        return Err(GenError::Invalid(format!(
            "the straight ramp must reach the floor in one level, at {maximum_angle} degrees or less"
        )));
    }
    let mut base = *p;
    base.lead = Lead::None;
    base.entry = Entry3D::Plunge;
    base.links = Links::PLANAR;
    let ring = contour(region, &base)?;
    let cuts: Vec<_> = ring
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Feed)
        .collect();
    if cuts.is_empty()
        || cuts.iter().any(|m| m.from.z != floor || m.to.z != floor)
        || cuts.windows(2).any(|m| m[0].to != m[1].from)
        || cuts[0].from != cuts[cuts.len() - 1].to
    {
        return Err(GenError::Invalid(
            "a straight ramp needs one complete level contour".into(),
        ));
    }
    let [dx, dy] = ramp.direction_xy;
    let [sx, sy] = ramp.start_xy;
    let matching: Vec<_> = cuts
        .iter()
        .enumerate()
        .filter(|(_, m)| {
            let vx = m.to.x - m.from.x;
            let vy = m.to.y - m.from.y;
            m.arc.is_none()
                && (m.from.x - sx).hypot(m.from.y - sy) <= p.tolerance
                && (vx * dy - vy * dx).abs() < 1e-6
                && vx * dx + vy * dy > ramp.run_mm + 1e-8
        })
        .collect();
    let [(index, entry)] = matching.as_slice() else {
        return Err(GenError::Invalid(
            "the straight ramp needs one matching compensated straight span at its saved seam and direction".into(),
        ));
    };
    let start = entry.from;
    let air = super::contour::air_side(Point2::new(dx, dy), p.climb);
    let lead = Point2::new(
        start.x + ramp.normal_lead_mm * air.x,
        start.y + ramp.normal_lead_mm * air.y,
    );
    let slack = super::lead_fit::slack(p);
    let reach = (p.tool_diameter * 0.5 - slack).max(0.0);
    if !normal_lead_clear(protected, Point2::new(start.x, start.y), lead, reach)? {
        return Err(GenError::Invalid(
            "the straight ramp's normal lead crosses a protected outline".into(),
        ));
    }
    let mut path = PathBuilder::approached(Point3::new(lead.x, lead.y, p.clearance_z));
    let ramp_feed = if ap.ramp_feed > 0.0 {
        ap.ramp_feed
    } else {
        p.feed
    };
    path.push(
        MoveKind::Plunge,
        Point3::new(lead.x, lead.y, start_z),
        Some(ap.plunge(p.plunge_feed)),
    );
    path.push(
        MoveKind::Feed,
        Point3::new(start.x, start.y, start_z),
        Some(ap.lead_feed(p.feed)),
    );
    path.push(
        MoveKind::Feed,
        Point3::new(
            start.x + dx * ramp.run_mm,
            start.y + dy * ramp.run_mm,
            floor,
        ),
        Some(ramp_feed),
    );
    for cut in cuts[*index..].iter().chain(&cuts[..*index]) {
        if let Some(arc) = cut.arc {
            path.push_arc(MoveKind::Feed, cut.to, Some(p.feed), arc.center, arc.ccw);
        } else {
            path.push(MoveKind::Feed, cut.to, Some(p.feed));
        }
    }
    path.push(
        MoveKind::Feed,
        Point3::new(lead.x, lead.y, floor),
        Some(p.feed),
    );
    path.push(
        MoveKind::Rapid,
        Point3::new(lead.x, lead.y, p.clearance_z),
        None,
    );
    Ok(path.finish())
}

fn normal_lead_clear(
    protected: &Region,
    seam: Point2,
    lead: Point2,
    reach: f64,
) -> Result<bool, GenError> {
    let outside = protected.offset(reach)?;
    Ok(!outside.contains(lead) && outside.clip_polyline(&[seam, lead], true)?.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo2d::Loop;

    #[test]
    fn a_lead_that_ends_in_air_can_still_cross_a_concave_outline() {
        let points = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (8.0, 10.0),
            (8.0, 2.0),
            (2.0, 2.0),
            (2.0, 7.0),
            (0.0, 7.0),
        ];
        let region = Region::new(vec![
            Loop::new(points.into_iter().map(|(x, y)| Point2::new(x, y)).collect())
                .expect("U outline"),
        ])
        .expect("protected region");
        let seam = Point2::new(3.0, 5.0);
        assert!(
            normal_lead_clear(&region, seam, Point2::new(6.0, 5.0), 0.995).expect("clear span")
        );
        assert!(
            !normal_lead_clear(&region, seam, Point2::new(12.0, 5.0), 0.995).expect("blocked span")
        );
    }
}
