//! Journeyman S6 (docs/journeyman/specs/S6-hole-chamfer-width.md); cone
//! geometry after Smid 2007; no vendor parameters, text or code.

use crate::cl::Move;
use crate::geo2d::{Loop, Point2};
use crate::geometry::{Point3, Triangle};

use super::excess::traces;
use super::hole_wall::{facet_sag, nearest, WALL_SAG};
use super::probe::{Atom, Cutter, Prof};

// The grant is geometric. Do not use the wider general gouge tolerance
// to decide whether a move is the declared operation.
const EPS: f64 = 1e-6;

/// An explicitly requested chamfer at a single, upright hole mouth.
/// The bottom is computed from the tool and width before verification,
/// independently of the moves being judged.
#[derive(Clone, Copy, Debug)]
pub struct HoleChamfer {
    xy: Point2,
    top: f64,
    bore: f64,
    mouth: f64,
    bottom: f64,
}

impl HoleChamfer {
    /// Invalid geometry grants nothing.
    #[must_use]
    pub fn new(xy: Point2, top: f64, bore: f64, width: f64, bottom: f64) -> Option<Self> {
        let mouth = bore + width;
        ([xy.x, xy.y, top, bore, mouth, bottom].iter().all(|v| v.is_finite())
            && bore >= 0.0
            && mouth > bore
            && bottom < top)
            .then_some(Self { xy, top, bore, mouth, bottom })
    }

    /// Only the vertical plunge and its in-place retraction can stand in
    /// this volume. Other moves keep all their original checks.
    pub(super) fn holds(&self, m: &Move, cutter: &Cutter, tip_radius: f64) -> bool {
        if m.arc.is_some() || m.axis.is_some() || !self.holds_piece(m.from, m.to) {
            return false;
        }
        let slope = (cutter.tip_angle.to_radians() * 0.5).tan();
        let depth = self.top - self.bottom;
        cutter.tip_angle > 0.0 && cutter.tip_angle < 180.0
            && tip_radius.is_finite() && tip_radius >= 0.0 && tip_radius < cutter.radius
            && depth <= cutter.flute_len + EPS && self.mouth <= cutter.radius + EPS
            && ((tip_radius + depth * slope) - self.mouth).abs() <= EPS
    }

    fn holds_piece(&self, p: Point3, q: Point3) -> bool {
        [p, q].into_iter().all(|a| {
            a.is_finite()
                && (a.x - self.xy.x).hypot(a.y - self.xy.y) <= EPS
                && a.z >= self.bottom - EPS
        })
    }

    pub(super) fn is_rim(&self, lp: &Loop) -> bool {
        self.bore > 0.0 && traces(lp, self.xy, self.bore)
    }

    /// At or below the mouth the verified cone is entirely within the
    /// chamfer and bore profile. Excuse only facets on that opening or
    /// its round wall. A lip, floor, other hole or piece above the mouth
    /// still reaches the original solid judge. Never excuse a shank,
    /// holder or flat flute atom.
    pub(super) fn excuses(&self, tri: &Triangle, a: &Atom, p: Point3, q: Point3) -> bool {
        if !matches!(a.prof, Prof::Cone { .. }) || a.z0 != 0.0 || !self.holds_piece(p, q) {
            return false;
        }
        let vertices = [tri.a, tri.b, tri.c];
        if vertices.iter().any(|v| !v.is_finite() || v.z > self.top + EPS) {
            return false;
        }
        let center = [self.xy.x, self.xy.y];
        let coords = vertices.map(|v| [v.x, v.y, v.z]);
        let radii = vertices.map(|v| (v.x - self.xy.x).hypot(v.y - self.xy.y));
        let on_bore = |r: f64| (r - self.bore).abs() <= EPS;
        if vertices.iter().all(|v| (v.z - self.top).abs() <= EPS) {
            let near = nearest(&coords, center);
            return near >= self.bore - WALL_SAG && near <= self.bore + EPS
                && (self.bore == 0.0 || radii.iter().copied().any(on_bore));
        }
        let normal = (tri.b - tri.a).cross(tri.c - tri.a);
        let len = normal.dot(normal).sqrt();
        self.bore > 0.0 && len > EPS && normal.z.abs() <= EPS * len
            && radii.into_iter().all(on_bore)
            && facet_sag(&coords, center).is_some_and(|s| s <= WALL_SAG)
    }
}
