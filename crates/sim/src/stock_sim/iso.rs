//! The field's surface as triangles when a consumer needs them (M7.2
//! D): iso-surface extraction at d = 0 over the field's own lattice.
//! Provenance: marching cubes (Lorensen & Cline, SIGGRAPH 1987) in its
//! tetrahedral form - each cube split into six tetrahedra around one
//! main diagonal (marching tetrahedra, as in Bloomenthal's "An Implicit
//! Surface Polygonizer", Graphics Gems IV, 1994), which needs no case
//! table and has no ambiguous faces; the default extractor is dual
//! contouring (Ju, Losasso, Schaefer & Warren, SIGGRAPH 2002, `dual`)
//! over the same samples. Our own Rust; nothing from vendor binaries.
//!
//! ## The laws here
//!
//! - **A cache of the field, never a second truth.** The mesh is
//!   sampled from `SdfField::distance` - the same query the eye marches
//!   and the verdict reads - and nothing reads it back as material.
//! - **Watertight by construction.** Every cube splits along the same
//!   diagonal direction, so two cubes sharing a face split it the same
//!   way and their triangles meet edge to edge. The sample grid is
//!   padded past the solid by a cell, so the surface closes.
//! - **Outward, always.** Each triangle is turned so its normal points
//!   from the inside corners toward the outside ones - STL's rule, and
//!   the one the viewport's back-face cull needs.
//! - **Refuse, never guess.** A grid past the sample budget, a NaN
//!   sample (an absent body has no surface to extract), or a degenerate
//!   cell is an `Err` by name - never a mesh with holes in it.

use std::collections::HashMap;

use crate::geometry::{Aabb, Point3, Triangle};

use super::field::SdfField;
use super::kernel::StockError;

/// Samples at most: the dense pack's own budget (`pack::MAX_TEXELS`).
pub const MAX_SAMPLES: usize = 1 << 23;

/// Cube corners as (dx, dy, dz) offsets, bit 0 = x, bit 1 = y, bit 2 = z.
pub(crate) const CORNER: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];

/// The six tetrahedra around the 0-7 diagonal.
const TETS: [[usize; 4]; 6] = [
    [0, 1, 3, 7],
    [0, 3, 2, 7],
    [0, 2, 6, 7],
    [0, 6, 4, 7],
    [0, 4, 5, 7],
    [0, 5, 1, 7],
];

/// How the d = 0 surface is polygonized. Dual contouring is the
/// default: it keeps a machined pocket's floor/wall edge sharp where
/// marching tetrahedra chamfers it; marching tetrahedra stays callable
/// by name and is the fallback whenever a dual mesh is not closed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IsoMethod {
    /// Dual contouring of Hermite data (`dual`), one QEF vertex a cell.
    #[default]
    Dual,
    /// Marching tetrahedra: crossings interpolated on the cell edges.
    Marching,
}

/// Extract the field's d = 0 surface at the field's own cell.
pub fn extract(f: &SdfField) -> Result<Vec<Triangle>, StockError> {
    extract_with(f, f.cell(), IsoMethod::default())
}

/// Extract at a sample spacing `step`, the grid anchored on the field's
/// cell centres (`step` = the cell samples each centre exactly once;
/// coarser skips cells for a lighter mesh).
pub fn extract_at(f: &SdfField, step: f64) -> Result<Vec<Triangle>, StockError> {
    extract_with(f, step, IsoMethod::default())
}

/// Extract at `step` by `method`. A dual mesh that is not closed (a
/// surface reaching the grid's pad, a fold no quad ring closes) is never
/// returned: the marching mesh of the same samples stands in, logged.
pub fn extract_with(f: &SdfField, step: f64, method: IsoMethod) -> Result<Vec<Triangle>, StockError> {
    polygonize(sample(f, step, None)?, method)
}

/// Extract only the part of the solid inside `clip`: the sampled value is
/// the CSG intersection `max(d, d_box)` of the field with the box, so the
/// box's faces close the mesh where it cuts the solid (a closed piece, as a
/// consumer cropping a cut to a region of interest needs). The grid spans
/// the field's bounds met with the box, so a small box on a large field
/// stays inside the sample budget. A box missing the field refuses by name.
pub fn extract_clipped(f: &SdfField, step: f64, method: IsoMethod, clip: Aabb) -> Result<Vec<Triangle>, StockError> {
    polygonize(sample(f, step, Some(clip))?, method)
}

fn polygonize(g: Grid, method: IsoMethod) -> Result<Vec<Triangle>, StockError> {
    match method {
        IsoMethod::Marching => Ok(marching(&g)),
        IsoMethod::Dual => {
            let tris = super::dual::contour(&g);
            let open = open_edges(&tris);
            if open == 0 {
                Ok(tris)
            } else {
                tracing::warn!("iso-surface: the dual mesh has {open} open edges - marching tetrahedra stands in");
                Ok(marching(&g))
            }
        }
    }
}

/// The sampled lattice both extractors read: `(n + 1)^3` field values
/// on cell centres, `n` cells a side, origin `lo`, spacing `step`.
pub(crate) struct Grid {
    pub(crate) lo: Point3,
    pub(crate) step: f64,
    pub(crate) n: [usize; 3],
    pub(crate) d: Vec<f64>,
}

impl Grid {
    /// World position of sample `s`.
    pub(crate) fn at(&self, s: [usize; 3]) -> Point3 {
        let st = self.step;
        Point3::new(self.lo.x + s[0] as f64 * st, self.lo.y + s[1] as f64 * st, self.lo.z + s[2] as f64 * st)
    }

    /// The value at sample `s`; NaN off the lattice (never a panic).
    pub(crate) fn val(&self, s: [usize; 3]) -> f64 {
        if s.iter().zip(self.n).any(|(&c, n)| c > n) {
            return f64::NAN;
        }
        let i = (s[2] * (self.n[1] + 1) + s[1]) * (self.n[0] + 1) + s[0];
        self.d.get(i).copied().unwrap_or(f64::NAN)
    }
}

/// Marching tetrahedra over the clipped lattice, streamed two z layers at
/// a time: memory grows with one layer, not the whole grid, so a box too
/// large for the sample budget at the field's own cell still extracts at
/// that cell (the budget guards a dense pack held whole; this holds two
/// layers). Same lattice, same values and same tetrahedra as `marching`
/// over `sample`, so the mesh is the one `extract_clipped(.., Marching, ..)`
/// would give, closed by the box faces. Layers are sampled on the
/// machine's threads.
pub fn extract_clipped_streamed(f: &SdfField, step: f64, clip: Aabb) -> Result<Vec<Triangle>, StockError> {
    let (lo, n, value) = lattice(f, step, Some(clip))?;
    let at = |s: [usize; 3]| Point3::new(lo.x + s[0] as f64 * step, lo.y + s[1] as f64 * step, lo.z + s[2] as f64 * step);
    let (nx, ny) = (n[0] + 1, n[1] + 1);
    let threads = std::thread::available_parallelism().map_or(1, |t| t.get()).min(ny).max(1);
    let layer = |k: usize| -> Result<Vec<f64>, StockError> {
        let mut out = vec![0.0f64; nx * ny];
        let rows_per = ny.div_ceil(threads);
        let bad = std::sync::Mutex::new(None);
        std::thread::scope(|sc| {
            for (t, chunk) in out.chunks_mut(rows_per * nx).enumerate() {
                let (at, value, bad) = (&at, &value, &bad);
                sc.spawn(move || {
                    for (r, row) in chunk.chunks_mut(nx).enumerate() {
                        let j = t * rows_per + r;
                        for (i, slot) in row.iter_mut().enumerate() {
                            let p = at([i, j, k]);
                            match value(p) {
                                Some(v) => *slot = v,
                                None => {
                                    *bad.lock().unwrap() = Some(p);
                                    return;
                                }
                            }
                        }
                    }
                });
            }
        });
        match bad.into_inner().unwrap() {
            Some(p) => Err(StockError::Invalid(format!("iso-surface: the field answered NaN at {p:?} - no surface to extract"))),
            None => Ok(out),
        }
    };
    let mut out = Vec::new();
    let mut below = layer(0)?;
    for k in 0..n[2] {
        let above = layer(k + 1)?;
        let val = |s: [usize; 3]| if s[2] == k { below[s[1] * nx + s[0]] } else { above[s[1] * nx + s[0]] };
        for j in 0..n[1] {
            for i in 0..n[0] {
                let mut p = [Point3::ZERO; 8];
                let mut v = [0.0f64; 8];
                for (c, o) in CORNER.iter().enumerate() {
                    let s = [i + o[0], j + o[1], k + o[2]];
                    p[c] = at(s);
                    v[c] = val(s);
                }
                if v.iter().all(|x| *x < 0.0) || v.iter().all(|x| *x >= 0.0) {
                    continue;
                }
                for t in TETS {
                    tet(&mut out, t.map(|c| p[c]), t.map(|c| v[c]));
                }
            }
        }
        below = above;
    }
    Ok(out)
}

/// The extraction lattice (origin, cells per axis) and its value at a
/// point (the field met with the clip box; None where the field is NaN).
#[allow(clippy::type_complexity)]
fn lattice<'a>(f: &'a SdfField, step: f64, clip: Option<Aabb>) -> Result<(Point3, [usize; 3], impl Fn(Point3) -> Option<f64> + Sync + 'a), StockError> {
    let refuse = |s: String| Err(StockError::Invalid(format!("iso-surface: {s}")));
    if !(step.is_finite() && step > 0.0) {
        return refuse(format!("the sample step must be finite and positive, not {step}"));
    }
    let mut b = f.bounds();
    if let Some(c) = clip {
        if !(c.min.is_finite() && c.max.is_finite()) {
            return refuse("the clip box is not finite".into());
        }
        b = Aabb {
            min: Point3::new(b.min.x.max(c.min.x), b.min.y.max(c.min.y), b.min.z.max(c.min.z)),
            max: Point3::new(b.max.x.min(c.max.x), b.max.y.min(c.max.y), b.max.z.min(c.max.z)),
        };
        if b.min.x > b.max.x || b.min.y > b.max.y || b.min.z > b.max.z {
            return refuse("the clip box misses the field's bounds".into());
        }
    }
    let cell = f.cell();
    let centre = |v: f64| ((v / cell).floor() - 1.0 + 0.5) * cell;
    let lo = Point3::new(centre(b.min.x), centre(b.min.y), centre(b.min.z));
    let span = [b.max.x - lo.x, b.max.y - lo.y, b.max.z - lo.z];
    if !(lo.is_finite() && span.iter().all(|s| s.is_finite() && *s >= 0.0)) {
        return refuse("the field's bounds are not a finite box".into());
    }
    let mut n = [0usize; 3];
    for (k, s) in span.iter().enumerate() {
        let c = (s / step).ceil() + 1.0;
        if !(1.0..1e7).contains(&c) {
            return refuse(format!("axis {k} gives no sane cell count ({c})"));
        }
        n[k] = c as usize;
    }
    let value = move |p: Point3| {
        let v = f.distance(p);
        if v.is_nan() {
            return None;
        }
        Some(match clip {
            Some(c) => {
                let o = [c.min.x - p.x, p.x - c.max.x, c.min.y - p.y, p.y - c.max.y, c.min.z - p.z, p.z - c.max.z];
                o.into_iter().fold(v, f64::max)
            }
            None => v,
        })
    };
    Ok((lo, n, value))
}

/// Sample the field on the extraction lattice; refusals by name.
pub(crate) fn sample(f: &SdfField, step: f64, clip: Option<Aabb>) -> Result<Grid, StockError> {
    let refuse = |s: String| Err(StockError::Invalid(format!("iso-surface: {s}")));
    let (lo, n, value) = lattice(f, step, clip)?;
    let samples = (n[0] + 1)
        .checked_mul(n[1] + 1)
        .and_then(|v| v.checked_mul(n[2] + 1))
        .filter(|&v| v <= MAX_SAMPLES);
    let Some(samples) = samples else {
        return refuse(format!("a {}x{}x{} grid is past the {MAX_SAMPLES}-sample budget", n[0], n[1], n[2]));
    };
    let mut g = Grid { lo, step, n, d: Vec::with_capacity(samples) };
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let p = g.at([i, j, k]);
                let Some(v) = value(p) else {
                    return refuse(format!("the field answered NaN at {p:?} - no surface to extract"));
                };
                g.d.push(v);
            }
        }
    }
    Ok(g)
}

/// Marching tetrahedra over the sampled lattice.
pub(crate) fn marching(g: &Grid) -> Vec<Triangle> {
    let mut out = Vec::new();
    for k in 0..g.n[2] {
        for j in 0..g.n[1] {
            for i in 0..g.n[0] {
                let mut p = [Point3::ZERO; 8];
                let mut v = [0.0f64; 8];
                for (c, o) in CORNER.iter().enumerate() {
                    let s = [i + o[0], j + o[1], k + o[2]];
                    p[c] = g.at(s);
                    v[c] = g.val(s);
                }
                if v.iter().all(|x| *x < 0.0) || v.iter().all(|x| *x >= 0.0) {
                    continue;
                }
                for t in TETS {
                    tet(&mut out, t.map(|c| p[c]), t.map(|c| v[c]));
                }
            }
        }
    }
    out
}

/// The fraction of a lattice edge within which a crossing is put on the
/// sample (#4107): at a 0.25 mm step it moves a corner at most 0.25 nm.
const NODE_SNAP: f64 = 1e-6;

/// Polygonize one tetrahedron: inside is d < 0.
fn tet(out: &mut Vec<Triangle>, p: [Point3; 4], v: [f64; 4]) {
    let inside: Vec<usize> = (0..4).filter(|&c| v[c] < 0.0).collect();
    let outside: Vec<usize> = (0..4).filter(|&c| v[c] >= 0.0).collect();
    let cross = |a: usize, b: usize| {
        let (da, db) = (v[a], v[b]);
        let t = if (da - db).abs() > f64::EPSILON { (da / (da - db)).clamp(0.0, 1.0) } else { 0.5 };
        // #4107: a crossing within NODE_SNAP of a sample sits on it. A value
        // a few picometres from zero (a floor on the lattice) put the corner
        // that close to the sample, under the 1e-9 weld; on the sample the
        // facets that share it are exactly zero-area and `push` drops them.
        let t = if t < NODE_SNAP { 0.0 } else if t > 1.0 - NODE_SNAP { 1.0 } else { t };
        let (pa, pb) = (p[a], p[b]);
        Point3::new(pa.x + (pb.x - pa.x) * t, pa.y + (pb.y - pa.y) * t, pa.z + (pb.z - pa.z) * t)
    };
    let mean = |s: &[usize]| {
        let n = s.len().max(1) as f64;
        let (x, y, z) = s.iter().fold((0.0, 0.0, 0.0), |(x, y, z), &c| (x + p[c].x, y + p[c].y, z + p[c].z));
        Point3::new(x / n, y / n, z / n)
    };
    let outward = mean(&outside) - mean(&inside);
    let mut push = |a: Point3, b: Point3, c: Point3| {
        let t = Triangle::new(a, b, c);
        let nrm = (b - a).cross(c - a);
        if nrm.dot(nrm) <= f64::MIN_POSITIVE {
            return; // a zero-area sliver adds no surface
        }
        out.push(if nrm.dot(outward) >= 0.0 { t } else { Triangle::new(a, c, b) });
    };
    match (inside.as_slice(), outside.as_slice()) {
        ([a], [b, c, e]) | ([b, c, e], [a]) => push(cross(*a, *b), cross(*a, *c), cross(*a, *e)),
        ([a, b], [c, e]) => {
            let (q0, q1, q2, q3) = (cross(*a, *c), cross(*a, *e), cross(*b, *e), cross(*b, *c));
            push(q0, q1, q2);
            push(q0, q2, q3);
        }
        _ => {}
    }
}

/// Edges used by an odd number of triangles: zero on a closed surface.
/// Vertices are welded on the 1e-9 lattice (`extract` interpolates each
/// shared edge from the same two samples, so shared points are equal).
#[must_use]
pub fn open_edges(tris: &[Triangle]) -> usize {
    let key = |p: Point3| [(p.x * 1e9).round() as i64, (p.y * 1e9).round() as i64, (p.z * 1e9).round() as i64];
    let mut count: HashMap<([i64; 3], [i64; 3]), u32> = HashMap::new();
    for t in tris {
        for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            let (ka, kb) = (key(a), key(b));
            let e = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *count.entry(e).or_insert(0) += 1;
        }
    }
    count.values().filter(|&&c| c % 2 == 1).count()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::cube;
    use crate::stock_sim::FIELD_BAND;

    fn volume(tris: &[Triangle]) -> f64 {
        tris.iter().map(|t| t.a.dot(t.b.cross(t.c)) / 6.0).sum()
    }

    #[test]
    fn a_box_extracts_closed_outward_and_near_its_volume() {
        let m = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 8.0, 6.0));
        let f = SdfField::from_mesh(&m.facets, 1.0, FIELD_BAND).unwrap();
        let tris = extract(&f).unwrap();
        assert!(!tris.is_empty());
        assert_eq!(open_edges(&tris), 0, "the extracted shell is watertight");
        let v = volume(&tris);
        let mt = volume(&extract_with(&f, 1.0, IsoMethod::Marching).unwrap());
        eprintln!("box volume: dual {v:.3}, marching {mt:.3}, true 480");
        // Outward normals give a positive volume; faces land exactly on
        // the cell-centre crossing, edges and corners are chamfered by
        // the tetrahedral split, so within 5% of 480.
        assert!(v > 0.0, "outward-facing: {v}");
        assert!((v - 480.0).abs() < 0.05 * 480.0, "volume {v} vs 480");
    }

    #[test]
    fn a_clip_box_extracts_the_closed_piece_inside_it() {
        let m = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 8.0, 6.0));
        let f = SdfField::from_mesh(&m.facets, 0.5, FIELD_BAND).unwrap();
        // A slab 3 wide in x, reaching past the solid in y and z: the piece is 3 x 8 x 6.
        let clip = Aabb { min: Point3::new(2.0, -1.0, -1.0), max: Point3::new(5.0, 9.0, 7.0) };
        let tris = extract_clipped(&f, 0.5, IsoMethod::Marching, clip).unwrap();
        assert_eq!(open_edges(&tris), 0, "the box faces close the piece");
        let v = volume(&tris);
        assert!((v - 144.0).abs() < 0.08 * 144.0, "piece volume {v} vs 144");
        let xs = tris.iter().flat_map(|t| [t.a.x, t.b.x, t.c.x]);
        let (lo, hi) = xs.fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), x| (l.min(x), h.max(x)));
        assert!(lo > 1.4 && hi < 5.6, "the piece stays inside the box in x: {lo}..{hi}");
        // The full field at a fine step is past the budget; the clipped grid is not.
        let wide = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 10.0));
        let g = SdfField::from_mesh(&wide.facets, 1.0, FIELD_BAND).unwrap();
        assert!(extract_at(&g, 0.05).is_err(), "the whole field at this step is past the budget");
        let small = Aabb { min: Point3::new(10.0, 10.0, 5.0), max: Point3::new(12.0, 12.0, 11.0) };
        let piece = extract_clipped(&g, 0.05, IsoMethod::Marching, small).unwrap();
        assert_eq!(open_edges(&piece), 0, "the clipped piece of the wide field is closed");
        let away = Aabb { min: Point3::new(900.0, 900.0, 900.0), max: Point3::new(901.0, 901.0, 901.0) };
        assert!(extract_clipped(&g, 0.05, IsoMethod::Marching, away).is_err(), "a box off the field refuses");
    }

    #[test]
    fn the_streamed_clip_is_the_dense_marching_mesh_and_skips_the_budget() {
        let m = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 8.0, 6.0));
        let f = SdfField::from_mesh(&m.facets, 0.5, FIELD_BAND).unwrap();
        let clip = Aabb { min: Point3::new(2.0, -1.0, -1.0), max: Point3::new(5.0, 9.0, 4.0) };
        let dense = extract_clipped(&f, 0.5, IsoMethod::Marching, clip).unwrap();
        let streamed = extract_clipped_streamed(&f, 0.5, clip).unwrap();
        assert_eq!(dense.len(), streamed.len(), "same lattice, same tetrahedra");
        for (a, b) in dense.iter().zip(&streamed) {
            assert_eq!([a.a, a.b, a.c], [b.a, b.b, b.c]);
        }
        assert_eq!(open_edges(&streamed), 0);
        // A box past the dense budget at this step still streams.
        let wide = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 10.0));
        let g = SdfField::from_mesh(&wide.facets, 1.0, FIELD_BAND).unwrap();
        let big = Aabb { min: Point3::new(-1.0, -1.0, 8.0), max: Point3::new(101.0, 101.0, 11.0) };
        assert!(extract_clipped(&g, 0.1, IsoMethod::Marching, big).is_err(), "dense refuses past the budget");
        let piece = extract_clipped_streamed(&g, 0.1, big).unwrap();
        assert_eq!(open_edges(&piece), 0, "streamed piece closed");
        let v = volume(&piece);
        assert!((v - 100.0 * 100.0 * 2.0).abs() < 0.03 * 20000.0, "slab volume {v} vs 20000");
    }

    #[test]
    fn nonsense_steps_and_huge_grids_refuse_by_name() {
        let m = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let f = SdfField::from_mesh(&m.facets, 0.5, FIELD_BAND).unwrap();
        for s in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(extract_at(&f, s).is_err(), "step {s} must refuse");
        }
        assert!(extract_at(&f, 1e-4).is_err(), "a 20000^3 grid is past the budget");
    }
}
