//! journeyman-job - cut one Journeyman group's stock with
//! the `stock_sim` seam and compare it with another engine's stock mesh.
//!
//! Inputs, either:
//!   --camotics <job.camotics>   the CAMotics project our cut lane writes
//!                               (tools, workpiece box, resolution, the NC
//!                               file); the NC is the same G-code subset
//!                               CAMotics cuts (G0/G1/G2/G3, G17/18/19,
//!                               M6 Tn per move, metric). A canned cycle
//!                               (G73/G74/G76/G81-G89) or G91 is refused
//!                               by name with its NC line, never skipped.
//!   --rows <rows.txt>           posed 5-axis rows `x y z i j k feed`
//!                               (feed <= 0 is a rapid), with --tool,
//!                               --stock and --scale.
//! Options:
//!   --backend column|field      stock engine (default column)
//!   --cell <mm>                 override the cell (default: the project's
//!                               resolution, else 0.254)
//!   --ref-stl <stock.stl>       another engine's stock: per-column top
//!                               heights compared with ours
//!   --part-stl <part.stl>       the part: `Stock::deviate` against it
//!   --tol <mm>                  deviate tolerance (default 0.0254)
//!   --out-stl <file>            field arm only: write the stock surface
//!   --shank <mm>                field arm only: cut with this tool length above the tip
//!   --cutting-length project|T=mm,..|<mm>   column arm: check each cutting move for material
//!                               above the flutes (the project's tool length, per-tool lengths,
//!                               or one length for all)
//!                               and report shank strikes; the cut itself is unchanged
//!   --out-closed-stl <file>     column arm: write the stock as a closed mesh
//!   --matrix a,b,...            16 numbers, row-major: setup-to-part for --out-closed-stl
//!   --move-lines <file.json>    camotics input: write each op's per-move NC line numbers
//!   --progressive <spec.json>   column arm: after each operation (JOURNEYMAN OPERATION ID n in
//!                               the NC), read sample points of each placed part copy against
//!                               the stock; per-operation surface on the cut / past the line
//!   --tool flat|ball|bull D [corner]   rows input only
//!   --stock x0 y0 z0 x1 y1 z1   rows input only (same units as the rows)
//!   --scale <k>                 rows input only: multiply lengths (25.4)
//!   --stl-scale <k>             multiply --ref-stl / --part-stl lengths
//! Prints one JSON object on stdout.
//!
//! Provenance: our driver over CamCraft's own public `stock_sim` API; the
//! G-code reading is the RS-274 subset our post writes.

use camcraft_sim::cl::{Move, MoveKind};
use camcraft_sim::geometry::{Point3, Triangle};
use camcraft_sim::geo2d::Point2;
use camcraft_sim::stock_sim::{iso, part_surface, Backend, SdfTool, SimOp, Stock, FIELD_BAND};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::time::Instant;

#[derive(Clone, Copy, Debug)]
struct ToolShape {
    radius: f64,
    ball: bool,
    cone: Option<f64>,
    corner: Option<f64>,
}

fn die(msg: String) -> ! {
    eprintln!("journeyman-job: {msg}");
    std::process::exit(1);
}

fn read_stl(path: &str, k: f64) -> Vec<Triangle> {
    let b = std::fs::read(path).unwrap_or_else(|e| die(format!("cannot read {path}: {e}")));
    let ascii = b.starts_with(b"solid") && {
        let n = u32::from_le_bytes([b[80], b[81], b[82], b[83]]) as usize;
        b.len() != 84 + 50 * n
    };
    let mut out = Vec::new();
    if ascii {
        let s = String::from_utf8_lossy(&b);
        let mut v = Vec::with_capacity(3);
        for line in s.lines() {
            let t = line.trim_start();
            if let Some(r) = t.strip_prefix("vertex") {
                let p: Vec<f64> = r.split_whitespace().filter_map(|w| w.parse().ok()).collect();
                if p.len() == 3 {
                    v.push(Point3::new(p[0] * k, p[1] * k, p[2] * k));
                }
                if v.len() == 3 {
                    out.push(Triangle::new(v[0], v[1], v[2]));
                    v.clear();
                }
            }
        }
    } else {
        let n = u32::from_le_bytes([b[80], b[81], b[82], b[83]]) as usize;
        out.reserve(n);
        let f = |o: usize| f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]) as f64 * k;
        for k in 0..n {
            let o = 84 + 50 * k + 12;
            let p = |j: usize| Point3::new(f(o + 12 * j), f(o + 12 * j + 4), f(o + 12 * j + 8));
            out.push(Triangle::new(p(0), p(1), p(2)));
        }
    }
    out
}

fn write_stl(path: &str, tris: &[Triangle]) {
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        b.extend_from_slice(&[0u8; 12]);
        for p in [t.a, t.b, t.c] {
            for v in [p.x, p.y, p.z] {
                b.extend_from_slice(&(v as f32).to_le_bytes());
            }
        }
        b.extend_from_slice(&[0u8; 2]);
    }
    std::fs::write(path, b).unwrap_or_else(|e| die(format!("cannot write {path}: {e}")));
}

/// The column stock as a closed solid: per material column a top and a bottom face, and vertical walls where
/// neighbours differ, each wall split at every height standing on its two end lines so no edge has a T-junction.
/// A saddle corner (two diagonal columns above the other two over a band) would put four faces on one edge. It is
/// split, not raised: a sliver (a right triangle with legs of a sixteenth of a cell, standing on the higher of the
/// two low columns, from its top up to the lower of the two high ones) joins the two high columns at the corner, so
/// the corner edge carries two faces. No cell height changes; the material added is the slivers' volume, reported.
/// `m` is an optional row-major 4x4 applied to every vertex (the job's setup-to-part matrix).
fn write_skyline(path: &str, tops: &[f64], nx: usize, ny: usize, b: [f64; 6], cell: f64, m: Option<&[f64]>) -> Value {
    use std::collections::HashMap;
    use std::io::{Seek, SeekFrom, Write};
    let [x0, y0, z0, x1, y1, z1] = b;
    // Heights are taken at f32 (the STL's precision) so two heights the file cannot tell apart are one height here.
    let q = |v: f64| v as f32 as f64;
    let (z0, z1) = (q(z0), q(z1));
    let h: Vec<f64> = tops.iter().map(|&t| if t.is_finite() && q(t) > z0 { q(t).min(z1) } else { z0 }).collect();
    let id = |i: usize, j: usize| j * nx + i;
    let xs: Vec<f64> = (0..=nx).map(|i| if i == nx { x1.min(x0 + nx as f64 * cell) } else { x0 + i as f64 * cell }).collect();
    let ys: Vec<f64> = (0..=ny).map(|j| if j == ny { y1.min(y0 + ny as f64 * cell) } else { y0 + j as f64 * cell }).collect();
    let cellh = |i: isize, j: isize| -> f64 {
        if i < 0 || j < 0 || i >= nx as isize || j >= ny as isize { z0 } else { h[j as usize * nx + i as usize] }
    };
    let corner = |i: usize, j: usize| -> Vec<f64> {
        let (i, j) = (i as isize, j as isize);
        let mut v = vec![z0, cellh(i - 1, j - 1), cellh(i, j - 1), cellh(i - 1, j), cellh(i, j)];
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.dedup();
        v
    };
    // Saddles and their slivers. A station is a sliver's corner on one of its owner's two edges (eps from the
    // saddle corner): every face that meets its vertical line splits there.
    let eps = cell / 16.0;
    struct Sliver { corner: (usize, usize), owner: (usize, usize), lo: f64, hi: f64, st: [usize; 2] }
    struct Station { p: [f64; 2], along: f64, heights: Vec<f64>, sliver: usize }
    let mut slivers: Vec<Sliver> = Vec::new();
    let mut stations: Vec<Station> = Vec::new();
    // Edge keys: (0, i, j) horizontal y = ys[j], x from xs[i] to xs[i + 1]; (1, i, j) vertical x = xs[i], y from ys[j].
    let mut on_edge: HashMap<(u8, usize, usize), Vec<usize>> = HashMap::new();
    let sorted = |mut v: Vec<f64>| { v.sort_by(|a, b| a.partial_cmp(b).unwrap()); v.dedup(); v };
    for j in 1..ny {
        for i in 1..nx {
            let (a, bb, c, d) = (h[id(i - 1, j - 1)], h[id(i, j - 1)], h[id(i - 1, j)], h[id(i, j)]);
            let pick = if a.min(d) > bb.max(c) {
                Some((if bb >= c { (i, j - 1) } else { (i - 1, j) }, a.min(d)))
            } else if bb.min(c) > a.max(d) {
                Some((if a >= d { (i - 1, j - 1) } else { (i, j) }, bb.min(c)))
            } else {
                None
            };
            let Some(((li, lj), hi)) = pick else { continue };
            let s = slivers.len();
            let lo = h[id(li, lj)];
            let (ex, ekey) = if li == i { (xs[i] + eps, (0u8, i, j)) } else { (xs[i] - eps, (0u8, i - 1, j)) };
            let across_h = h[id(li, if lj == j { j - 1 } else { j })];
            stations.push(Station { p: [ex, ys[j]], along: ex, heights: sorted(vec![lo, hi, across_h]), sliver: s });
            on_edge.entry(ekey).or_default().push(stations.len() - 1);
            let (ey, vkey) = if lj == j { (ys[j] + eps, (1u8, i, j)) } else { (ys[j] - eps, (1u8, i, j - 1)) };
            let across_v = h[id(if li == i { i - 1 } else { i }, lj)];
            stations.push(Station { p: [xs[i], ey], along: ey, heights: sorted(vec![lo, hi, across_v]), sliver: s });
            on_edge.entry(vkey).or_default().push(stations.len() - 1);
            slivers.push(Sliver { corner: (i, j), owner: (li, lj), lo, hi, st: [stations.len() - 2, stations.len() - 1] });
        }
    }
    for v in on_edge.values_mut() {
        v.sort_by(|a, b| stations[*a].along.partial_cmp(&stations[*b].along).unwrap());
    }
    let own_at: HashMap<(usize, usize), usize> = slivers.iter().enumerate().map(|(k, s)| (s.corner, k)).collect();
    let xf = |p: [f64; 3]| -> [f32; 3] {
        match m {
            Some(m) => [
                (m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3]) as f32,
                (m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7]) as f32,
                (m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11]) as f32,
            ],
            None => [p[0] as f32, p[1] as f32, p[2] as f32],
        }
    };
    let f = std::fs::File::create(path).unwrap_or_else(|e| die(format!("cannot write {path}: {e}")));
    let mut w = std::io::BufWriter::with_capacity(1 << 22, f);
    w.write_all(&[0u8; 84]).unwrap();
    let mut n: u64 = 0;
    let mut tri = |w: &mut std::io::BufWriter<std::fs::File>, a: [f64; 3], b2: [f64; 3], c: [f64; 3]| {
        let mut rec = [0u8; 50];
        for (k, p) in [a, b2, c].iter().enumerate() {
            let q = xf(*p);
            for (l, v) in q.iter().enumerate() {
                rec[12 + 12 * k + 4 * l..16 + 12 * k + 4 * l].copy_from_slice(&v.to_le_bytes());
            }
        }
        w.write_all(&rec).unwrap();
        n += 1;
    };
    let empty: Vec<usize> = Vec::new();
    let edge_stations = |k: (u8, usize, usize)| on_edge.get(&k).unwrap_or(&empty);
    let mut volume = 0.0;
    for j in 0..ny {
        for i in 0..nx {
            let z = h[id(i, j)];
            if z <= z0 { continue; }
            let (xa, xb, ya, yb) = (xs[i], xs[i + 1], ys[j], ys[j + 1]);
            volume += (z - z0) * (xb - xa) * (yb - ya);
            let keys = [(0u8, i, j), (1u8, i + 1, j), (0u8, i, j + 1), (1u8, i, j)];
            if keys.iter().all(|k| edge_stations(*k).is_empty()) {
                tri(&mut w, [xa, ya, z], [xb, ya, z], [xb, yb, z]);
                tri(&mut w, [xa, ya, z], [xb, yb, z], [xa, yb, z]);
                tri(&mut w, [xa, ya, z0], [xb, yb, z0], [xb, ya, z0]);
                tri(&mut w, [xa, ya, z0], [xa, yb, z0], [xb, yb, z0]);
                continue;
            }
            // Top and bottom as fans from the cell centre over the boundary loop (CCW from +z), so every station
            // on this cell's edges is a vertex. The top skips a corner where this cell owns a sliver (its two
            // stations cut that corner off); the bottom carries only the stations of slivers that stand on z0.
            let cs = [[xa, ya], [xb, ya], [xb, yb], [xa, yb]];
            let ks = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
            let (mut top, mut bot) = (Vec::new(), Vec::new());
            for k in 0..4 {
                let owned = own_at.get(&ks[k]).is_some_and(|s| slivers[*s].owner == (i, j));
                if !owned { top.push(cs[k]); }
                bot.push(cs[k]);
                let list = edge_stations(keys[k]);
                let ordered: Vec<usize> = if k >= 2 { list.iter().rev().copied().collect() } else { list.clone() };
                for s in ordered {
                    let sl = &slivers[stations[s].sliver];
                    top.push(stations[s].p);
                    if sl.owner != (i, j) && sl.lo <= z0 { bot.push(stations[s].p); }
                }
            }
            let c = [(xa + xb) / 2.0, (ya + yb) / 2.0];
            for k in 0..top.len() {
                let (p, r) = (top[k], top[(k + 1) % top.len()]);
                tri(&mut w, [c[0], c[1], z], [p[0], p[1], z], [r[0], r[1], z]);
            }
            for k in 0..bot.len() {
                let (p, r) = (bot[k], bot[(k + 1) % bot.len()]);
                tri(&mut w, [c[0], c[1], z0], [r[0], r[1], z0], [p[0], p[1], z0]);
            }
        }
    }
    // One wall: end lines p (heights la) and q (heights lb), between lo and hi, facing `out`.
    let mut wall = |w: &mut std::io::BufWriter<std::fs::File>, p: [f64; 2], q: [f64; 2], la: &[f64], lb: &[f64], lo: f64, hi: f64, out: [f64; 2]| {
        let a: Vec<f64> = la.iter().copied().filter(|z| *z >= lo && *z <= hi).collect();
        let bq: Vec<f64> = lb.iter().copied().filter(|z| *z >= lo && *z <= hi).collect();
        // cross(q - p, +z) = (dy, -dx): flip when it points away from `out`.
        let flip = (q[1] - p[1]) * out[0] - (q[0] - p[0]) * out[1] < 0.0;
        let (mut ia, mut ib) = (0usize, 0usize);
        while ia + 1 < a.len() || ib + 1 < bq.len() {
            let adv_a = ib + 1 == bq.len() || (ia + 1 < a.len() && a[ia + 1] <= bq[ib + 1]);
            let (u, v, s) = if adv_a {
                let t = ([p[0], p[1], a[ia]], [q[0], q[1], bq[ib]], [p[0], p[1], a[ia + 1]]);
                ia += 1;
                t
            } else {
                let t = ([p[0], p[1], a[ia]], [q[0], q[1], bq[ib]], [q[0], q[1], bq[ib + 1]]);
                ib += 1;
                t
            };
            if flip { tri(w, u, s, v) } else { tri(w, u, v, s) }
        }
    };
    // Every cell edge, split at its stations. On the piece between a saddle corner and its sliver's station the
    // owner's side stands at the sliver's top.
    struct Pt { p: [f64; 2], heights: Vec<f64>, corner: Option<(usize, usize)>, sliver: Option<usize> }
    for kind in 0..2u8 {
        let (ni, nj) = if kind == 0 { (nx, ny + 1) } else { (nx + 1, ny) };
        for j in 0..nj {
            for i in 0..ni {
                // Sides a (below / left) and b (above / right); `out` when a stands higher.
                let (ca, cb, k1, p1, out) = if kind == 0 {
                    ((i as isize, j as isize - 1), (i as isize, j as isize), (i + 1, j), [xs[i + 1], ys[j]], [0.0, 1.0])
                } else {
                    ((i as isize - 1, j as isize), (i as isize, j as isize), (i, j + 1), [xs[i], ys[j + 1]], [1.0, 0.0])
                };
                let list = edge_stations((kind, i, j));
                let (ha0, hb0) = (cellh(ca.0, ca.1), cellh(cb.0, cb.1));
                if ha0 == hb0 && list.is_empty() { continue; }
                let mut pts = vec![Pt { p: [xs[i], ys[j]], heights: corner(i, j), corner: Some((i, j)), sliver: None }];
                for s in list {
                    pts.push(Pt { p: stations[*s].p, heights: stations[*s].heights.clone(), corner: None, sliver: Some(stations[*s].sliver) });
                }
                pts.push(Pt { p: p1, heights: corner(k1.0, k1.1), corner: Some(k1), sliver: None });
                for seg in pts.windows(2) {
                    let (mut ha, mut hb) = (ha0, hb0);
                    for (st, other) in [(&seg[0], &seg[1]), (&seg[1], &seg[0])] {
                        if let (Some(s), Some(k)) = (st.sliver, other.corner) {
                            let sl = &slivers[s];
                            if sl.corner == k {
                                let o = (sl.owner.0 as isize, sl.owner.1 as isize);
                                if o == ca { ha = sl.hi } else if o == cb { hb = sl.hi }
                            }
                        }
                    }
                    if ha == hb { continue; }
                    let o = if ha > hb { out } else { [-out[0], -out[1]] };
                    wall(&mut w, seg[0].p, seg[1].p, &seg[0].heights, &seg[1].heights, ha.min(hb), ha.max(hb), o);
                }
            }
        }
    }
    // The slivers' own faces: the top at the band's top, the face across the owner cell, and a bottom on z0.
    let oriented = |a: [f64; 3], b2: [f64; 3], c: [f64; 3], want: [f64; 3]| {
        let (u, v) = ([b2[0] - a[0], b2[1] - a[1], b2[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let nrm = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        if nrm[0] * want[0] + nrm[1] * want[1] + nrm[2] * want[2] < 0.0 { (a, c, b2) } else { (a, b2, c) }
    };
    let mut sliver_vol = 0.0;
    for sl in &slivers {
        let p = [xs[sl.corner.0], ys[sl.corner.1]];
        let (e1, e2) = (stations[sl.st[0]].p, stations[sl.st[1]].p);
        let (a, b2, c) = oriented([p[0], p[1], sl.hi], [e1[0], e1[1], sl.hi], [e2[0], e2[1], sl.hi], [0.0, 0.0, 1.0]);
        tri(&mut w, a, b2, c);
        let out = [(e1[0] + e2[0]) / 2.0 - p[0], (e1[1] + e2[1]) / 2.0 - p[1], 0.0];
        let (a, b2, c) = oriented([e1[0], e1[1], sl.lo], [e2[0], e2[1], sl.lo], [e2[0], e2[1], sl.hi], out);
        tri(&mut w, a, b2, c);
        let (a, b2, c) = oriented([e1[0], e1[1], sl.lo], [e2[0], e2[1], sl.hi], [e1[0], e1[1], sl.hi], out);
        tri(&mut w, a, b2, c);
        if sl.lo <= z0 {
            let (a, b2, c) = oriented([p[0], p[1], z0], [e1[0], e1[1], z0], [e2[0], e2[1], z0], [0.0, 0.0, -1.0]);
            tri(&mut w, a, b2, c);
        }
        sliver_vol += 0.5 * eps * eps * (sl.hi - sl.lo);
    }
    w.flush().unwrap();
    let mut f = w.into_inner().unwrap();
    f.seek(SeekFrom::Start(80)).unwrap();
    f.write_all(&(n as u32).to_le_bytes()).unwrap();
    let band_max = slivers.iter().map(|s| s.hi - s.lo).fold(0.0f64, f64::max);
    json!({"path": path, "facets": n, "volume_mm3": volume + sliver_vol, "saddles": slivers.len(),
           "saddle_band_max_mm": band_max, "saddle_sliver_mm": eps, "saddle_sliver_mm3": sliver_vol,
           "saddle_cells_raised": 0, "saddle_raise_max_mm": 0.0,
           "saddle_basis": "each saddle corner split by a sliver (right triangle, legs a sixteenth of a cell) from the higher low column's top to the lower high column's top; no cell raised",
           "frame": if m.is_some() { "part (setup-to-part matrix applied)" } else { "setup" }})
}

#[cfg(test)]
mod skyline_tests {
    use super::write_skyline;
    use std::collections::HashMap;

    fn read(path: &std::path::Path) -> Vec<[[f32; 3]; 3]> {
        let b = std::fs::read(path).unwrap();
        let n = u32::from_le_bytes(b[80..84].try_into().unwrap()) as usize;
        assert_eq!(b.len(), 84 + 50 * n);
        (0..n).map(|t| {
            let r = &b[84 + 50 * t..];
            let v = |k: usize, l: usize| f32::from_le_bytes(r[12 + 12 * k + 4 * l..16 + 12 * k + 4 * l].try_into().unwrap());
            [[v(0, 0), v(0, 1), v(0, 2)], [v(1, 0), v(1, 1), v(1, 2)], [v(2, 0), v(2, 1), v(2, 2)]]
        }).collect()
    }

    /// Closed and oriented: every directed edge (by exact vertex) once, and its reverse once; no zero-area facet;
    /// returns the enclosed volume.
    fn closed_volume(tris: &[[[f32; 3]; 3]]) -> f64 {
        let key = |p: [f32; 3]| (p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
        let mut edges: HashMap<_, i32> = HashMap::new();
        let mut vol = 0.0;
        for t in tris {
            let d = |a: usize, b: usize| [(t[b][0] - t[a][0]) as f64, (t[b][1] - t[a][1]) as f64, (t[b][2] - t[a][2]) as f64];
            let (u, v) = (d(0, 1), d(0, 2));
            let cr = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            assert!(cr.iter().map(|c| c * c).sum::<f64>() > 0.0, "zero-area facet {t:?}");
            for k in 0..3 {
                *edges.entry((key(t[k]), key(t[(k + 1) % 3]))).or_default() += 1;
            }
            let p = |k: usize| [t[k][0] as f64, t[k][1] as f64, t[k][2] as f64];
            let (a, b, c) = (p(0), p(1), p(2));
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
        }
        for (&(a, b), &n) in &edges {
            assert_eq!(n, 1, "directed edge used {n} times");
            assert_eq!(edges.get(&(b, a)).copied(), Some(1), "edge without its twin (open or T-junction)");
        }
        vol
    }

    #[test]
    fn random_skylines_with_saddles_are_closed_and_keep_their_volume() {
        let dir = std::env::temp_dir();
        let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut saddles = 0;
        for case in 0..60 {
            let (nx, ny) = (3 + case % 9, 2 + case % 7);
            let levels = [f64::NEG_INFINITY, 1.0, 2.5, 4.0, 18.3];
            let tops: Vec<f64> = (0..nx * ny).map(|_| {
                seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
                levels[(seed % levels.len() as u64) as usize]
            }).collect();
            let cell = 0.254;
            let b = [-1.0, 2.0, 0.0, -1.0 + nx as f64 * cell, 2.0 + ny as f64 * cell, 20.0];
            let path = dir.join(format!("jm-skyline-test-{}-{case}.stl", std::process::id()));
            let rep = write_skyline(path.to_str().unwrap(), &tops, nx, ny, b, cell, None);
            let vol = closed_volume(&read(&path));
            std::fs::remove_file(&path).ok();
            let cells: f64 = tops.iter().map(|t| if t.is_finite() { t.min(20.0) } else { 0.0 }).sum::<f64>() * cell * cell;
            let want = rep["volume_mm3"].as_f64().unwrap();
            assert!((vol - want).abs() < 1e-3 * want.max(1.0), "case {case}: mesh {vol} vs reported {want}");
            assert!((want - cells - rep["saddle_sliver_mm3"].as_f64().unwrap()).abs() < 1e-6, "case {case}");
            assert_eq!(rep["saddle_cells_raised"].as_u64(), Some(0));
            saddles += rep["saddles"].as_u64().unwrap();
        }
        assert!(saddles > 20, "the random grids must exercise saddles ({saddles})");
    }
}

/// Rim copies of a band's cone (see `profile_atoms`).
const PHI: usize = 24;

/// Our CAMotics fork's revolved profile bands `[z0, z1, inner r0, inner r1, outer r0, outer r1]` (mm, tip at z = 0)
/// as CamCraft column atoms, each with the lift of the moves it is swept along. A band that widens upward (or
/// keeps its radius) is the flat disc of its lower radius lifted to z0, plus, where it widens, the sharp cone of
/// its slope swept around that disc's rim: the frustum is exactly the disc's Minkowski sum with the cone, so `PHI`
/// copies on the rim are an inner approximation within r0 (1 - cos(pi / PHI)) radially. A band narrower above than
/// below (an undercut) or with an inner radius (a hollow tool) cannot stand on the column engine's upright atoms
/// and is refused by name. None when the record has no bands.
fn profile_atoms(rec: &Value) -> Option<Result<Vec<(ToolShape, [f64; 3])>, String>> {
    let rows = rec["profile_bands"].as_array()?;
    if rows.is_empty() {
        return None;
    }
    let mut bands: Vec<[f64; 6]> = Vec::new();
    for r in rows {
        let v: Vec<f64> = r.as_array().map(|a| a.iter().filter_map(Value::as_f64).collect()).unwrap_or_default();
        if v.len() != 6 {
            return Some(Err("a profile band needs six numbers".into()));
        }
        bands.push([v[0], v[1], v[2], v[3], v[4], v[5]]);
    }
    bands.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
    let mut atoms = Vec::new();
    let mut below = 0.0f64;
    for &[z0, z1, i0, i1, r0, r1] in &bands {
        if i0 > 0.0 || i1 > 0.0 {
            return Some(Err(format!("hollow profile band (inner radius {i0} to {i1} mm): not an upright solid")));
        }
        if r1 + 1e-9 < r0 || r0 + 1e-9 < below {
            return Some(Err(format!("undercut profile band (radius {r0} to {r1} mm over z {z0} to {z1} mm): narrower above than below")));
        }
        below = r1;
        if r0 > 1e-9 {
            atoms.push((ToolShape { radius: r0, ball: false, cone: None, corner: None }, [0.0, 0.0, z0]));
        }
        if r1 > r0 + 1e-9 {
            let cone = ToolShape { radius: r1 - r0, ball: false, cone: Some((z1 - z0) / (r1 - r0)), corner: None };
            if r0 <= 1e-9 {
                atoms.push((cone, [0.0, 0.0, z0]));
            } else {
                for k in 0..PHI {
                    let a = std::f64::consts::TAU * k as f64 / PHI as f64;
                    atoms.push((cone, [r0 * a.cos(), r0 * a.sin(), z0]));
                }
            }
        }
    }
    Some(Ok(atoms))
}

/// The moves carried by `d` (tip positions and XY arc centres).
fn lifted(moves: &[Move], d: [f64; 3]) -> Vec<Move> {
    moves
        .iter()
        .map(|m| {
            let mut m = *m;
            m.from = Point3::new(m.from.x + d[0], m.from.y + d[1], m.from.z + d[2]);
            m.to = Point3::new(m.to.x + d[0], m.to.y + d[1], m.to.z + d[2]);
            if let Some(a) = m.arc.as_mut() {
                a.center = Point2::new(a.center.x + d[0], a.center.y + d[1]);
            }
            m
        })
        .collect()
}

/// CAMotics tool record -> the stock_sim atom. `conical` is a cone of
/// height `length` to `diameter` (the virtual sharp leads); `ballnose`
/// the sphere; `snubnose` a bull nose; anything else the flat cylinder
/// (a tapped-thread record cuts its major-diameter cylinder here).
fn camotics_tool(t: &Value) -> Option<ToolShape> {
    let d = t["diameter"].as_f64()?;
    if d <= 0.0 {
        return None;
    }
    let r = d / 2.0;
    let shape = t["shape"].as_str().unwrap_or("cylindrical");
    Some(match shape {
        "ballnose" => ToolShape { radius: r, ball: true, cone: None, corner: None },
        "conical" => {
            let h = t["length"].as_f64().unwrap_or(r);
            ToolShape { radius: r, ball: false, cone: Some(h / r), corner: None }
        }
        "snubnose" => {
            let sd = t["snub_diameter"].as_f64().unwrap_or(d);
            let corner = ((d - sd) / 2.0).max(1e-6).min(r);
            ToolShape { radius: r, ball: false, cone: None, corner: Some(corner) }
        }
        _ => match t["corner_radius"].as_f64() {
            // Our CAMotics fork's bull nose: a cylinder with a corner radius.
            Some(c) if c > 0.0 && c < r => ToolShape { radius: r, ball: false, cone: None, corner: Some(c) },
            Some(c) if c >= r => ToolShape { radius: r, ball: true, cone: None, corner: None },
            _ => ToolShape { radius: r, ball: false, cone: None, corner: None },
        },
    })
}

fn word(line: &str, c: char) -> Option<f64> {
    let mut it = line.char_indices().peekable();
    while let Some((i, ch)) = it.next() {
        if ch == c {
            let rest = &line[i + 1..];
            let end = rest
                .find(|x: char| !(x.is_ascii_digit() || x == '.' || x == '-' || x == '+'))
                .unwrap_or(rest.len());
            if end > 0 {
                return rest[..end].parse().ok();
            }
        }
    }
    None
}

/// Chord an arc in plane (u, v) with normal coordinate w: points from a to b
/// about centre c, sagitta <= tol.
fn arc_points(a: [f64; 3], b: [f64; 3], c: [f64; 2], ccw: bool, plane: (usize, usize, usize), tol: f64) -> Vec<[f64; 3]> {
    let (u, v, w) = plane;
    let r = ((a[u] - c[0]).powi(2) + (a[v] - c[1]).powi(2)).sqrt();
    let a0 = (a[v] - c[1]).atan2(a[u] - c[0]);
    let mut a1 = (b[v] - c[1]).atan2(b[u] - c[0]);
    let tau = std::f64::consts::TAU;
    if ccw {
        while a1 <= a0 + 1e-12 { a1 += tau; }
    } else {
        while a1 >= a0 - 1e-12 { a1 -= tau; }
    }
    let sweep = a1 - a0;
    let step = if r > tol { 2.0 * (1.0 - tol / r).acos() } else { sweep.abs() };
    let n = ((sweep.abs() / step.max(1e-6)).ceil() as usize).clamp(1, 100_000);
    (1..=n)
        .map(|k| {
            let t = k as f64 / n as f64;
            let ang = a0 + sweep * t;
            let mut p = [0.0; 3];
            p[u] = c[0] + r * ang.cos();
            p[v] = c[1] + r * ang.sin();
            p[w] = a[w] + (b[w] - a[w]) * t;
            if k == n { p = b; }
            p
        })
        .collect()
}

/// Every G number on a line ("G81G99" and "G81 G99" alike), as integers; G-words with a fraction are kept as
/// their integer part times 10 plus the tenth (G90.1 -> 901) so they never read as a whole code.
fn g_words(line: &str) -> Vec<u32> {
    let b = line.as_bytes();
    let mut out = Vec::new();
    for (i, &c) in b.iter().enumerate() {
        if (c == b'G' || c == b'g') && (i == 0 || !b[i - 1].is_ascii_alphanumeric() || b[i - 1].is_ascii_digit()) {
            let rest = &line[i + 1..];
            let end = rest.find(|x: char| !(x.is_ascii_digit() || x == '.')).unwrap_or(rest.len());
            let w = &rest[..end];
            match w.split_once('.') {
                Some((a, f)) => if let (Ok(a), Ok(f)) = (a.parse::<u32>(), f.parse::<u32>()) { out.push(a * 10 + f) },
                None => if let Ok(a) = w.parse::<u32>() { out.push(a) },
            }
        }
    }
    out
}

/// A word whose motion this reader does not expand: cutting it would silently skip or misplace material, so the
/// program is refused by name (the lane routes such a program to CAMotics).
fn refused_word(g: u32) -> Option<&'static str> {
    match g {
        73 | 74 | 76 | 81..=89 => Some("canned cycle (not expanded upstream; this reader cuts G0/G1/G2/G3 only)"),
        91 => Some("incremental positioning (this reader reads absolute G90 only)"),
        _ => None,
    }
}

/// One run of moves of one tool within one operation, in program order.
type NcOp = (i64, Option<i64>, Vec<Move>, Vec<usize>);

/// Our post's G-code -> runs of moves per (tool number, operation), in program order: the moves and, for each move,
/// its 1-based NC line. The operation is the id in our post's `(JOURNEYMAN OPERATION ID n)` comment (None before
/// the first one).
fn read_nc(path: &str) -> Vec<NcOp> {
    let src = std::fs::read_to_string(path).unwrap_or_else(|e| die(format!("cannot read {path}: {e}")));
    let mut ops: Vec<NcOp> = Vec::new();
    let (mut pos, mut have) = ([0.0f64; 3], false);
    let (mut tool, mut modal, mut plane) = (0i64, 0u8, 17u8);
    let mut op: Option<i64> = None;
    let mut feed = 1000.0;
    let mut ln = 0usize;
    let push = |tool: i64, op: Option<i64>, m: Move, ln: usize, ops: &mut Vec<NcOp>| match ops.last_mut() {
        Some((t, o, v, l)) if *t == tool && *o == op => { v.push(m); l.push(ln); }
        _ => ops.push((tool, op, vec![m], vec![ln])),
    };
    for raw in src.lines() {
        ln += 1;
        if let Some(i) = raw.find("JOURNEYMAN OPERATION ID") {
            op = raw[i + 23..].trim().trim_end_matches(')').trim().parse::<i64>().ok().or(op);
        }
        let line = match raw.find('(') { Some(i) => &raw[..i], None => raw }.trim();
        if line.is_empty() { continue; }
        if let Some((g, why)) = g_words(line).into_iter().find_map(|g| refused_word(g).map(|w| (g, w))) {
            let g = if g > 100 { format!("G{}.{}", g / 10, g % 10) } else { format!("G{g}") };
            die(format!("refused: {g} at NC line {ln}: {why}"));
        }
        if line.contains("M6") || line.starts_with('T') {
            if let Some(t) = word(line, 'T') { tool = t.round() as i64; }
        }
        for g in [17u8, 18, 19] {
            if line.split_whitespace().any(|w| w == format!("G{g}")) { plane = g; }
        }
        for w in line.split_whitespace() {
            match w { "G0" | "G00" => modal = 0, "G1" | "G01" => modal = 1, "G2" | "G02" => modal = 2, "G3" | "G03" => modal = 3, _ => {} }
        }
        if let Some(f) = word(line, 'F') { feed = f; }
        let (x, y, z) = (word(line, 'X'), word(line, 'Y'), word(line, 'Z'));
        if x.is_none() && y.is_none() && z.is_none() { continue; }
        let to = [x.unwrap_or(pos[0]), y.unwrap_or(pos[1]), z.unwrap_or(pos[2])];
        if !have { pos = to; have = true; continue; }
        let p3 = |p: [f64; 3]| Point3::new(p[0], p[1], p[2]);
        let cutting = tool != 0 && modal != 0;
        let kind = if !cutting { MoveKind::Rapid } else { MoveKind::Feed };
        let fd = if cutting { Some(feed) } else { None };
        if modal >= 2 {
            let (i, j, k) = (word(line, 'I').unwrap_or(0.0), word(line, 'J').unwrap_or(0.0), word(line, 'K').unwrap_or(0.0));
            let ccw = modal == 3;
            let full = (to[0] - pos[0]).abs() < 1e-9 && (to[1] - pos[1]).abs() < 1e-9 && (to[2] - pos[2]).abs() < 1e-9;
            if plane == 17 && !full {
                let c = Point2::new(pos[0] + i, pos[1] + j);
                push(tool, op, Move::new(kind, p3(pos), p3(to), fd).with_arc(c, ccw), ln, &mut ops);
            } else {
                let (pl, c) = match plane {
                    18 => ((2usize, 0usize, 1usize), [pos[2] + k, pos[0] + i]),
                    19 => ((1, 2, 0), [pos[1] + j, pos[2] + k]),
                    _ => ((0, 1, 2), [pos[0] + i, pos[1] + j]),
                };
                // G18 views from +Y: Z->X is CCW (RS-274), handled by the (z, x) pair.
                let mut prev = pos;
                for p in arc_points(pos, to, c, ccw, pl, 0.002) {
                    push(tool, op, Move::new(kind, p3(prev), p3(p), fd), ln, &mut ops);
                    prev = p;
                }
            }
        } else if to != pos {
            push(tool, op, Move::new(kind, p3(pos), p3(to), fd), ln, &mut ops);
        }
        pos = to;
    }
    ops
}

/// `--progressive`: each part copy's surface read toolpath by toolpath against the stock the group's earlier
/// toolpaths left (the column engine's stock: one material height per column). A copy is a set of surface samples
/// (point, outward normal, area, face index, setup frame mm). After each operation a sample is past the line when
/// the material `line` inside its surface is gone, and on the cut when the material `line` outside is gone while
/// the inside one stands. Each sample keeps the first operation that made it so; the state before the first
/// operation is kept apart (never counted to a toolpath).
struct PartCopy {
    key: String,
    s: Vec<[f32; 8]>,          // x y z nx ny nz area face (face as f32 bits)
    past: Vec<u32>,            // first evaluation index past the line (u32::MAX: never)
    on: Vec<u32>,              // first evaluation index on the cut
    unmeasured: Vec<bool>,     // the inside or outside point is in another X tile or off the stock's columns
}

struct Progressive {
    line: f64,
    z0: f64,
    copies: Vec<PartCopy>,
    order: Vec<i64>,           // operation ids, program order
}

const BEFORE: u32 = u32::MAX - 1;
const NEVER: u32 = u32::MAX;

impl Progressive {
    fn load(path: &str, z0: f64, op_ids: &[Option<i64>]) -> Self {
        let spec: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap_or_else(|e| die(format!("{path}: {e}"))))
            .unwrap_or_else(|e| die(format!("{path}: {e}")));
        let line = spec["line_mm"].as_f64().filter(|l| *l > 0.0).unwrap_or_else(|| die("progressive: line_mm".into()));
        let mut copies = Vec::new();
        for c in spec["copies"].as_array().unwrap_or_else(|| die("progressive: copies".into())) {
            let f = c["samples"].as_str().unwrap_or_else(|| die("progressive: samples".into()));
            let b = std::fs::read(f).unwrap_or_else(|e| die(format!("{f}: {e}")));
            if b.len() % 32 != 0 { die(format!("{f}: not 32-byte samples")); }
            let s: Vec<[f32; 8]> = b.chunks_exact(32).map(|r| {
                let mut a = [0f32; 8];
                for (k, v) in a.iter_mut().enumerate() { *v = f32::from_le_bytes(r[4 * k..4 * k + 4].try_into().unwrap()); }
                a
            }).collect();
            let n = s.len();
            copies.push(PartCopy { key: c["key"].as_str().unwrap_or("?").to_string(), s, past: vec![NEVER; n], on: vec![NEVER; n],
                               unmeasured: vec![false; n] });
        }
        let mut order = Vec::new();
        for id in op_ids.iter().flatten() { if !order.contains(id) { order.push(*id); } }
        Progressive { line, z0, copies, order }
    }

    /// Read every sample whose point lies in this X tile against the tile's stock; `at` is the evaluation index.
    fn eval(&mut self, stock: &Stock, tx0: f64, tx1: f64, last_tile: bool, at: u32) {
        let (line, z0) = (self.line, self.z0);
        let in_tile = |x: f64| x >= tx0 && (x < tx1 || (last_tile && x <= tx1));
        for c in &mut self.copies {
            for k in 0..c.s.len() {
                let s = c.s[k];
                if !in_tile(s[0] as f64) || c.unmeasured[k] { continue; }
                let p = [s[0] as f64, s[1] as f64, s[2] as f64];
                let nrm = [s[3] as f64, s[4] as f64, s[5] as f64];
                let q = |sgn: f64| [p[0] + sgn * line * nrm[0], p[1] + sgn * line * nrm[1], p[2] + sgn * line * nrm[2]];
                let (qi, qo) = (q(-1.0), q(1.0));
                if !in_tile(qi[0]) || !in_tile(qo[0]) { c.unmeasured[k] = true; continue; }
                // Gone: below the stock's bottom, or above the column's top (a column with no top is off the grid).
                let mut off = false;
                let mut gone = |r: [f64; 3]| -> bool {
                    if r[2] < z0 { return true; }
                    let t = stock.top(r[0], r[1]);
                    if t.is_nan() { off = true; return true; }
                    !(r[2] < t)
                };
                let (gi, go) = (gone(qi), gone(qo));
                if off { c.unmeasured[k] = true; continue; }
                if gi && c.past[k] == NEVER { c.past[k] = at; }
                if go && !gi && c.on[k] == NEVER { c.on[k] = at; }
            }
        }
    }

    /// Per copy, per operation in program order: surface newly on the cut and newly past the line (mm2), faces
    /// first entered, faces first cut to size (some of the face on the cut and none of it past the line, at the
    /// first operation where that holds), plus what stood before the first operation.
    fn report(&self) -> Value {
        let n = self.order.len() as u32;
        let mut copies = serde_json::Map::new();
        for c in &self.copies {
            let face = |k: usize| c.s[k][7].to_bits();
            let (mut on_a, mut past_a) = (vec![0.0f64; n as usize], vec![0.0f64; n as usize]);
            let (mut before_on, mut before_past, mut unmeasured_a, mut area) = (0.0, 0.0, 0.0, 0.0);
            let mut faces: BTreeMap<u32, (u32, u32)> = BTreeMap::new();   // face -> (first past, first on by a toolpath)
            for k in 0..c.s.len() {
                let a = c.s[k][6] as f64;
                area += a;
                if c.unmeasured[k] { unmeasured_a += a; continue; }
                let e = faces.entry(face(k)).or_insert((NEVER, NEVER));
                // A face any of whose surface was past the line before the first toolpath stays BEFORE.
                match c.past[k] {
                    BEFORE => { before_past += a; e.0 = BEFORE; }
                    NEVER => {}
                    i => { past_a[i as usize] += a; if e.0 != BEFORE { e.0 = e.0.min(i); } }
                }
                match c.on[k] { BEFORE => before_on += a, NEVER => {}, i => { on_a[i as usize] += a; e.1 = e.1.min(i); } }
            }
            let (mut entered, mut to_size) = (vec![0u32; n as usize], vec![0u32; n as usize]);
            for (_, &(fp, fo)) in &faces {
                if fp < n { entered[fp as usize] += 1; }
                // cut to size at the first toolpath that brings some of it on the cut while none of it is past
                if fo < n && (fp == NEVER || (fp != BEFORE && fp > fo)) { to_size[fo as usize] += 1; }
            }
            let rows: Vec<Value> = (0..n as usize).map(|i| json!({"operation_id": self.order[i],
                "surface_newly_on_cut_mm2": on_a[i], "surface_newly_past_mm2": past_a[i],
                "faces_newly_entered": entered[i], "faces_first_cut_to_size": to_size[i]})).collect();
            copies.insert(c.key.clone(), json!({"per_toolpath": rows, "samples": c.s.len(), "faces": faces.len(),
                "surface_mm2": area, "unmeasured_mm2": unmeasured_a, "before_first_toolpath_on_cut_mm2": before_on,
                "before_first_toolpath_past_mm2": before_past}));
        }
        json!({"line_mm": self.line, "operations": self.order, "copies": copies,
               "basis": "column stock (one material height per column) after each operation; a surface sample is past the line when the material line_mm inside it is gone, on the cut when the material line_mm outside is gone and the inside stands; areas are the samples' own; a face is entered at its first sample past the line, cut to size at the first operation that puts some of it on the cut while none of it is past; volume into a copy not measured"})
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut opt: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut i = 0;
    while i < args.len() {
        let k = args[i].trim_start_matches("--").to_string();
        let n = match k.as_str() { "tool" => 3, "stock" => 6, _ => 1 };
        let mut v = Vec::new();
        for j in 0..n {
            if let Some(a) = args.get(i + 1 + j) {
                if j > 0 && a.starts_with("--") { break; }
                v.push(a.clone());
            }
        }
        i += 1 + v.len();
        opt.insert(k, v);
    }
    let get = |k: &str| opt.get(k).and_then(|v| v.first()).cloned();
    let num = |s: &str| s.parse::<f64>().unwrap_or_else(|_| die(format!("not a number: {s}")));
    let backend = match get("backend").as_deref() {
        None | Some("column") => Backend::Column,
        Some("field") => Backend::Field { band: FIELD_BAND },
        Some(o) => die(format!("unknown backend {o}")),
    };
    let stl_k = get("stl-scale").map(|s| num(&s)).unwrap_or(1.0);
    let t0 = Instant::now();
    // Per op: the CAMotics tool number and its cutting length (the project's `length`, which our lane writes from
    // the tool's flute height); None where the project's length is not a flute (a conical record's is its cone).
    let mut cut_lens: Vec<(i64, Option<f64>)> = Vec::new();
    let mut profiled: Vec<i64> = Vec::new();
    // Per op: each move's 1-based NC line (empty for rows input), so a reported move names its program line.
    let mut op_lines: Vec<Vec<usize>> = Vec::new();
    // Per op: its operation id (our post's comment; None for rows input) and whether it is the first atom of its
    // run (the atom that carries the shank check).
    let mut op_ids: Vec<Option<i64>> = Vec::new();
    let mut first_atom: Vec<bool> = Vec::new();
    let (bounds, cell, ops): ([f64; 6], f64, Vec<(ToolShape, Vec<Move>)>) = if let Some(p) = get("camotics") {
        let proj: Value = serde_json::from_str(&std::fs::read_to_string(&p).unwrap_or_else(|e| die(format!("{p}: {e}"))))
            .unwrap_or_else(|e| die(format!("{p}: {e}")));
        if proj["units"].as_str() != Some("metric") { die("only metric projects".into()); }
        let w = &proj["workpiece"]["bounds"];
        let f = |a: &str, k: usize| w[a][k].as_f64().unwrap_or_else(|| die("workpiece bounds".into()));
        let b = [f("min", 0), f("min", 1), f("min", 2), f("max", 0), f("max", 1), f("max", 2)];
        let cell = get("cell").map(|s| num(&s)).or(proj["resolution"].as_f64()).unwrap_or(0.254);
        let nc = proj["files"][0].as_str().unwrap_or_else(|| die("no NC file".into())).to_string();
        let mut ops = Vec::new();
        for (t, opid, mv, lines) in read_nc(&nc) {
            let rec = &proj["tools"][t.to_string()];
            let len = rec["length"].as_f64().filter(|l| *l > 0.0 && rec["shape"].as_str() != Some("conical"));
            match profile_atoms(rec) {
                Some(Err(e)) => die(format!("tool {t}: {e}")),
                Some(Ok(atoms)) => {
                    if !profiled.contains(&t) { profiled.push(t); }
                    // The first atom carries the shank check (its own tip); the rest are the same tool's form.
                    for (k, (ts, d)) in atoms.into_iter().enumerate() {
                        cut_lens.push((t, if k == 0 { len } else { None }));
                        op_lines.push(lines.clone());
                        op_ids.push(opid);
                        first_atom.push(k == 0);
                        ops.push((ts, if d == [0.0; 3] { mv.clone() } else { lifted(&mv, d) }));
                    }
                    continue;
                }
                None => {}
            }
            match camotics_tool(rec) {
                Some(ts) => {
                    cut_lens.push((t, len));
                    op_lines.push(lines);
                    op_ids.push(opid);
                    first_atom.push(true);
                    ops.push((ts, mv));
                }
                None => {
                    if mv.iter().any(|m| m.kind != MoveKind::Rapid) && t != 0 {
                        die(format!("tool {t} has no usable shape"));
                    }
                }
            }
        }
        (b, cell, ops)
    } else if let Some(p) = get("rows") {
        let k = get("scale").map(|s| num(&s)).unwrap_or(1.0);
        let tv = opt.get("tool").cloned().unwrap_or_else(|| die("--tool needed".into()));
        let r = num(&tv[1]) * k / 2.0;
        let ts = match tv[0].as_str() {
            "ball" => ToolShape { radius: r, ball: true, cone: None, corner: None },
            "bull" => ToolShape { radius: r, ball: false, cone: None, corner: Some(num(&tv[2]) * k) },
            _ => ToolShape { radius: r, ball: false, cone: None, corner: None },
        };
        let sv = opt.get("stock").cloned().unwrap_or_else(|| die("--stock needed".into()));
        let b: Vec<f64> = sv.iter().map(|s| num(s) * k).collect();
        let cell = get("cell").map(|s| num(&s)).unwrap_or(0.254);
        let max_seg = get("max-seg").map(|s| num(&s)).unwrap_or(f64::INFINITY);
        let src = std::fs::read_to_string(&p).unwrap_or_else(|e| die(format!("{p}: {e}")));
        let mut mv = Vec::new();
        let mut prev: Option<Point3> = None;
        for line in src.lines() {
            let v: Vec<f64> = line.split_whitespace().filter_map(|w| w.parse().ok()).collect();
            if v.len() < 7 { continue; }
            let p = Point3::new(v[0] * k, v[1] * k, v[2] * k);
            if let Some(a) = prev {
                if (a.x - p.x).abs() + (a.y - p.y).abs() + (a.z - p.z).abs() > 1e-9 {
                    let kind = if v[6] > 0.0 { MoveKind::Feed } else { MoveKind::Rapid };
                    let fd = if v[6] > 0.0 { Some(v[6] * k) } else { None };
                    let ax = [v[3], v[4], v[5]];
                    let upright = ax[0].abs() < 1e-12 && ax[1].abs() < 1e-12 && ax[2] > 0.0;
                    // --max-seg: the field arm refuses a chord whose window passes 4M cells; split long cuts.
                    let len = ((p.x - a.x).powi(2) + (p.y - a.y).powi(2) + (p.z - a.z).powi(2)).sqrt();
                    let n = if kind == MoveKind::Rapid { 1 } else { ((len / max_seg).ceil() as usize).max(1) };
                    for s in 0..n {
                        let f0 = s as f64 / n as f64;
                        let f1 = (s + 1) as f64 / n as f64;
                        let lerp = |f: f64| Point3::new(a.x + (p.x - a.x) * f, a.y + (p.y - a.y) * f, a.z + (p.z - a.z) * f);
                        let mut m = Move::new(kind, lerp(f0), if s + 1 == n { p } else { lerp(f1) }, fd);
                        if !upright { m = m.with_axis(ax); }
                        mv.push(m);
                    }
                }
            }
            prev = Some(p);
        }
        ([b[0], b[1], b[2], b[3], b[4], b[5]], cell, vec![(ts, mv)])
    } else {
        die("need --camotics or --rows".into())
    };
    let read_s = t0.elapsed().as_secs_f64();
    let [x0, y0, z0, x1, y1, z1] = bounds;
    let moves: usize = ops.iter().map(|o| o.1.len()).sum();
    let feeds: usize = ops.iter().map(|o| o.1.iter().filter(|m| m.kind != MoveKind::Rapid).count()).sum();
    let posed: usize = ops.iter().map(|o| o.1.iter().filter(|m| m.axis.is_some()).count()).sum();
    let nx = ((x1 - x0) / cell).ceil() as usize;
    let ny = ((y1 - y0) / cell).ceil() as usize;
    if ny > 4000 { die(format!("{ny} rows of cells: tile in Y not built")); }
    // stock_sim caps a grid at 4096 cells per axis; a wider stock is cut in X tiles (every column is
    // independent in the height field, and each tile gets the whole program).
    let ntx = nx.div_ceil(4000).max(1);
    let per = nx.div_ceil(ntx);
    let sim_ops: Vec<SimOp> = ops
        .iter()
        .map(|(t, m)| SimOp { moves: m, radius: t.radius, ball: t.ball, cone: t.cone, corner: t.corner })
        .collect();
    let mut tops = vec![f64::NAN; nx * ny];
    let (mut sim_s, mut tops_s) = (0.0, 0.0);
    let mut sim: Result<(), String> = Ok(());
    let mut stocks: Vec<Stock> = Vec::new();
    // --cutting-length: `project` (each tool's project length), `T=mm,T=mm` (per tool number; a tool not named is
    // not checked), or one length in mm for every tool.
    let per_tool: BTreeMap<i64, f64> = get("cutting-length").filter(|s| s.contains('=')).map(|s| s.split(',').map(|kv| {
        let (k, v) = kv.split_once('=').unwrap_or_else(|| die(format!("cutting length {kv}: want T=mm")));
        (k.trim().parse::<i64>().unwrap_or_else(|_| die(format!("tool number {k}"))), num(v.trim()))
    }).collect()).unwrap_or_default();
    let strike_len: Option<Option<f64>> = get("cutting-length").map(|s| if s == "project" || s.contains('=') { None } else { Some(num(&s)) });
    if !per_tool.is_empty() {
        // Every run of a named tool is checked on its first atom (a profile tool's other atoms are its form).
        for (k, c) in cut_lens.iter_mut().enumerate() {
            c.1 = if first_atom.get(k).copied().unwrap_or(true) { per_tool.get(&c.0).copied() } else { None };
        }
    }
    // tool number -> (cutting length, strike moves, worst excess mm, first (op, move), the moves seen across X tiles)
    let mut strikes: BTreeMap<i64, (f64, usize, f64, (usize, usize), std::collections::BTreeSet<(usize, usize)>)> = BTreeMap::new();
    let mut prog = get("progressive").map(|p| Progressive::load(&p, z0, &op_ids));
    for tx in 0..ntx {
        let (i0, i1) = (tx * per, ((tx + 1) * per).min(nx));
        let (tx0, tx1) = (x0 + i0 as f64 * cell, x0 + i1 as f64 * cell);
        let t1 = Instant::now();
        let mut stock = Stock::block(backend, tx0, y0, tx1, y1, z1, cell).unwrap_or_else(|e| die(e.to_string()));
        let shank = get("shank").map(|s| num(&s));
        match (&mut stock, shank) {
            // --shank: the field arm cuts each tool with its real length above the tip (a finite
            // solid), not the world-diagonal flank `Stock::simulate` gives it; a tilted tool's
            // window then stays payable.
            (Stock::Field(f), Some(len)) => {
                let tol = (f.cell() * 0.25).max(1e-9);
                for (t, m) in &ops {
                    let tool = if t.ball {
                        SdfTool::Shanked { r: t.radius, shank: len }
                    } else if let Some(c) = t.corner {
                        SdfTool::BullNose { r: t.radius, corner: c, flank: len.max(c) }
                    } else if let Some(cot) = t.cone {
                        SdfTool::Drill { r: t.radius, cot, flank: len.max(t.radius * cot) }
                    } else {
                        SdfTool::Flat { r: t.radius, flank: len }
                    };
                    for chunk in m.chunks(4096) {
                        if let Err(e) = f.subtract_batch(chunk, &tool, tol) { sim = Err(e.to_string()); break; }
                    }
                }
            }
            // --cutting-length project|<mm>: the column atoms are endless upward (material above the flutes is cut
            // as if the shank were flute). Cut exactly that, and before each cutting move read the stock top at the
            // tip centre and 8 points on 0.9 of the radius, at the move's start, middle and end: material standing
            // more than one cell above tip + cutting length there is a shank strike, recorded per tool. A lower
            // bound (sampled points, not the whole swept disc); never a lid left in the stock.
            // --progressive (alone or with the strike check): the same cut op by op, each copy read after the last
            // atom of every operation.
            (Stock::Column(_), _) if strike_len.is_some() || prog.is_some() => {
                let fixed = strike_len.flatten();
                let last_tile = tx + 1 == ntx;
                if let Some(p) = prog.as_mut() { p.eval(&stock, tx0, tx1, last_tile, BEFORE); }
                'ops: for (oi, (t, m)) in ops.iter().enumerate() {
                    let (tn, own) = cut_lens.get(oi).copied().unwrap_or((-1, None));
                    let len = if strike_len.is_some() { fixed.or(own) } else { None };
                    if strike_len.is_none() {
                        if let Err(e) = stock.simulate(&sim_ops[oi..oi + 1]) { sim = Err(e.to_string()); break 'ops; }
                    }
                    for (k, mv) in m.iter().enumerate().filter(|_| strike_len.is_some()) {
                        if let (Some(len), true) = (len, mv.kind != MoveKind::Rapid) {
                            let mut worst = 0.0f64;
                            for f in [0.0, 0.5, 1.0] {
                                let c = [mv.from.x + (mv.to.x - mv.from.x) * f, mv.from.y + (mv.to.y - mv.from.y) * f,
                                         mv.from.z + (mv.to.z - mv.from.z) * f];
                                for s in 0..9 {
                                    let (dx, dy) = if s == 0 { (0.0, 0.0) } else {
                                        let a = std::f64::consts::TAU * (s - 1) as f64 / 8.0;
                                        (0.9 * t.radius * a.cos(), 0.9 * t.radius * a.sin())
                                    };
                                    let top = stock.top(c[0] + dx, c[1] + dy);
                                    if top.is_finite() { worst = worst.max(top - (c[2] + len)); }
                                }
                            }
                            if worst > cell {
                                let e = strikes.entry(tn).or_insert((len, 0usize, 0.0f64, (oi, k), std::collections::BTreeSet::new()));
                                if e.4.insert((oi, k)) { e.1 += 1; }
                                e.2 = e.2.max(worst);
                            }
                        }
                        let one = [SimOp { moves: std::slice::from_ref(mv), radius: t.radius, ball: t.ball, cone: t.cone, corner: t.corner }];
                        if let Err(e) = stock.simulate(&one) { sim = Err(e.to_string()); break 'ops; }
                    }
                    // The last atom of an operation: read every copy against the stock it leaves.
                    if let (Some(p), Some(id)) = (prog.as_mut(), op_ids.get(oi).copied().flatten()) {
                        if oi + 1 == ops.len() || op_ids.get(oi + 1).copied().flatten() != Some(id) {
                            let at = p.order.iter().position(|x| *x == id).unwrap_or(0) as u32;
                            p.eval(&stock, tx0, tx1, last_tile, at);
                        }
                    }
                }
            }
            _ => {
                if let Err(e) = stock.simulate(&sim_ops) { sim = Err(e.to_string()); }
            }
        }
        sim_s += t1.elapsed().as_secs_f64();
        let t2 = Instant::now();
        for iy in 0..ny {
            for ix in i0..i1 {
                let (cx, cy) = (x0 + (ix as f64 + 0.5) * cell, y0 + (iy as f64 + 0.5) * cell);
                tops[iy * nx + ix] = stock.top(cx, cy);
            }
        }
        tops_s += t2.elapsed().as_secs_f64();
        stocks.push(stock);
    }
    let mut tool_names: Vec<String> = ops
        .iter()
        .map(|(t, _)| format!("r{:.4} ball{} cone{:?} corner{:?}", t.radius, t.ball, t.cone, t.corner))
        .collect();
    tool_names.sort();
    tool_names.dedup();
    let mut out = json!({
        "backend": format!("{backend:?}"), "cell_mm": cell, "bounds": bounds, "ops": ops.len(),
        "moves": moves, "cutting_moves": feeds, "posed_moves": posed, "read_s": read_s, "simulate_s": sim_s,
        "tools": tool_names, "x_tiles": ntx,
    });
    if let Err(e) = sim {
        out["error"] = json!(e);
        println!("{out}");
        std::process::exit(3);
    }
    let a2 = cell * cell;
    let mut removed = 0.0;
    let mut through = 0usize;
    for &t in &tops {
        if t.is_finite() {
            if t < z1 { removed += (z1 - t.max(z0)) * a2; }
            if t <= z0 { through += 1; }
        } else {
            removed += (z1 - z0) * a2;
            through += 1;
        }
    }
    out["tops_s"] = json!(tops_s);
    out["grid"] = json!([nx, ny]);
    if let Some(p) = get("out-closed-stl") {
        let m: Option<Vec<f64>> = opt.get("matrix").map(|v| v[0].split(',').map(|s| num(s.trim())).collect());
        let t6 = Instant::now();
        out["closed_mesh"] = write_skyline(&p, &tops, nx, ny, bounds, cell, m.as_deref());
        out["closed_mesh"]["seconds"] = json!(t6.elapsed().as_secs_f64());
    }
    if let Some(p) = get("move-lines") {
        let per: Vec<Value> = op_lines.iter().enumerate()
            .map(|(oi, l)| json!({"op": oi, "tool": cut_lens.get(oi).map(|c| c.0), "nc_lines": l})).collect();
        std::fs::write(&p, json!({"basis": "per op, in cut order: the 1-based NC line of each move (an arc chorded into several moves repeats its line)", "ops": per}).to_string())
            .unwrap_or_else(|e| die(format!("{p}: {e}")));
        out["move_lines"] = json!(p);
    }
    if strike_len.is_some() {
        let rows: Vec<Value> = strikes.iter().map(|(t, (len, n, w, first, _))| json!({
            "tool": t, "cutting_length_mm": len, "strike_moves": n, "worst_above_flutes_mm": w,
            "first_op": first.0, "first_move": first.1,
            "first_nc_line": op_lines.get(first.0).and_then(|l| l.get(first.1))})).collect();
        let checked: std::collections::BTreeSet<i64> = cut_lens.iter().filter(|c| c.1.is_some()).map(|c| c.0).collect();
        let mut unchecked: Vec<i64> = cut_lens.iter().map(|c| c.0).filter(|t| !checked.contains(t)).collect();
        unchecked.dedup();
        out["shank_strike"] = json!({
            "basis": "stock top at the tip centre and 8 points on 0.9 radius, at each cutting move's start, middle and end, before the move; material more than one cell above tip + cutting length is a strike; a lower bound",
            "cutting_length": get("cutting-length"), "tools_struck": rows, "tools_not_checked": unchecked,
            "stock": "the column engine cuts above the flutes as if the shank were flute: no lid is left"});
    }
    if !profiled.is_empty() {
        out["profile_tools"] = json!({"tools": profiled, "rim_copies": PHI,
            "basis": "profile bands as a flat disc per band plus its cone swept on the disc rim (inner approximation within r0 (1 - cos(pi / rim_copies)) radially)"});
    }
    if let Some(p) = &prog { out["progressive"] = p.report(); }
    out["removed_mm3"] = json!(removed);
    out["remaining_mm3"] = json!((x1 - x0) * (y1 - y0) * (z1 - z0) - removed);
    out["through_columns"] = json!(through);
    if let Some(r) = get("ref-stl") {
        let t3 = Instant::now();
        let tris = read_stl(&r, stl_k);
        // Nudged off the lattice by microns: our cell centres can fall exactly on the edges of a mesh made on
        // the same 0.254 grid, where the facet test admits neither neighbour (a false "air" column).
        let rt = part_surface(&tris, x0 + 1.37e-3 * cell, y0 + 2.11e-3 * cell, cell, nx, ny);
        let (mut n, mut both_air, mut air_mismatch) = (0usize, 0usize, 0usize);
        let mut d: Vec<f64> = Vec::with_capacity(nx * ny);
        let mut vol_ref = 0.0;
        for k in 0..nx * ny {
            // Only columns whose centre lies inside the box (the ceil lattice overshoots the far edges).
            let (cx, cy) = (x0 + ((k % nx) as f64 + 0.5) * cell, y0 + ((k / nx) as f64 + 0.5) * cell);
            if cx > x1 || cy > y1 { continue; }
            let ours = if tops[k].is_finite() && tops[k] > z0 + 1e-9 { Some(tops[k].min(z1)) } else { None };
            let theirs = if rt[k].is_finite() && rt[k] > z0 + 1e-9 { Some(rt[k].min(z1)) } else { None };
            if let Some(t) = theirs { vol_ref += (t - z0) * a2; }
            match (ours, theirs) {
                (Some(a), Some(b)) => { n += 1; d.push(a - b); }
                (None, None) => both_air += 1,
                _ => {
                    air_mismatch += 1;
                    if std::env::var_os("JJ_DEBUG").is_some() && air_mismatch % 50000 == 1 {
                        eprintln!("air mismatch at ({cx:.3}, {cy:.3}): ours {} ref {}", tops[k], rt[k]);
                    }
                }
            }
        }
        let mut ad: Vec<f64> = d.iter().map(|v| v.abs()).collect();
        ad.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let q = |p: f64| if ad.is_empty() { f64::NAN } else { ad[((ad.len() - 1) as f64 * p) as usize] };
        let within = |t: f64| ad.iter().filter(|v| **v <= t).count() as f64 / ad.len().max(1) as f64;
        let ours_vol: f64 = tops.iter().map(|t| if t.is_finite() && *t > z0 { (t.min(z1) - z0) * a2 } else { 0.0 }).sum();
        out["vs_ref"] = json!({
            "ref_facets": tris.len(), "columns_both_material": n, "columns_both_air": both_air,
            "columns_air_mismatch": air_mismatch, "mean_signed_mm": d.iter().sum::<f64>() / d.len().max(1) as f64,
            "abs_p50_mm": q(0.5), "abs_p99_mm": q(0.99), "abs_max_mm": q(1.0),
            "within_0.01mm": within(0.01), "within_0.05mm": within(0.05), "within_cell": within(cell),
            "ref_column_volume_mm3": vol_ref, "our_column_volume_mm3": ours_vol,
            "seconds": t3.elapsed().as_secs_f64(),
        });
    }
    if let Some(p) = get("part-stl") {
        let t4 = Instant::now();
        let tol = get("tol").map(|s| num(&s)).unwrap_or(0.0254);
        let part = read_stl(&p, stl_k);
        let dvs: Result<Vec<_>, _> = stocks.iter().map(|st| st.deviate(&part, tol)).collect();
        match dvs.map(|v| v.into_iter().reduce(|mut a, b| { a.rest_cells += b.rest_cells; a.gouge_cells += b.gouge_cells; a.rest_volume += b.rest_volume; a.gouge_volume += b.gouge_volume; a.max_rest = a.max_rest.max(b.max_rest); a.max_gouge = a.max_gouge.max(b.max_gouge); a }).unwrap()) {
            Ok(dv) => out["vs_part"] = json!({
                "tol_mm": tol, "part_facets": part.len(), "rest_cells": dv.rest_cells, "gouge_cells": dv.gouge_cells,
                "rest_mm3": dv.rest_volume, "gouge_mm3": dv.gouge_volume, "max_rest_mm": dv.max_rest,
                "max_gouge_mm": dv.max_gouge, "clean": dv.is_clean(), "seconds": t4.elapsed().as_secs_f64(),
            }),
            Err(e) => out["vs_part"] = json!({"error": e.to_string()}),
        }
    }
    if let Some(p) = get("out-stl") {
        let t5 = Instant::now();
        match stocks[0].field().filter(|_| stocks.len() == 1) {
            Some(f) => match iso::extract(f) {
                Ok(tris) => {
                    write_stl(&p, &tris);
                    out["mesh"] = json!({"facets": tris.len(), "open_edges": iso::open_edges(&tris), "seconds": t5.elapsed().as_secs_f64()});
                }
                Err(e) => out["mesh"] = json!({"error": e.to_string()}),
            },
            None => out["mesh"] = json!({"error": "column arm has no closed mesh export; use --backend field"}),
        }
    }
    out["total_s"] = json!(t0.elapsed().as_secs_f64());
    println!("{out}");
}

#[cfg(test)]
mod progressive_tests {
    use super::*;

    fn pass(stock: &mut Stock, xs: &[f64], z: f64) {
        let moves: Vec<Move> = xs.iter().map(|&x| Move::new(MoveKind::Feed, Point3::new(x, -3.0, z), Point3::new(x, 23.0, z), Some(500.0))).collect();
        stock.simulate(&[SimOp { moves: &moves, radius: 2.0, ball: false, cone: None, corner: None }]).unwrap();
    }

    /// A block 20 x 20 x 10 mm; the part's top face at z 5 (face 1, 54 samples of 1 mm2) and a wall at x 15 facing +x
    /// (face 2, 27 samples). Operation 10 faces to z 5, 20 cuts the wall to z 2, 30 dips to z 4 over x 5-10.
    #[test]
    fn three_operations_read_in_order() {
        let mut s = Vec::new();
        for x in [6.0, 7.0, 8.0, 12.0, 13.0, 14.0] { for y in 6..15 { s.push([x as f32, y as f32, 5.0, 0.0, 0.0, 1.0, 1.0, f32::from_bits(1)]); } }
        for z in [2.5f32, 3.5, 4.5] { for y in 6..15 { s.push([15.0, y as f32, z, 1.0, 0.0, 0.0, 1.0, f32::from_bits(2)]); } }
        let n = s.len();
        let mut p = Progressive { line: 0.3, z0: 0.0, order: vec![10, 20, 30],
            copies: vec![PartCopy { key: "a".into(), s, past: vec![NEVER; n], on: vec![NEVER; n], unmeasured: vec![false; n] }] };
        let mut stock = Stock::block(Backend::Column, 0.0, 0.0, 20.0, 20.0, 10.0, 0.1).unwrap();
        p.eval(&stock, 0.0, 20.0, true, BEFORE);
        pass(&mut stock, &[-1.0, 1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0, 21.0], 5.0);
        p.eval(&stock, 0.0, 20.0, true, 0);
        pass(&mut stock, &[17.0, 19.0, 21.0], 2.0);
        p.eval(&stock, 0.0, 20.0, true, 1);
        pass(&mut stock, &[7.0, 8.0], 4.0);
        p.eval(&stock, 0.0, 20.0, true, 2);
        let r = p.report();
        let rows = &r["copies"]["a"]["per_toolpath"];
        let f = |i: usize, k: &str| rows[i][k].as_f64().unwrap();
        assert_eq!((f(0, "surface_newly_on_cut_mm2"), f(1, "surface_newly_on_cut_mm2"), f(2, "surface_newly_on_cut_mm2")), (54.0, 27.0, 0.0));
        assert_eq!((f(0, "surface_newly_past_mm2"), f(1, "surface_newly_past_mm2"), f(2, "surface_newly_past_mm2")), (0.0, 0.0, 27.0));
        assert_eq!((f(0, "faces_first_cut_to_size"), f(1, "faces_first_cut_to_size"), f(2, "faces_first_cut_to_size")), (1.0, 1.0, 0.0));
        assert_eq!((f(0, "faces_newly_entered"), f(2, "faces_newly_entered")), (0.0, 1.0));
        assert_eq!(r["copies"]["a"]["unmeasured_mm2"].as_f64(), Some(0.0));
        assert_eq!(r["copies"]["a"]["before_first_toolpath_on_cut_mm2"].as_f64(), Some(0.0));
    }
}
