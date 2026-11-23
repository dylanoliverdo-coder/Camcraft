//! journeyman_job - bake-off driver: cut one Journeyman group's stock with
//! the `stock_sim` seam and compare it with another engine's stock mesh.
//!
//! Inputs, either:
//!   --camotics <job.camotics>   the CAMotics project our cut lane writes
//!                               (tools, workpiece box, resolution, the NC
//!                               file); the NC is the same G-code subset
//!                               CAMotics cuts (G0/G1/G2/G3, G17/18/19,
//!                               M6 Tn per move, metric).
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
use camcraft_sim::stock_sim::{iso, part_surface, Backend, SimOp, Stock, FIELD_BAND};
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
    eprintln!("journeyman_job: {msg}");
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

/// Our post's G-code -> moves per tool number, in program order.
fn read_nc(path: &str) -> Vec<(i64, Vec<Move>)> {
    let src = std::fs::read_to_string(path).unwrap_or_else(|e| die(format!("cannot read {path}: {e}")));
    let mut ops: Vec<(i64, Vec<Move>)> = Vec::new();
    let (mut pos, mut have) = ([0.0f64; 3], false);
    let (mut tool, mut modal, mut plane) = (0i64, 0u8, 17u8);
    let mut feed = 1000.0;
    let push = |tool: i64, m: Move, ops: &mut Vec<(i64, Vec<Move>)>| match ops.last_mut() {
        Some((t, v)) if *t == tool => v.push(m),
        _ => ops.push((tool, vec![m])),
    };
    for raw in src.lines() {
        let line = match raw.find('(') { Some(i) => &raw[..i], None => raw }.trim();
        if line.is_empty() { continue; }
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
                push(tool, Move::new(kind, p3(pos), p3(to), fd).with_arc(c, ccw), &mut ops);
            } else {
                let (pl, c) = match plane {
                    18 => ((2usize, 0usize, 1usize), [pos[2] + k, pos[0] + i]),
                    19 => ((1, 2, 0), [pos[1] + j, pos[2] + k]),
                    _ => ((0, 1, 2), [pos[0] + i, pos[1] + j]),
                };
                // G18 views from +Y: Z->X is CCW (RS-274), handled by the (z, x) pair.
                let mut prev = pos;
                for p in arc_points(pos, to, c, ccw, pl, 0.002) {
                    push(tool, Move::new(kind, p3(prev), p3(p), fd), &mut ops);
                    prev = p;
                }
            }
        } else if to != pos {
            push(tool, Move::new(kind, p3(pos), p3(to), fd), &mut ops);
        }
        pos = to;
    }
    ops
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
        for (t, mv) in read_nc(&nc) {
            match camotics_tool(&proj["tools"][t.to_string()]) {
                Some(ts) => ops.push((ts, mv)),
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
                    let mut m = Move::new(kind, a, p, fd);
                    let ax = [v[3], v[4], v[5]];
                    let upright = ax[0].abs() < 1e-12 && ax[1].abs() < 1e-12 && ax[2] > 0.0;
                    if !upright { m = m.with_axis(ax); }
                    mv.push(m);
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
    for tx in 0..ntx {
        let (i0, i1) = (tx * per, ((tx + 1) * per).min(nx));
        let (tx0, tx1) = (x0 + i0 as f64 * cell, x0 + i1 as f64 * cell);
        let t1 = Instant::now();
        let mut stock = Stock::block(backend, tx0, y0, tx1, y1, z1, cell).unwrap_or_else(|e| die(e.to_string()));
        if let Err(e) = stock.simulate(&sim_ops) { sim = Err(e.to_string()); }
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
