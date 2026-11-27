//! A floor on the extraction lattice meshes to a valid solid (#4107).
//!
//! Marching tetrahedra put a crossing a few picometres from a sample whose
//! field value is almost zero (a cut floor on the lattice of cell centres).
//! The facets that share that sample kept two corners closer than the
//! 1e-9 weld, so after the weld they collapsed and their neighbours' edges
//! sat on three or more facets. `open_edges` counts only odd edges, so the
//! check here counts every edge.
//!
//! Gate map (#4107): covers sim::stock_sim::iso (tet's crossing, the clipped
//! streamed export). Provenance: our own fixtures; synthetic sizes.

use std::collections::HashMap;

use camcraft_sim::cl::{Move, MoveKind};
use camcraft_sim::geometry::{Aabb, Point3, Triangle};
use camcraft_sim::stock_sim::iso::{extract_clipped, extract_clipped_streamed, extract_with, IsoMethod};
use camcraft_sim::stock_sim::{Backend, SimOp, Stock, FIELD_BAND};

const CELL: f64 = 0.5;

/// A 30 x 30 x 20 mm block faced by four passes of a flat tool 6 mm across
/// at height `z`. The lattice samples stand at (k + 0.5) * CELL: 15.25 is
/// one, 15.1 is not.
fn faced(z: f64) -> Stock {
    let mut s = Stock::block(Backend::Field { band: FIELD_BAND }, 0.0, 0.0, 30.0, 30.0, 20.0, CELL)
        .unwrap_or_else(|e| panic!("block: {e}"));
    let moves: Vec<Move> = (0..4)
        .map(|k| {
            let y = 8.0 + 4.0 * k as f64;
            Move::new(MoveKind::Feed, Point3::new(4.0, y, z), Point3::new(26.0, y, z), Some(500.0))
        })
        .collect();
    s.simulate(&[SimOp { moves: &moves, radius: 3.0, ball: false, cone: None, corner: None }])
        .unwrap_or_else(|e| panic!("cut: {e}"));
    s
}

/// Facets that lose a corner at the 1e-9 weld, and edges that are not on
/// exactly two facets after it.
fn defects(tris: &[Triangle]) -> (usize, usize) {
    let key = |p: Point3| [(p.x * 1e9).round() as i64, (p.y * 1e9).round() as i64, (p.z * 1e9).round() as i64];
    let mut collapsed = 0;
    let mut uses: HashMap<([i64; 3], [i64; 3]), u32> = HashMap::new();
    for t in tris {
        let k = [key(t.a), key(t.b), key(t.c)];
        if k[0] == k[1] || k[1] == k[2] || k[0] == k[2] {
            collapsed += 1;
        }
        for (a, b) in [(k[0], k[1]), (k[1], k[2]), (k[2], k[0])] {
            *uses.entry(if a <= b { (a, b) } else { (b, a) }).or_insert(0) += 1;
        }
    }
    (collapsed, uses.values().filter(|&&n| n != 2).count())
}

#[test]
fn a_floor_on_the_lattice_meshes_to_a_valid_solid_by_marching_tetrahedra() {
    let s = faced(15.25);
    let f = s.field().expect("a field");
    assert!(f.distance(Point3::new(15.25, 15.25, 15.25)).abs() < 1e-9, "the floor stands on a lattice sample");
    let dense = extract_with(f, f.cell(), IsoMethod::Marching).unwrap_or_else(|e| panic!("mesh: {e}"));
    assert_eq!(defects(&dense), (0, 0), "collapsed facets, edges not on two facets");
    let clip = Aabb { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(30.0, 30.0, 20.0) };
    let streamed = extract_clipped_streamed(f, f.cell(), clip).unwrap_or_else(|e| panic!("streamed: {e}"));
    assert_eq!(defects(&streamed), (0, 0), "the clipped export: collapsed facets, edges not on two facets");
}

#[test]
fn a_floor_off_the_lattice_stays_valid() {
    let s = faced(15.1);
    let f = s.field().expect("a field");
    let dense = extract_with(f, f.cell(), IsoMethod::Marching).unwrap_or_else(|e| panic!("mesh: {e}"));
    assert_eq!(defects(&dense), (0, 0));
}

#[test]
fn the_streamed_export_of_a_floor_on_the_lattice_is_the_dense_marching_mesh() {
    let s = faced(15.25);
    let f = s.field().expect("a field");
    let clip = Aabb { min: Point3::new(2.0, 2.0, 10.0), max: Point3::new(28.0, 28.0, 19.0) };
    let dense = extract_clipped(f, f.cell(), IsoMethod::Marching, clip).unwrap_or_else(|e| panic!("dense: {e}"));
    let streamed = extract_clipped_streamed(f, f.cell(), clip).unwrap_or_else(|e| panic!("streamed: {e}"));
    assert_eq!(dense.len(), streamed.len(), "same lattice, same tetrahedra");
    for (a, b) in dense.iter().zip(&streamed) {
        assert_eq!([a.a, a.b, a.c], [b.a, b.b, b.c]);
    }
    assert_eq!(defects(&streamed), (0, 0));
}
