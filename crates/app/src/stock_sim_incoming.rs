//! Posting's comparison with an explicitly acknowledged incoming stock
//! on the same upright, single-setup lattice. Unchanged deficits are
//! visible in the posting receipt. Any deepened column still goes to the full gate.
//! Provenance: a before/after height comparison on our stock simulator.

use flecs_ecs::prelude::World;

use super::{Deviation, Dexel, Planned, stock_grid};
use crate::geometry::Point3;

pub(super) fn report(
    world: &World,
    planned: &[Planned],
    after: &Dexel,
    dev: &mut Deviation,
) -> Option<String> {
    if dev.gouge_cells == 0 && !planned.is_empty() {
        return None;
    }
    // The setup and lattice the walk started from (`carry::walk`): a program
    // of touch probes only plans nothing, yet its uncut stock is judged on
    // the span-sized lattice an infinite radius gives (#592), and so here.
    let setup = planned.first().and_then(|p| p.setup.as_deref());
    let judged = crate::setup::effective(world, setup);
    let probes = unplanned_probe_setups(world)?;
    if planned.is_empty() {
        // Touch probes only, so nothing is cut: every deficit of the stock
        // they machine against is inherited. An empty plan is judged on the
        // first setup's stock; probes of one other setup are judged here on
        // their own setup's stock, and only that stock's acknowledgement
        // counts. Probes across setups keep the full gate.
        let [own] = probes.iter().collect::<Vec<_>>()[..] else {
            return None;
        };
        if *own != judged {
            return own_setup_note(world, own.as_deref(), dev);
        }
        if dev.gouge_cells == 0 {
            return None;
        }
    } else if probes.iter().any(|s| *s != judged) {
        return None;
    }
    let radius = planned.iter().map(|op| op.radius).fold(f64::INFINITY, f64::min);
    note(world, setup, radius, after, dev)
}

/// The located note for the acknowledged deficits of `setup`'s stock that
/// `after` leaves unchanged, taking them out of `dev`.
fn note(world: &World, setup: Option<&str>, radius: f64, after: &Dexel, dev: &mut Deviation) -> Option<String> {
    let source = crate::stock_source::incoming_stock_source(world, setup)?;
    let before = stock_grid(world, setup, radius).ok()?;
    let inherited = separate_unchanged(&before, after, dev)?;
    let units = crate::units::of(world);
    let depth = crate::units::len(inherited.worst, units, 2);
    // Located like the gouge line, so a person can find the worst cell.
    let at = inherited.at.map_or_else(String::new, |q| {
        format!(
            " at ({}, {}, z {})",
            crate::units::len(q.x, units, 2),
            crate::units::len(q.y, units, 2),
            crate::units::len(q.z, units, 2)
        )
    });
    Some(format!(
        "note: incoming stock '{source}' already has {} deficient cells, worst {depth} {}{at}. This program leaves those cells at their incoming heights",
        inherited.cells,
        units.suffix()
    ))
}

/// A probe-only program in a setup the empty plan was not judged on: its
/// own uncut stock, judged on the same span-sized lattice and against the
/// same part as the ordinary verdict, takes the verdict's place - only when
/// that stock is acknowledged (`note` refuses otherwise), so the ordinary
/// verdict is untouched outside this opt-in.
fn own_setup_note(world: &World, setup: Option<&str>, dev: &mut Deviation) -> Option<String> {
    crate::stock_source::incoming_stock_source(world, setup)?;
    let uncut = stock_grid(world, setup, f64::INFINITY).ok()?;
    let facets = super::part_facets(world).ok()?;
    let tol = crate::model_fidelity::program_tol(world).ok()?;
    let mut own = super::deviate(&uncut, &facets, tol).ok()?;
    let line = note(world, setup, f64::INFINITY, &uncut, &mut own)?;
    *dev = own;
    Some(line)
}

/// The effective setups of the program's touch probes, which the column
/// plan leaves out (#108); `None` keeps the full gate. The receipt's
/// "unchanged" reads only the planned cuts, and a torch or a wire removes
/// material the column never sees (#670), so a program with one gets no
/// note.
fn unplanned_probe_setups(world: &World) -> Option<std::collections::BTreeSet<Option<String>>> {
    use crate::tools::ToolKind;
    let mut setups = std::collections::BTreeSet::new();
    for (_, e) in crate::posting::program_ops(world).ok()? {
        let op = crate::model::operation_of(world, &e)?;
        let kind = crate::model::find_by_name(world, &op.tool).and_then(|t| crate::model::tool_of(world, &t))?.kind;
        match kind {
            ToolKind::Torch | ToolKind::Wire => return None,
            ToolKind::Probe => {
                setups.insert(crate::setup::effective(world, op.setup.as_deref()));
            }
            _ => {}
        }
    }
    Some(setups)
}

#[derive(Default)]
struct Gouges {
    cells: usize,
    volume: f64,
    worst: f64,
    at: Option<Point3>,
}

impl Gouges {
    fn add(&mut self, depth: f64, area: f64, at: Point3) {
        self.cells += 1;
        self.volume += depth * area;
        if depth > self.worst {
            self.worst = depth;
            self.at = Some(at);
        }
    }
}

fn separate_unchanged(before: &Dexel, after: &Dexel, dev: &mut Deviation) -> Option<Gouges> {
    let lattice = |d: &Dexel| (d.nx, d.ny, d.cell, d.x0, d.y0);
    if lattice(before) != lattice(after)
        || (dev.cells_x, dev.cells_y, dev.cell) != (after.nx, after.ny, after.cell)
        || dev.dev.len() != after.nx * after.ny
    {
        return None;
    }

    let mut inherited = Gouges::default();
    let mut machined = Gouges::default();
    let area = after.cell * after.cell;
    for (k, &deviation) in dev.dev.iter().enumerate() {
        if !deviation.is_finite() || deviation >= -dev.tol {
            continue;
        }
        let z = after.top_at(k);
        let xy = after.center(k % after.nx, k / after.nx);
        let found = if z.is_finite() && z == before.top_at(k) {
            &mut inherited
        } else {
            &mut machined
        };
        found.add(-deviation, area, Point3::new(xy.x, xy.y, z));
    }

    // Only adjust a verdict completely accounted for on this lattice.
    if inherited.cells == 0 || inherited.cells + machined.cells != dev.gouge_cells {
        return None;
    }
    dev.gouge_cells = machined.cells;
    dev.gouge_volume = machined.volume;
    dev.max_gouge = machined.worst;
    dev.max_gouge_at = machined.at;
    Some(inherited)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::cube;
    use crate::stock_sim::deviate;

    #[test]
    fn unchanged_deficits_do_not_hide_fresh_or_deepened_gouges() {
        let mut incoming = Dexel::block(0.0, 0.0, 4.0, 4.0, 10.0, 1.0).unwrap();
        incoming.lower(0, 5.0);
        incoming.lower(1, 8.0);
        let mut finished = incoming.clone();
        finished.lower(1, 7.0);
        finished.lower(2, 9.0);
        let part = cube(Point3::ZERO, Point3::new(4.0, 4.0, 10.0));
        let mut dev = deviate(&finished, &part.facets, 0.01).unwrap();
        assert_eq!(dev.gouge_cells, 3);
        let inherited = separate_unchanged(&incoming, &finished, &mut dev).unwrap();
        assert_eq!((inherited.cells, inherited.worst), (1, 5.0));
        assert_eq!((dev.gouge_cells, dev.gouge_volume, dev.max_gouge), (2, 4.0, 3.0));
        assert_eq!(dev.max_gouge_at, Some(Point3::new(1.5, 0.5, 7.0)));
    }

    #[test]
    fn a_different_lattice_keeps_the_full_verdict() {
        let mut incoming = Dexel::block(0.0, 0.0, 4.0, 4.0, 10.0, 1.0).unwrap();
        incoming.lower(0, 5.0);
        let mut shifted = incoming.clone();
        shifted.translate_origin(1.0, 0.0, 0.0);
        let part = cube(Point3::ZERO, Point3::new(5.0, 4.0, 10.0));
        let mut dev = deviate(&shifted, &part.facets, 0.01).unwrap();
        assert_eq!(dev.gouge_cells, 1);
        assert!(separate_unchanged(&incoming, &shifted, &mut dev).is_none());
        assert_eq!(dev.gouge_cells, 1);
    }
}
