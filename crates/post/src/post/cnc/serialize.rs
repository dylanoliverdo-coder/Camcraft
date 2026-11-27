//! Text emission for the CamCraft intermediate motion records.
//! Provenance: our own CNC intermediate format and integer geometry lattice.

use super::{Record, cycle_name};
use crate::cl::{CompSide, MoveKind};
use serde_json::Value;

/// µm-integer formatting, the `.cl` golden law (`scale=0.001mm`):
/// value times a thousand, nearest, written as an integer.
fn q(v: f64) -> String {
    if !v.is_finite() {
        return "nan".into();
    }
    ((v * 1000.0).round() as i64).to_string()
}

/// The writer side: the stream as text, `records` in driver order. The
/// canonical v1 spelling keys every coordinate (`X 1000`, spaced, as
/// `docs/CNC-INTERMEDIATE.md` specs it); the parser also accepts the
/// joined (`X1000`) and legacy positional forms.
pub fn serialize(records: &[Record]) -> String {
    serialize_with_motion_precision(records, false)
}

/// In-process posting may retain fractional microns for a post that
/// explicitly supports and requests them. Exported CNC text keeps its
/// integer lattice and the arc short-move guard stays active.
pub(in crate::post) fn serialize_with_motion_precision(records: &[Record], fine: bool) -> String {
    let length = |mm: f64| {
        if !fine || !mm.is_finite() {
            return q(mm);
        }
        let nm = crate::post::cnc_arc_lattice::motion_units(mm, true);
        let magnitude = nm.unsigned_abs();
        let sign = if nm < 0 { "-" } else { "" };
        format!("{sign}{}.{:03}", magnitude / 1000, magnitude % 1000)
    };
    let mut s = String::from("; HSMW-CNC v1\n");
    // The true position before each record (#3442): an arc's start, for
    // the lattice law of `cnc_arc_lattice`. None where a record leaves it
    // unknown here; an arc after it is written as it is.
    let mut pos: Option<[f64; 3]> = None;
    for r in records {
        match r {
            Record::Param { name, value } => {
                let v = match value {
                    Value::String(t) => t.clone(),
                    Value::Number(n) if n.is_f64() => format!("{:.6}", n.as_f64().unwrap_or(0.0)),
                    other => other.to_string(),
                };
                s.push_str(&format!("PARAM {name} {v}\n"));
            }
            Record::Warn { text } => s.push_str(&format!("WARN {text}\n")),
            Record::Tool { number, name } => s.push_str(&format!("TOOL {number} {name}\n")),
            Record::Section { begin } => {
                s.push_str(if *begin { "SECTION\n" } else { "ENDSECTION\n" });
            }
            Record::Move(m) => {
                let short = crate::post::cnc_arc_lattice::short_arc_on_motion_grid(pos, m, fine);
                pos = Some([m.x, m.y, m.z]);
                let f = m
                    .feed
                    .map(|f| format!(" F {}", (f * 1000.0).round() as i64))
                    .unwrap_or_default();
                // Commanded angles (stage D): milli-degrees on the
                // same lattice law as the microns above - the angle
                // a record states is the angle it parses back to.
                let abc = m
                    .abc
                    .map(|[a, b, c]| format!(" A {} B {} C {}", q(a), q(b), q(c)))
                    .unwrap_or_default();
                // #3442, #3474: a short arc whose ends round to one
                // micrometre, or to one ray from the rounded centre, is
                // written as the straight move it is on the lattice.
                match m.center.filter(|_| !short) {
                    // The center words name the plane (M11.2 A): I J
                    // is XY - byte-identical to every pre-M11.2
                    // stream - I K is XZ, J K is YZ.
                    Some((cx, cy)) => {
                        let (a, b) = m.plane.center_letters();
                        s.push_str(&format!(
                            "Arc {} X {} Y {} Z {}{f}{abc} {a} {} {b} {}\n",
                            if m.ccw { "CCW" } else { "CW" },
                            length(m.x),
                            length(m.y),
                            length(m.z),
                            length(cx),
                            length(cy),
                        ));
                    }
                    None => s.push_str(&format!(
                        "{} X {} Y {} Z {}{f}{abc}\n",
                        match m.kind {
                            _ if short => "Feed",
                            MoveKind::Rapid => "Rapid",
                            MoveKind::Feed => "Feed",
                            MoveKind::Plunge => "Plunge",
                        },
                        length(m.x),
                        length(m.y),
                        length(m.z),
                    )),
                }
            }
            Record::Cycle(c) => {
                pos = Some([c.x, c.y, c.r_z]);
                let f = c
                    .feed
                    .map(|f| format!(" F {}", (f * 1000.0).round() as i64))
                    .unwrap_or_default();
                s.push_str(&format!(
                    "CYCLE {} X {} Y {} Z {} R {} ENTRY {} Q {} P {}{} AT {}\n",
                    cycle_name(c.kind),
                    length(c.x),
                    length(c.y),
                    length(c.bottom_z),
                    length(c.r_z),
                    length(c.entry_z),
                    length(c.peck),
                    (c.dwell * 1000.0).round() as i64,
                    f,
                    c.first_move,
                ))
            }
            Record::Part { loading, advance } => {
                let adv = if *loading && *advance > 0.0 {
                    format!(" ADVANCE {}", q(*advance))
                } else {
                    String::new()
                };
                s.push_str(&format!(
                    "PART {}{adv}\n",
                    if *loading { "LOAD" } else { "UNLOAD" }
                ));
            }
            Record::Transfer { from, to } => {
                s.push_str(&format!("PART TRANSFER {from} {to}\n"));
            }
            Record::Probe { tol_mm } => {
                // A length like the loader's ADVANCE: the µm integer,
                // the lattice law - not a parameter real.
                s.push_str(&format!("PROBE {}\n", q(*tol_mm)));
            }
            Record::Comp { side, radius_mm, wear } => {
                // The radius rides as microns like every length here;
                // a trailing WEAR (#38) marks the centre-path variant.
                let w = if *wear { " WEAR" } else { "" };
                s.push_str(&match side {
                    Some(CompSide::Left) => format!("COMP LEFT {}{w}\n", q(*radius_mm)),
                    Some(CompSide::Right) => format!("COMP RIGHT {}{w}\n", q(*radius_mm)),
                    None => "COMP OFF\n".to_string(),
                });
            }
            Record::Thread(Some(t)) => s.push_str(&t.line()),
            Record::Thread(None) => s.push_str("THREAD END\n"),
            Record::Touch(t) => {
                pos = None;
                s.push_str(&crate::post::probe_rec::write_touch(t))
            }
            Record::Found(f) => s.push_str(&crate::post::probe_rec::write_found(f)),
            Record::Retreat(r) => {
                pos = None;
                s.push_str(&crate::post::probe_rec::write_retreat(r))
            }
            Record::Guard(g) => {
                pos = None;
                s.push_str(&crate::post::probe_rec::write_guard(g))
            }
            Record::Plasma(p) => s.push_str(&p.line()),
            Record::Torch(t) => s.push_str(&t.line()),
            Record::Waterjet(w) => s.push_str(&w.line()),
            Record::Jet(j) => s.push_str(&j.line()),
        }
    }
    s
}
