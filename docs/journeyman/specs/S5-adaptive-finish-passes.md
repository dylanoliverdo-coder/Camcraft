# S5: 2D Adaptive, wall finishing passes after clearing

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: M. Held, *On the Computational Geometry of
Pocket Machining*, LNCS 500, Springer 1991; M. Stori and P. K. Wright, "Constant engagement tool path
generation for convex geometries", J. Manuf. Systems 19 (2000) 172-184 (why a constant-engagement
clearing leaves a scalloped wall that a contour-parallel pass finishes).

## Today

2D Adaptive (`OpType::Adaptive2D`, `crates/motion/src/machining2d/adaptive/`, parameters at
`crates/app/src/operation/types/params.rs:234`) clears to `stock_to_leave` from the walls with
constant-engagement motion. It has no finishing pass on the walls, so the wall keeps the marks of the
clearing arcs.

## The change

After the clearing of a level, the tool rides contour-parallel finishing passes round every wall that the
operation's area touches: the boundary of an inside area, the outline of an outside (boss) area, and each
island.

New fields on the 2D Adaptive operation: `finish_passes` (integer >= 0, default 0), `finish_spacing`
(mm > 0, default 0, unused while `finish_passes` = 0), `spring_passes` (integer >= 0, default 0) and
`finish_level` (`every` or `last`, default `every`), with the meanings of S4.

- With `n_f = finish_passes` > 0, the clearing leaves `w + n_f * d_f` on every wall, where `w = r + s`.
- Finishing pass `j` (j = 1..n_f) rides at `w + (n_f - j) * d_f`; spring passes ride at `w`.
- The finishing passes turn in the direction that `climb` gives each wall (opposite senses on an outer
  boundary and on an island), as S4's `same`.
- The floor does not change.
- The first finishing pass of a wall starts from the clearing's last position by today's link law: stay
  down where the link is proved inside the area already cut, else lift to the retract height and enter
  again with the operation's entry (outside the material where the area allows it).
- `finish_level = last` refuses by name when the flute length is shorter than the wall height.

### Refusals

Refuse by name, when the command is created: `finish_passes` > 0 with `finish_spacing` <= 0; a
non-finite value.

## Checks (synthetic geometry only)

1. Defaults: every existing 2D Adaptive test and golden file gives the same toolpath.
2. An inside area, 60 x 40 mm, 10 mm end mill, `stock_to_leave` 0, one level, `n_f` = 2, `d_f` = 0.25:
   the clearing stays at least 5.5 mm from the walls; two finishing rings at 5.25 and 5.0 mm; the swept
   footprint matches the area (`shape_mask`).
3. An outside (boss) area: a 30 mm diameter boss on an 80 x 80 mm stock, the passes of check 2: two rings
   round the boss at 20.25 and 20.0 mm radius; the turning sense is the opposite of check 2's boundary
   rings.
4. `spring_passes` = 1 on check 2: a third ring at 5.0 mm.
5. Project file: a project saved before the fields loads with the defaults; a project with the fields
   saves and loads back the same values.
