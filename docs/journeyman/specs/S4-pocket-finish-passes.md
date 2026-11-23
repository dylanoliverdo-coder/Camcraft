# S4: Pocket, wall finishing passes with their own direction

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: M. Held, *On the Computational Geometry of
Pocket Machining*, LNCS 500, Springer 1991 (contour-parallel passes round the boundary and the islands);
P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007 (climb and conventional milling).

## Today

A 2D pocket (`crates/motion/src/machining2d/pocket.rs`, `PocketParams` at `:32`) has one direction,
`climb`. Its strategies clear the area and, for the zigzag, close with one wall ring that follows `climb`
(`pocket_zigzag.rs:58-65`). There are no separate finishing passes on the walls.

## The change

After the clearing passes of a level, the tool rides finishing passes round every wall of the pocket
(the outer boundary and each island), with a direction of their own.

New fields on the Pocket operation:

| Field | Type, default | Meaning |
|---|---|---|
| `finish_passes` | integer >= 0, default 0 | finishing passes round the walls |
| `finish_spacing` | mm > 0, default 0 (unused while `finish_passes` = 0) | distance between two finishing passes |
| `spring_passes` | integer >= 0, default 0 | repeats of the final wall pass |
| `finish_direction` | `same`, `climb` or `conventional`, default `same` | `same` follows `climb`; the other two set the direction of the finishing and spring passes only |
| `finish_level` | `every` or `last`, default `every` | the levels where the finishing and spring passes run, as in S1 |

### Offsets

Let `w = r + s` (tool radius plus `stock_to_leave`), `n_f = finish_passes`, `d_f = finish_spacing`.

- With `n_f` > 0, the clearing passes leave `w + n_f * d_f` on every wall instead of `w`.
- Finishing pass `j` (j = 1..n_f, cut order) rides at `w + (n_f - j) * d_f` from each wall.
- Each spring pass rides at `w`.
- The floor does not change.

### Direction

`climb` and `conventional` use the meaning that CamCraft's `climb` field already has. On an outer
boundary and on an island the same choice gives opposite turning senses (the material is on opposite
sides). The roughing passes keep `climb`.

### Order, links and entry

- `finish_level` works as in S1: `every` runs the finishing passes at each level after its clearing;
  `last` runs them only at the last level, and refuses by name when the tool's flute length is shorter
  than the wall height.
- Inside one level: the clearing passes, then the finishing passes of every wall, then the spring
  passes. The links between them follow today's link law (`Links`): stay down where the link is proved
  inside the area already cut, else lift and enter again.
- With `n_f` = 0 the zigzag's closing wall ring is as today.

### Refusals

Refuse by name, when the command is created: `finish_passes` > 0 with `finish_spacing` <= 0; a
non-finite value.

## Checks (synthetic geometry only)

1. Defaults: every existing pocket test and golden file gives the same toolpath.
2. A 50 x 40 mm pocket with a 10 x 10 mm island in the middle, a 6 mm end mill, one level, `n_f` = 2,
   `d_f` = 0.1, `finish_direction = same`: the clearing passes stay 3.2 mm from every wall; two rings
   round the boundary and two round the island at 3.1 and 3.0 mm; the swept footprint matches the pocket
   area (`shape_mask`: no overcut, no leftover).
3. The same with `climb` = true and `finish_direction = conventional`: the finishing rings turn the
   other way round each wall; the clearing passes do not change.
4. A circular pocket of radius 20 mm, `climb` = true, `n_f` = 1: the wall ring is all arcs of one sense
   (G3 when the spindle turns clockwise).
5. `spring_passes` = 1: one more ring at 3.0 mm round each wall.
6. Project file: a project saved before the fields loads with the defaults; a project with the fields
   saves and loads back the same values.
