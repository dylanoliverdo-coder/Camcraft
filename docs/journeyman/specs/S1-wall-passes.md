# S1: Contour, several wall passes per level

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: M. Held, *On the Computational Geometry of
Pocket Machining*, LNCS 500, Springer 1991 (offset curves of a chain); R. T. Farouki and C. A. Neff,
"Analytic properties of plane offset curves", CAGD 7 (1990) 83-99.

## Today

A 2D contour (`OpType::Contour`, `crates/motion/src/machining2d/contour.rs`, parameters
`crates/motion/src/machining2d/contour_params.rs:74`) rides one offset of its chain at each level. The
tool centre is at `w = r + s` from the wall, where `r` is the tool radius and `s` is `stock_to_leave`.

## The change

At each level the tool rides several offsets of the same chain. It starts with the offset farthest from
the wall (the most material) and ends with the final offset `w`.

New fields on the Contour operation (motion parameters, `OpParams::Contour`, the project file and the
operation panel):

| Field | Type, default | Meaning |
|---|---|---|
| `rough_passes` | integer >= 1, default 1 | roughing passes at each level; 1 is today's single pass |
| `rough_spacing` | mm > 0, default 0 (unused while `rough_passes` = 1) | distance between two roughing passes |
| `finish_passes` | integer >= 0, default 0 | finishing passes after the roughing passes |
| `finish_spacing` | mm > 0, default 0 (unused while `finish_passes` = 0) | distance between two finishing passes, and between the last roughing pass and the first finishing pass |
| `spring_passes` | integer >= 0, default 0 | repeats of the final pass, at the final offset |
| `finish_level` | `every` or `last`, default `every` | the levels where the finishing and spring passes run |

### Offsets

Let `n_r = rough_passes`, `d_r = rough_spacing`, `n_f = finish_passes`, `d_f = finish_spacing`.
Number the passes in cut order.

- Roughing pass `i` (i = 1..n_r) rides at `w + n_f * d_f + (n_r - i) * d_r`.
- Finishing pass `j` (j = 1..n_f) rides at `w + (n_f - j) * d_f`.
- Each spring pass rides at `w`, after the last finishing pass (or after the last roughing pass when
  `n_f` = 0).

Example: `r` = 5, `s` = 0, `n_r` = 2, `d_r` = 2.54, `n_f` = 1, `d_f` = 0.05 gives three passes at 7.59,
5.05 and 5.00 mm from the wall, in that order.

### Order and levels

- `finish_level = every`: at each level, all roughing passes, then all finishing passes, then all spring
  passes.
- `finish_level = last`: at each level above the last one, only the roughing passes run (the last one at
  `w + n_f * d_f`). At the last level, the roughing passes, then the finishing and spring passes. A
  finishing pass at the last level then cuts the whole wall height with the side of the tool. If the
  tool's flute length is shorter than the wall height (`top_z - bottom_z`), refuse the operation by name.
- The order of levels and chains follows today's contour rules (`contour_link.rs`): this spec changes
  only what happens inside one level of one chain.

### Leads, links and entry

- Each pass is a full pass of the chain with its own lead in and lead out, as the operation's lead setting
  makes them today, tangent to that pass's own offset.
- Between two passes of one level, use today's link law between levels (`Links`, `ladder_links.rs`): stay
  at depth where the link is proved inside the area already cut, else lift to the retract height and
  enter again with the operation's entry.

### Open chains

On an open chain, each pass starts and ends on the offset of the chain's two end points, at that pass's
own offset: the start is the end point moved along the chain's end normal by the pass's offset. There is
no extra move square to the chain at either end. The lead attaches tangent at that point.

### Closed chains

Each pass starts at the point of its own offset nearest the start point of the first pass, so that the
starts of the passes stand on one line normal to the wall.

### Passes that do not fit

On an inside closed chain, a large offset can be empty (the material there is narrower than the pass).
Drop that pass and name it in a diagnostic (chain, level, pass number, offset). Never drop a pass in
silence. If the final pass `w` itself is empty, the existing refusal for an empty contour applies.

### Refusals

Refuse by name, when the command is created:
- `rough_passes` > 1 and `rough_spacing` <= 0;
- `finish_passes` > 0 and `finish_spacing` <= 0;
- a non-finite value in any field.

## Checks (synthetic geometry only)

1. Defaults: every existing contour test and golden file gives the same toolpath as before.
2. A 60 x 40 mm rectangular boss, side outside, a 10 mm flat end mill, one level 0 to -5 mm,
   `n_r` = 2, `d_r` = 2.54, `n_f` = 1, `d_f` = 0.05: three passes at 7.59, 5.05 and 5.00 mm from the
   straight walls (to 1e-6 mm), in that order. The swept footprint (`shape_mask`) cuts nothing inside the
   wall and leaves nothing outside it.
3. An open chain (a 30 mm radius quarter arc, then a 20 mm straight leg), side outside, the passes of
   check 2: the first and the last cut point of each pass lie on the end normals of the chain at that
   pass's offset (to 1e-6 mm). No move at a pass end is square to the chain.
4. `spring_passes` = 1 on check 2: a fourth pass at 5.00 mm, the same path as the third.
5. `finish_level = last`, three levels of 2 mm: levels 1 and 2 carry two passes (7.59, 5.05); level 3
   carries three (7.59, 5.05, 5.00). The same with a flute length of 4 mm: refused by name.
6. A circle of radius 12 mm, side inside, a 10 mm end mill, `n_r` = 3, `d_r` = 4: the pass at 13 mm is
   dropped with a named diagnostic; the passes at 9 and 5 mm are cut.
7. Project file: a project saved before the fields loads with the defaults; a project with the fields
   saves and loads back the same values.
