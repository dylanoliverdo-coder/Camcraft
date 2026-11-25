# S2: Pocket, Contour and Face, floor allowance

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: P. Smid, *CNC Programming Handbook*, 3rd ed.,
Industrial Press 2007 (finishing allowances on walls and floors).

## Today

`stock_to_leave` is a radial allowance only (`crates/motion/src/machining2d/pocket.rs:259`,
`contour_params.rs:76-78`), and a negative value is refused (`pocket.rs:148`,
`contour_params.rs:165`). The Face operation has no allowance (`face.rs:28-64`). The deepest level of each
of these operations is `bottom_z`.

## The change

A separate axial allowance on the floor.

New field on Pocket, Contour and Face (2D): `floor_stock`, mm, default 0. Negative values are allowed.

- Let `a = floor_stock`. The deepest level of the operation is at `bottom_z + a`. A positive `a` leaves
  that much material on the floor; a negative `a` cuts that much below `bottom_z`.
- The levels run from `top_z` down to `bottom_z + a` by the operation's existing step-down law
  (`stepdown`, and `equal_steps` when it is set). The last level is exactly at `bottom_z + a`.
- The walls do not change: the radial allowance (`stock_to_leave`) and its rules are as today. The
  existing refusal of a negative radial allowance stays.
- Every level, the moved last level included, is entered with the operation's entry, as today.
- The safety checks do not change. A negative `a` that takes the tool into the part model is caught by the
  existing verify and gouge checks, which block the post as today. This spec adds no exception to them.

### Refusals

Refuse by name, when the command is created:
- `bottom_z + a >= top_z` (the allowance reaches the top: nothing is left to cut);
- a non-finite `a`.

## Checks (synthetic geometry only)

1. Defaults: every existing Pocket, Contour and Face test and golden file gives the same toolpath.
2. A 40 x 30 mm rectangular pocket, 0 to -10 mm, `stepdown` 2.5, `a` = 0.3: levels at -2.5, -5.0, -7.5
   and -9.7 mm. The walls are the same as with `a` = 0.
3. The same with `equal_steps` on: four equal levels down to -9.7 mm (2.425 mm each).
4. The same pocket open through the stock bottom (the stock ends at -10), `a` = -0.2: the last level at
   -10.2 mm, in air below the stock.
5. Each level of checks 2 to 4 starts with the operation's entry (for example a helix when the entry is
   a helix).
6. A Contour on a 60 x 40 mm boss and a Face on a 100 x 80 mm stock, `a` = 0.5 and `a` = -0.5: the last
   level moves by `a`, the walls of the contour do not move.
7. `a` = 10 on the pocket of check 2: refused by name.
8. Project file: a project saved before the field loads with `a` = 0; a project with the field saves and
   loads back the same value.
