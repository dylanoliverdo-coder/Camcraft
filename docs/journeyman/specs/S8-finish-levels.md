# S8: Contour, Pocket and Face, finishing levels at a smaller stepdown

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: P. Smid, *CNC Programming Handbook*, 3rd ed.,
Industrial Press 2007 (roughing and finishing depth of cut); M. P. Groover, *Fundamentals of Modern
Manufacturing*, Wiley (a light finishing depth after heavier roughing depths, for floor finish and
accuracy).

## Today

The levels of a 2D operation come from one law: `cut_levels_under` (`crates/motion/src/machining2d/levels.rs`,
~99) with the stepdown, and the equal-steps choice (`equal_steps.rs`, `cut_levels_for`, which reads
`Approach::equal_steps`). Contour and Pocket read it through `Approach`; Face calls `cut_levels_under` with
its own fields (`face.rs`, ~181). Every level below the first is one stepdown (or one equal step) under the
level above, down to the bottom. The last levels cannot be made shallower than the others.

## The change

The last levels of the operation can be finishing levels at their own, smaller step.

New settings (one group): `finish_levels` (integer >= 0, default 0) and `finish_stepdown` (mm >= 0, default 0),
on Contour and Pocket (beside the equal-steps choice they already share) and on Face.

With `n = finish_levels` > 0, `s = finish_stepdown`, the final level `z_b` (the bottom; with spec S2's floor
allowance, the level S2 states) and the top of the cut as today:

1. The roughing levels are today's levels (stepdown, equal-steps choice, material top) with
   `z_r = z_b + n * s` in place of the bottom. `z_r` is the last roughing level.
2. Then the `n` finishing levels: `z_r - s, z_r - 2s, ..., z_b`.
3. With `s = 0`, the levels are today's, then the final level `z_b` is cut `n` more times (a full repeat of
   the level's passes at the same height).

Every pass of a level (wall passes, finishing wall passes, leads, links) is the same at a finishing level as
at a roughing level; only the heights change.

### Refusals

Refuse by name, when the command is created: a negative or non-finite `finish_stepdown`. When the path is made:
`n * s` at or above the depth of the cut (top of the cut minus `z_b`): the finishing levels would leave no
roughing level; name both values.

## Checks (synthetic geometry only)

1. Defaults: every existing test and golden file gives the same toolpath.
2. Levels only: top of the cut 0, `z_b` -3.81, stepdown 1.905, equal steps on, `n` 1, `s` 0.508:
   levels -1.651, -3.302, -3.81.
3. The same with equal steps off: -1.905, -3.302, -3.81.
4. Top 0, `z_b` -6, stepdown 2, equal steps off, `n` 3, `s` 0.25: -2, -4, -5.25, -5.5, -5.75, -6.
5. `n` 1, `s` 0, top 0, `z_b` -4, stepdown 2: -2, -4, -4 (the last level twice).
6. `n` 2, `s` 2, top 0, `z_b` -3: refused by name.
7. A Contour round a 30 x 20 mm boss, check 2's levels: the passes at -3.81 are the passes at -3.302 moved
   down 0.508 mm, move for move.
8. A Face of an 80 x 60 mm stock, its top at 0, cut to -1, stepdown 0.6, `n` 1, `s` 0.3, equal steps off:
   `z_r` -0.7; levels -0.6, -0.7 (today's law down to -0.7), then -1.0.
9. With S2's floor allowance 0.3 on check 4: `z_b` is -5.7 and the levels are -2, -4, -4.95, -5.2, -5.45, -5.7.
10. Project file: a project saved before the settings loads with the defaults; a project with them saves and
    loads back the same values.
