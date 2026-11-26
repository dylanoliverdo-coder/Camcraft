# S9: Contour on an open sketch chain, the cutter to a chosen side

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: RS-274/NGC cutter radius compensation (G41 left of
the path, G42 right of it; NIST IR 6556, Kramer, Proctor, Messina 2000); P. Smid, *CNC Programming Handbook*,
3rd ed., Industrial Press 2007 (cutter radius offset on open profiles, climb milling with the cutter left of a
right-hand spindle's path).

## Today

A 2D Contour on a DXF profile rides each OPEN sketch chain on its centerline only. In
`crates/app/src/operation/contour_sketch_pick.rs` (`collect_chains`, ~228-240) an open chain with a per-chain side
override, or under an operation side other than Auto, is refused by name ("side compensation is not allowed on
open chains"). Inside and Outside cannot be read on an open chain that bounds no area, and a DXF profile has no
solid to pick the side against.

CamCraft already offsets an open chain to one side for a Contour under an open boundary
(`crates/app/src/operation/contour_open.rs`, `open_run`): the offset is the chosen side of the disc swept along
the chain (`Region::sweep`), a chain that crosses itself or an offset that folds is refused by name, and the run is
handed on with the material on its right for climb under M3 (#1555). That code needs the operation's own area to
decide which side is Inside (`pick_side_left`); a sketch chain has none.

## The change

An open sketch chain can be cut with the cutter one radius to its LEFT or RIGHT, read along the chain's direction.

New setting, per chain (on the existing per-chain override that already carries `side` and `reverse`):
`open_side`, one of `none` (default), `left`, `right`.

- The chain's direction is its stored order, turned round when the chain's `reverse` is set.
- `left` / `right`: the tool centre rides the curve at distance `d = r + a` from the chain, on that side of the
  direction, where `r` is the tool radius and `a` the operation's stock to leave. This is the same offset law
  `open_run` uses (the chosen side of the swept disc's boundary): straight runs move parallel, a convex corner on
  the tool's side becomes an arc of radius `d` about the corner, a concave corner becomes the point where the two
  offset runs meet.
- Travel: with climb on, the run is travelled so the material (the side opposite the tool) is on the right of
  travel under M3: the chain's direction when the tool is `left`, against it when the tool is `right`. Climb off
  reverses that.
- Every other part of the operation is today's: levels, wall passes (spec S1), leads, links, heights.
- `none` is today's behaviour, unchanged, including today's refusals for an operation side other than Auto.
- An open chain with `open_side` set is cut to its side whatever the operation's side is; the operation's side
  keeps governing the closed chains only.

### Refusals

Refuse by name when the path is made, as `open_run` does: a chain that crosses or touches itself; an offset run
that folds onto itself or breaks into pieces (name the chain and `d`). Refuse `open_side` on a closed chain when the
override is set: closed chains take Inside or Outside.

## Checks (synthetic geometry only)

1. Defaults: every existing test and golden file gives the same toolpath; an open chain with no `open_side` rides
   its centerline; an operation side of Inside on a profile with an open chain and no override is refused as today.
2. Straight chain (0,0) to (50,0), tool diameter 10, stock to leave 0, `open_side` left, climb on: the tool centre
   runs on y = +5 from x = 0 to x = 50, travelling +x.
3. The same with `right`: y = -5, travelling -x (from x = 50 to x = 0).
4. The same as check 2 with `reverse` set: the direction is -x, so left is y = -5, travelling -x.
5. Stock to leave 0.5 on check 2: y = +5.5.
6. L chain (0,0) to (40,0) to (40,30), diameter 10, `left` (the concave side): y = 5 from x = 0 to 35, then x = 35
   from y = 5 to 30; the corner is the single point (35, 5).
7. The same chain, `right` (the convex side): y = -5 from x = 0 to 40, an arc of radius 5 about (40,0) from
   (40,-5) to (45,0), then x = 45 from y = 0 to 30; travelled from (45,30) back to (0,-5) with climb on.
8. A U chain (0,30) to (0,0) to (8,0) to (8,30), diameter 10, the tool on the inside of the U: refused by name
   (the offset folds: 8 mm between the walls, 10 mm of cutter).
9. A chain crossing itself (a figure of four segments through one point): refused by name.
10. A closed rectangle chain with `open_side` set: refused by name when the override is set.
11. Stock simulator: a 60 x 40 x 10 mm block, its top at 0, chain of check 2 lying along y = 0 inside the block,
    `left`, cut to -3: the cut removes a band 10 mm wide with its edge on y = 0, on the +y side only, and leaves
    the material at y < 0 standing.
12. Project file: a project saved before the setting loads with `none`; a project with `left` or `right` saves and
    loads back the same value.
