# S6: Drill, chamfer each hole to a width

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: elementary cone geometry; P. Smid, *CNC
Programming Handbook*, 3rd ed., Industrial Press 2007 (spot drilling and chamfering with a pointed tool).

## Today

- The Drill operation's spot cycle feeds to a fixed `SPOT_DEPTH` of 2 mm, or to a `bottom_z` that the user
  sets (`crates/motion/src/drilling/hole.rs:167`, `cycle.rs:78`). The user must work out the depth that
  gives a wanted chamfer by hand, hole by hole.
- The Drill operation mounts only a drill, a tap or a boring bar: adding one with a chamfer mill is refused.
- No tool record carries a tip diameter: the drill's point and the chamfer mill's cone are both modelled
  as sharp.
- The gouge judgement reads the hole's wall as its finished surface, so a spot or chamfer wider than the
  hole holds the post ("cuts past the finish surface").

## The change

The user gives the chamfer width; the operation computes the tip depth of each hole from the hole's
diameter and the tool's cone.

New field on the Drill operation: `chamfer_width`, mm, default 0 (off: today's depth rules).

For each hole with `chamfer_width = c` > 0:

- `D_h` is the hole's diameter at its top (the diameter the hole recognition reads). A point location with
  no diameter has `D_h` = 0.
- The chamfer's diameter at the hole top is `D_c = D_h + 2c`.
- The tool is a cone-tipped tool: a drill (its point angle) or a chamfer mill (its cone angle). Call the
  included angle `theta` (0 < theta < 180 degrees). With `c` > 0 the Drill operation accepts either kind.
- The tool states its tip diameter `t`: a new tool field, mm, default 0 (a sharp point), read by both
  kinds. A tool saved before the field loads with 0. A real chamfer or spot tool usually has a ground
  flat; `t` is that flat's diameter.
- The tip depth below the hole's top is `h = (D_c - t) / (2 * tan(theta / 2))`.
- The cycle is not changed: the tip goes to `z_top - h` with the cycle the operation selects today.
  - An explicitly chosen cycle stays as chosen. A positive width does not require the spot cycle, and
    no cycle is refused for it.
  - Auto resolves by today's law (by the tool's kind and the depth); the width does not make Auto pick
    the spot cycle.
  - A dwell is written only when the operation sets one; the width adds none.
  - The retract and the R plane do not change.
- `bottom_z` is not used for that hole while `chamfer_width` > 0.
- The chamfer is an intended cut. The operation's gouge judgement and the post's gate read the cone,
  out to `D_c` at the hole's top and down to the tip at `z_top - h`, as part of the hole's finished
  shape, not as a cut past the finished surface. Any cut outside that cone (deeper than the tip, or
  wider than `D_c` at the top) is judged as today.

### Refusals

Refuse the hole by name (the hole's position in the message), and never drill it to a guessed depth,
when:
- the tool has no cone (a flat end), or `theta` is outside (0, 180) degrees;
- `D_c` is larger than the cone's largest diameter (the tool body diameter): the cone cannot cut that
  chamfer;
- `D_c <= t` (the chamfer is narrower than the tip).

Refuse by name, when the command is created: a negative or non-finite `chamfer_width` (the operation);
a negative or non-finite tip diameter, or a tip diameter not smaller than the body diameter (the tool).

## Checks (synthetic geometry only)

1. Defaults: every existing drilling test and golden file gives the same toolpath, and every existing
   tool loads with a tip diameter of 0.
2. A plate with four holes of 6.6 mm diameter, top at z 0, a 90 degree drill of 12 mm body (sharp),
   `c` = 0.5: `D_c` = 7.6 mm, each tip at z -3.8 mm.
3. The same holes with a 90 degree chamfer mill of 12 mm body whose tip diameter is 0.5 mm, set on the
   tool: tips at z -3.55 mm. The depth comes through the operation and its tool, not only through a
   depth function.
4. A 120 degree tool, `c` = 0.5, the same holes: tips at z -2.194 mm (7.6 / (2 tan 60 degrees)).
5. A point location (no diameter), `c` = 1.0, the 90 degree tool: tip at z -1.0 mm.
6. A hole of 11.5 mm with `c` = 0.5 and the 12 mm tool: refused by name (`D_c` = 12.5 > 12); the other
   holes of the operation are still drilled.
7. In the stock simulator, check 2's chamfer: the top edge of each hole is cut out to 7.6 mm diameter
   (to the simulator's cell size), and no cut reaches below z -3.8 mm. Check 3's chamfer, cut with a
   simulated tool that has the 0.5 mm flat, is also 7.6 mm at the top.
8. Cycles, on check 2's holes: with the peck cycle chosen, the motion is the peck cycle's, ending at
   z -3.8; with a dwell set, the dwell is kept; with Auto, the cycle is the one today's Auto law gives
   for that tool and a 3.8 mm depth, with no dwell added.
9. The gate: the programs of checks 2 and 3 pass the operation's gouge judgement and post without a
   hold. On the same plate, a second Drill operation with `c` = 0 and `bottom_z` = -4.5 mm (deeper than
   the chamfer's tip) is still held as today.
10. Project file: a project saved before the fields loads with 0 (width and tip diameter); a project with
    them saves and loads back the same values.
