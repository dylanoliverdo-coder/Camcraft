# S6: Drill, chamfer each hole to a width

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: elementary cone geometry; P. Smid, *CNC
Programming Handbook*, 3rd ed., Industrial Press 2007 (spot drilling and chamfering with a pointed tool).

## Today

The Drill operation's spot cycle feeds to a fixed `SPOT_DEPTH` of 2 mm, or to a `bottom_z` that the user
sets (`crates/motion/src/drilling/hole.rs:167`, `cycle.rs:78`). The user must work out the depth that
gives a wanted chamfer by hand, hole by hole.

## The change

The user gives the chamfer width; the operation computes the tip depth of each hole from the hole's
diameter and the tool's cone.

New field on the Drill operation: `chamfer_width`, mm, default 0 (off: today's depth rules).

For each hole with `chamfer_width = c` > 0:

- `D_h` is the hole's diameter at its top (the diameter the hole recognition reads). A point location with
  no diameter has `D_h` = 0.
- The chamfer's diameter at the hole top is `D_c = D_h + 2c`.
- The tool is a pointed (cone-tipped) tool: included point angle `theta` (0 < theta < 180 degrees) and
  tip diameter `t` (0 for a sharp point).
- The tip depth below the hole's top is `h = (D_c - t) / (2 * tan(theta / 2))`.
- The cycle feeds the tip to `z_top - h` with the cycle the operation already uses (the spot cycle, or
  a dwell cycle when a dwell is set). The retract and the R plane do not change.
- `bottom_z` is not used for that hole while `chamfer_width` > 0.

### Refusals

Refuse the hole by name (the hole's position in the message), and never drill it to a guessed depth,
when:
- the tool has no cone (a flat end), or `theta` is outside (0, 180) degrees;
- `D_c` is larger than the cone's largest diameter (the tool body diameter): the cone cannot cut that
  chamfer;
- `D_c <= t` (the chamfer is narrower than the tip).

Refuse the operation by name, when the command is created: a negative or non-finite `chamfer_width`.

## Checks (synthetic geometry only)

1. Defaults: every existing drilling test and golden file gives the same toolpath.
2. A plate with four holes of 6.6 mm diameter, top at z 0, a 90 degree sharp spot drill of 12 mm body,
   `c` = 0.5: `D_c` = 7.6 mm, each tip at z -3.8 mm.
3. The same tool with a 0.5 mm tip diameter: tips at z -3.55 mm.
4. A 120 degree tool, `c` = 0.5, the same holes: tips at z -2.194 mm (7.6 / (2 tan 60 degrees)).
5. A point location (no diameter), `c` = 1.0, the 90 degree tool: tip at z -1.0 mm.
6. A hole of 11.5 mm with `c` = 0.5 and the 12 mm tool: refused by name (`D_c` = 12.5 > 12); the other
   holes of the operation are still drilled.
7. In the stock simulator, check 2's chamfer: the top edge of each hole is cut out to 7.6 mm diameter
   (to the simulator's cell size), and no cut reaches below z -3.8 mm.
8. Project file: a project saved before the field loads with 0; a project with the field saves and loads
   back the same value.
