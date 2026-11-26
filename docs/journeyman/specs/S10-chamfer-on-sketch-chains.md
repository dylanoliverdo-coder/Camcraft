# S10: 2D Chamfer on the chains of a DXF profile

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: machine-tool handbook chamfer-mill geometry (a cone
whose generatrix lies on the chamfer face; *Machinery's Handbook*, 31st ed., Industrial Press 2020, chamfering
with countersink and chamfer cutters); P. Smid, *CNC Programming Handbook*, 3rd ed., 2007 (edge breaking along a
programmed profile).

## Today

A 2D Chamfer on a DXF profile is refused when it is generated: `crates/app/src/sketch_model.rs`
(`generation_problem`, ~184-206) lets only Contour, the wire kinds and the slot kinds generate from a profile's
chains; every other kind gets "a toolpath from the chains of a DXF profile is not generated yet".

The chamfer kernel itself (`crates/motion/src/machining2d/chamfer.rs`, `ChamferParams`) rides each picked closed
loop once at the constant height `top_z - width * tan(angle_deg)`, the tool centre on the loop (the cone's virtual
tip at the corner), entered by the axial dive; the app's doors (`crates/app/src/operation/generate.rs`, ~2518-2560)
check the cone (`180 - 2 * angle_deg`), the reach (`diameter >= 2 * width`) and the flute length. `side` does not
move the ride; it picks the travel direction for climb (#1555).

## The change

A 2D Chamfer generates from the chains of a DXF profile, closed and open.

1. Closed chains: each is a loop the kernel rides exactly as a picked loop today. The side of each chain is
   today's Contour rule for sketch chains: the operation's side, or the per-chain override, or under Auto the
   chain's nesting (an outer loop is a boss edge, a hole is a bore edge).
2. Open chains: the tool centre rides the chain itself at the same constant height, from one end to the other,
   entered by the same axial dive at the first end and retracted at the last. The side the cutter body stands
   on is the per-chain `open_side` of spec S9 (`left` or `right` of the chain's direction); the material is on
   the other side. With climb on, the material is on the right of travel under M3, as on a closed loop. An open
   chain with `open_side` = `none` is refused by name (a chamfer on an open edge needs the side of the material).
3. `top_z` is the operation's Top of Part, as today; the heights, feeds and doors are today's.
4. Each chain is one ride. The closed chains go in today's loop order (`chamfer`, ~113: largest loop first), then
   the open chains in the profile's order, with today's link between rides (retract to the hop plane, rapid, dive).

### Refusals

Today's doors stand unchanged (the cone, the reach, the flute). New, when the path is made: an open chain without
an `open_side`, named; a profile with no chains, named.

## Checks (synthetic geometry only)

1. Defaults: every existing test and golden file gives the same toolpath; a 2D Chamfer on a solid's edge is
   unchanged.
2. Closed rectangle chain 40 x 20 mm, top at 0, width 0.5, angle 45, 90-degree chamfer mill of diameter 6:
   one ride on the rectangle at z = -0.5, the same moves a picked rectangle loop of a solid gives today.
3. Open straight chain (0,0) to (40,0), `open_side` left, same tool and width: the tool centre runs on y = 0 from
   x = 0 to x = 40 at z = -0.5, travelling +x with climb on; `right`: from x = 40 to x = 0.
4. The same open chain, `open_side` none: refused by name.
5. Angle 30 with a 120-degree chamfer mill, width 0.5: z = -0.5 * tan(30 degrees) = -0.2887 (to 4 places).
6. Stock simulator: a block from (0,0) to (60,40), 10 mm thick, top at 0; an open chain along its +y edge from
   (0,40) to (60,40), `open_side` left (left of +x is +y, so the cutter stands in air), width 1, angle 45: the
   edge is broken by a 45-degree face 1 mm wide and 1 mm deep along the whole edge; nothing below z = -1 and
   nothing further than 1 mm from the edge is touched.
7. Two closed chains, one inside the other, Auto side: the outer loop rides as a boss edge and the inner as a bore
   edge (the travel directions of #1555).
8. Project file: a 2D Chamfer on a profile saves and loads back the same; a project saved before this change loads
   unchanged.
