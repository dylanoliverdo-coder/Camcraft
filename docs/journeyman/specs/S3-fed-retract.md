# S3: 2D operations, fed retract

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: P. Smid, *CNC Programming Handbook*, 3rd ed.,
Industrial Press 2007 (feed rate and rapid motion out of a cut); RS-274 / ISO 6983 (G0 and G1).

## Today

Every move up out of the cut is a rapid (`MoveKind::Rapid`, for example the lifts in
`crates/motion/src/machining2d/ladder_links.rs:106` and `:183`). The descent into a level is a rapid down
to the plunge height over the material, then a feed (`descent_dive.rs:27-32`, `feed_down`): the switch
height is `hz = min(mat_top + plunge_height, max(hop_z, mat_top))`, where `mat_top` is the material top
at that level.

## The change

A lift out of the cut can be fed up to the switch height, then rapid on.

New field on the 2D milling operations that lift between passes and levels (Pocket, Contour, Face,
Slot, Adaptive2D, Offset2D, Circular, Bore): `retract_feed`, mm/min, default 0.

- `retract_feed = 0`: every lift is a rapid, as today.
- `retract_feed > 0`: each straight-up move that leaves the cut becomes two moves:
  1. a feed straight up at `retract_feed`, from the cut level to the switch height `hz` of that level
     (the same height where the descent into that level changed from rapid to feed);
  2. a rapid from `hz` to the target of the lift (the retract height, the clearance height or the next
     level's traverse height).
  If the lift ends at or below `hz`, the whole lift is fed.
- "Leaves the cut" means a vertical move that starts at a cutting level: the lift at the end of a pass,
  of a level, of a chain and of the operation. It does not mean the small lifts that keep the tool down
  (a micro lift, or a lift inside a proved keep-down link); those do not change.
- Cut moves, entries, leads and links that stay down do not change.
- Canned drilling cycles do not change: the cycle owns its retract.
- The post writes the fed part as a feed move with its own feed word, then the rapid.

### Refusals

Refuse by name, when the command is created: a negative or non-finite `retract_feed`.

## Checks (synthetic geometry only)

1. Defaults: every existing test and golden file gives the same toolpath.
2. A 40 x 30 mm pocket, two levels (-2.5 and -5.0 mm), `plunge_height` 1, `retract_feed` 2540, the
   links at the retract height: every lift that starts at a cutting level is a feed at 2540 mm/min that
   ends at that level's switch height, then a rapid. No rapid starts below a switch height. The cut moves
   are the same as with `retract_feed` = 0, move for move.
3. The final lift of the operation: a feed to the switch height, then the rapid to the clearance height.
4. A Contour with a keep-down link between two passes: the keep-down link does not change.
5. The posted program of check 2 (any shipped mill post): the fed part is a G1 with an F word equal to
   2540 mm/min in the program's units; the next move is a G0.
6. Project file: a project saved before the field loads with 0; a project with the field saves and loads
   back the same value.
