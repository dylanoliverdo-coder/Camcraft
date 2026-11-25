# S7: 2D descents, a stated height where the rapid becomes a feed

Provenance: Journeyman spec writer, from geometric and kinematic relationships checked in a stock
simulator. No vendor parameters, text or code. Literature: P. Smid, *CNC Programming Handbook*, 3rd ed.,
Industrial Press 2007 (rapid approach to a clearance above the work, feed from there); RS-274 / ISO 6983
(G0 and G1).

## Today

- Closed-area 2D ladders (`crates/motion/src/machining2d/descent_dive.rs`, `feed_down`, ~20-36): a descent
  rapids to `hz = min(mat_top + plunge_height, max(hop_z, mat_top))`, then feeds. `mat_top` is the material
  top at that level: the stock top on the first level, the level above on later levels. `plunge_height` is
  `Approach::plunge_height` (`crates/motion/src/approach.rs`, default 2 mm). `plunge_height = 0` feeds the
  whole way from the traverse height.
- Open-run contours (`contour_chain.rs`, the dive at the lead-in, ~136-142) feed the whole way down from the
  traverse height at every level, whatever `plunge_height` says.
- So "rapid down to the material top, then feed" cannot be stated, and no descent can switch at a stated
  absolute Z.

## The change

One new field on `Approach`, `feed_from`, with three values:

- `auto` (the default): today's laws, unchanged (closed ladders as above; open runs feed from the traverse
  height).
- `material { gap }`: every descent of the operation, open-run dives included, rapids to
  `hz = min(mat_top + gap, z_t)` and feeds from there. `gap` is mm, `>= 0`; `gap = 0` switches exactly at the
  material top. `z_t` is the height the tool stands at over the descent point (the traverse it came on).
- `absolute { z }`: every descent rapids to `hz = min(z, z_t)` and feeds from there. `z` is a height in the
  operation's frame.

In both stated modes:

- The rapid never ends at or below the level it enters: `hz <= z_cut` is refused by name (see Refusals).
- If `hz >= z_t`, the whole descent is fed (no rapid).
- `mat_top` for an open run: the stock top on the first level; on a later level, the previous level of the
  same run (the lead-in of the previous level cleared the column at the dive point).
- `mat_top` for a closed ladder: as today.
- The feed after the rapid is the operation's plunge feed, as today.
- This spec changes plunge descents only. Helix and ramp entries keep their own start heights.

### Refusals

Refuse by name:
- when the command is created: a negative or non-finite `gap`; a non-finite `z`;
- when the path is made: an `absolute` descent whose `hz` is below the material top over a column the path
  has not proved cleared (a rapid into material); name the descent's position and both heights.
- when the path is made: `hz <= z_cut` (the rapid would reach the cut level); name the descent's position.

## Checks (synthetic geometry only)

1. Defaults: every existing test and golden file gives the same toolpath.
2. An open run (a straight line 50 mm long, the stock on one side), one level, stock top 0, bottom -5, the
   traverse at 5, a linear lead, `material { gap: 1 }`: the dive is a rapid from z 5 to z 1, then a plunge
   feed from z 1 to z -5. With `material { gap: 0 }`: a rapid to z 0, then the feed.
3. The same open run, three levels (0 to -6, stepdown 2), `material { gap: 1 }`: level 2's dive rapids to
   z -1 and feeds to -4; level 3's dive rapids to z -3 and feeds to -6.
4. A 40 x 30 mm pocket, two levels (-2.5, -5), plunge entry, `absolute { z: 0.5 }`: every descent rapids to
   z 0.5 and feeds from there.
5. The pocket of check 4, stock top 0, `absolute { z: -1 }`: the first descent is refused by name (a rapid into
   material below z 0).
6. `absolute { z: 20 }` with the traverse at 5: every descent is fed the whole way from z 5.
7. Project file: a project saved before the field loads with `auto`; a project with each mode saves and loads
   back the same value.
