# Authored straight-edge contour entry

An optional ramp belongs to one selected closed outside DXF chain. Its
start is a cutter-center seam in sketch coordinates. A unit direction
selects one straight span of the compensated single-level ring. The
generator descends on that span, finishes one lap, and exits on the
normal lead. Walking an ordinary floor lap after the descent would
recut the ramp at floor depth.

The normal lead is clipped against the cutter-inflated union of the
selected exteriors. Exterior contours must be separated by swept-cutter
clearance. Every exterior in an authored operation needs its own ramp.
Contained bores retain their normal behavior. Open chains, tabs,
additional wall passes, and controls that would conflict with the
authored descent are refused. The per-chain normal lead replaces the
ordinary lead-in and lead-out shapes, lead height, extensions, and
overlap. Explicit feed overrides and the configured ramp limits still
apply.

The ordinary program gate still inspects the complete job. The existing
geometry command authors the optional setting and the project stores it
with the chain override. The viewport can display or clear it.

Evidence uses a synthetic outline. The tests cover a single lap without
a floor recut, the command and project round trip, malformed geometry,
a crossing normal lead, and incompatible companions.
