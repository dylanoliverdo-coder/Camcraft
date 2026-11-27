# Closed sketch chamfer arcs

The sketch chamfer already knows which side of each closed pass is air.
Its closed passes use the same material-sided fitter as a contour,
bounded by the operation tolerance and the standard arc fit budget.
The generated arcs reach the same verifier and post as the other
generated moves. The authored edge break keeps the original guide.

Open sketch chains keep their pointwise walk. The existing model-edge
chamfer also keeps its current path. Caches from the earlier sketch
chamfer generator read stale once and regenerate with the current path.

The circular sketch test runs through the normal Haas post. The motion
tests check both directions, closed straight controls and open chains.
Physical controller timing requires a separate measurement.
