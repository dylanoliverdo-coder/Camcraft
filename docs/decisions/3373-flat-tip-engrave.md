## #3373 - Flat-tip taper engraving

Allow Engrave to use a center-cutting taper with a flat, unrounded tip.
Keep the programmed tool point at the physical tip plane. The verifier and
stock engines use the cutter's measured tip diameter and included angle.
The canonical stream carries the tip form separately from the tool kind.

The tip disk removes material at the programmed plane and changes where
the taper can contact the part. A sharp-cone substitute would misstate both
effects. The feature reuses the existing Engrave centerline and post checks.

Round tips and additional operations need their own contact laws and test
fixtures. Supporting them later requires extending the cutter profile and
the corresponding operation gates, while retaining the flat-tip datum.
