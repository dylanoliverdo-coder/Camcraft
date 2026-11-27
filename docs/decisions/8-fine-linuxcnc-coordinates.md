# Optional finer LinuxCNC post coordinates

The LinuxCNC post can opt into a finer coordinate lattice when the generated motion requires it. The application sends the same additional precision for an approach move and its associated canned cycle, including mounted and per-operation posts. This preserves the verified location through posting.

The default remains the integer-micron stream. CNC exports retain that format. The short-arc test uses the selected motion lattice so a short sweep remains distinct from a full turn and a move that collapses at the selected resolution stays a linear move. Rotary words and dwell time retain their existing formats.

Synthetic tests cover fractional drilling coordinates and the short-arc, full-turn, and collapsed-arc controls. The opt-in requires a controller configured to accept the finer output.
