# Journeyman operation specs

Each file here specifies one change to a CamCraft operation as geometric and kinematic relationships:
what the cutter touches, the offsets and levels it rides, the order of its passes, its entry, link and
retract motions, and the checks that prove them on synthetic geometry.

## Two roles

- The **spec writer** studies machined motion and stock, writes the relationships in neutral words, and
  checks them in a stock simulator. A spec carries no vendor parameter names, dialog or document text,
  user-interface text, post-processor text or customer data. Its tests use synthetic geometry only.
- The **builder** reads only the spec, CamCraft's own code and docs, and published literature. The
  builder writes the code and the tests. The spec writer reviews the result.

## Provenance

Each module that a spec changes or adds carries one provenance line, for example:

    //! Provenance: Journeyman spec S1 (docs/journeyman/specs/S1-wall-passes.md); offset curves after
    //! Held 1991; no vendor parameters, text or code.

## Rules that every spec inherits

- Defaults keep today's output: with every new field at its default, each existing test and golden file
  gives the same result, and a project file saved before the field loads and saves unchanged.
- CamCraft's own rules hold (its `AGENTS.md`): no `unwrap`, `expect` or `panic!` outside tests; a value
  that cannot be honoured is refused by name, never approximated in silence; files stay near 400 lines;
  a test lives with the code it tests; every user-facing setting is edited through the existing
  undoable command.
- Units are millimetres and millimetres per minute, as in CamCraft.

## Index

| Spec | Operation | Status |
|---|---|---|
| S1 | Contour: several wall passes per level | built (008af6cfd), checked on a lab part |
| S2 | Pocket, Contour, Face: floor allowance | built (e91a07bf5), checked on a lab part |
| S3 | 2D operations: fed retract | written |
| S4 | Pocket: wall finishing passes with their own direction | written |
| S5 | 2D Adaptive: wall finishing passes after clearing | built (06aa8e191), checked on a lab part |
| S6 | Drill: chamfer each hole to a width | built (360cbfe6c), checked on a lab part |
| S7 | 2D descents: a stated height where the rapid becomes a feed | written |
| S8 | Contour, Pocket, Face: finishing levels at a smaller stepdown | written |
| S9 | Contour: open sketch chain on a chosen side | built (9200f5ff2), checked on a lab part |
| S10 | 2D Chamfer: closed and open sketch chains | built (9200f5ff2), checked on a lab part |
