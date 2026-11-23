# journeyman/main: CamCraft with Journeyman's driver on top

Upstream: https://github.com/NeoTech/Camcraft (Apache-2.0). This branch is upstream `main` plus small additive
commits that touch only files upstream does not have:

- `crates/sim/src/bin/journeyman-job.rs`: the driver. Reads the CAMotics project our cut lane writes (or posed rows),
  cuts it through CamCraft's public `stock_sim` API, writes the stock as a closed mesh (`--out-closed-stl`,
  `--matrix`), compares it with another engine's stock (`--ref-stl`), runs `Stock::deviate` against the part
  (`--part-stl`), and reports material above the flutes as shank strikes (`--cutting-length`).
- `journeyman/`: this note, `sync.sh`, `build.sh`, `record.py`.

## Sync with upstream: one command

    bash journeyman/sync.sh          # merge upstream/main, build, write the build record
    bash journeyman/sync.sh --tag    # merge the newest cut tag v*.*.0 instead

A merge commit records each sync; nothing is rebased or force-pushed. A driver build failure after a sync means
upstream changed the `stock_sim` API: the fix is one commit on the driver.

## Build

    bash journeyman/build.sh

`cargo build --release --locked`, paths remapped out of the binary; writes `journeyman-job.exe.json` beside the
binary with its sha256, the fork commit, the upstream commit it is based on, and the toolchain.

## What goes upstream

Generic fixes only, as issues and PRs under CamCraft's own process (AGENTS.md, its Issue Law, clean-room rule:
public sources only). Nothing derived from a commercial CAM system, and no customer data, ever goes into this
fork.
