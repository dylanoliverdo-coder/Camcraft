#!/bin/bash
# Build the journeyman-job bin reproducibly and write its record next to it.
#   bash journeyman/build.sh
# Env (lab defaults shown): JM_TARGET (target-jm under the checkout), CARGO, JM_LANE_JOB (the lab's
# lane_job.py; empty runs cargo directly). --locked: the build uses Cargo.lock as committed.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
export CARGO_TARGET_DIR="${JM_TARGET:-$PWD/target-jm}"
CARGO="${CARGO:-cargo}"
# Reproducible: no absolute build paths in the binary.
export RUSTFLAGS="${RUSTFLAGS:-} --remap-path-prefix=$PWD=camcraft --remap-path-prefix=${CARGO_HOME:-$HOME/.cargo}=cargo"
# Windows (MSVC link.exe): /Brepro replaces the link time stamps and the PDB GUID with hashes of the content, so
# two builds of one commit are byte-identical (without it 24 bytes differed: PE and debug-directory stamps, GUID).
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) RUSTFLAGS="$RUSTFLAGS -C link-arg=/Brepro" ;; esac
CMD=("$CARGO" build --release --locked -p camcraft-sim --bin journeyman-job)
if [ -n "${JM_LANE_JOB:-}" ]; then
  python "$JM_LANE_JOB" --name camcraft-jm-build --process-gb 8 --job-gb 12 -- "${CMD[@]}"
else
  "${CMD[@]}"
fi
EXE="$CARGO_TARGET_DIR/release/journeyman-job.exe"; [ -f "$EXE" ] || EXE="$CARGO_TARGET_DIR/release/journeyman-job"
UP=$(git merge-base HEAD upstream/main)
python "$(dirname "$0")/record.py" "$EXE" "$(git rev-parse HEAD)" "$UP" "$("$CARGO" --version)"
