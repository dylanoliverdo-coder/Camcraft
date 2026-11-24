#!/bin/bash
# One-command upstream sync for the journeyman/main hybrid fork.
#
#   bash journeyman/sync.sh            merge upstream/main (the newest upstream code)
#   bash journeyman/sync.sh --tag      merge the newest upstream cut tag (v*.*.0) instead
#
# The fork is upstream plus small additive commits, each touching only files upstream does not
# have (journeyman/, crates/sim/src/bin/journeyman-job.rs), so a merge does not conflict unless
# upstream changes the stock_sim API the driver calls. Then the build fails, and the fix is one
# commit on the driver. Upstream's own fork-sync skill syncs forks at cut tags; our ruling
# (2026-11-23, camcraft is the engine) follows upstream main, so main is the default.
# Never rebases, never force-pushes; a merge commit records each sync.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
[ "$(git rev-parse --abbrev-ref HEAD)" = journeyman/main ] || { echo "not on journeyman/main"; exit 1; }
[ -z "$(git status --porcelain --untracked-files=no)" ] || { echo "tracked changes present: commit them first"; exit 1; }
git fetch upstream --tags --quiet
if [ "${1:-}" = --tag ]; then
  TARGET=$(git tag -l 'v*.*.0' --sort=-v:refname | head -1)
else
  TARGET=upstream/main
fi
SHA=$(git rev-parse "$TARGET^{commit}")
if git merge-base --is-ancestor "$SHA" HEAD; then echo "already contains $TARGET ($SHA)"; exit 0; fi
git merge --no-ff --no-edit -m "journeyman/main: sync with $TARGET ($SHA)" "$SHA"
echo "merged $TARGET $SHA; ours on top:"
git log --oneline --no-merges "$SHA"..HEAD
# Build and smoke-test the driver on the merged tree.
bash "$(dirname "$0")/build.sh"
