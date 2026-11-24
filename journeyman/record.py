"""Write the build record (sha256, fork commit, upstream commit, toolchain) beside the journeyman-job binary."""
import hashlib, json, pathlib, subprocess, sys

exe, head, up, cargo = sys.argv[1:]
p = pathlib.Path(exe)
rec = dict(binary=p.name, sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size,
           fork_commit=head, upstream_commit=up, toolchain=cargo)
# What the binary itself says it can do; an older build has no --capabilities and records none.
try:
    out = subprocess.run([str(p), '--capabilities'], capture_output=True, text=True, timeout=60).stdout
    rec['capabilities'] = json.loads(out.strip().splitlines()[-1])['capabilities']
except (OSError, ValueError, KeyError, IndexError, subprocess.TimeoutExpired):
    rec['capabilities'] = []
pathlib.Path(exe + '.json').write_text(json.dumps(rec, indent=1))
print(json.dumps(rec))
