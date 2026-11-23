"""Write the build record (sha256, fork commit, upstream commit, toolchain) beside the journeyman-job binary."""
import hashlib, json, pathlib, sys

exe, head, up, cargo = sys.argv[1:]
p = pathlib.Path(exe)
rec = dict(binary=p.name, sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size,
           fork_commit=head, upstream_commit=up, toolchain=cargo)
pathlib.Path(exe + '.json').write_text(json.dumps(rec, indent=1))
print(json.dumps(rec))
