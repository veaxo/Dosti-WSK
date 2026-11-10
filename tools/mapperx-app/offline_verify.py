"""Template: verify immutable offline payloads while allowing normal MapperX code/vault updates."""
import base64
import hashlib
import json
from pathlib import Path
import sys
import tempfile

EXPECTED = json.loads(r'''@@PAYLOAD_RECORDS@@''')
INITIAL_MAIN_SHA256 = "@@MAIN_SHA256@@"


def verify(path):
    # The portable launcher also accepts Python 3.9/3.10, which have no file_digest.
    hasher = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            hasher.update(chunk)
    current = hasher.hexdigest()
    identity = hashlib.sha256(str(path.resolve()).encode()).hexdigest()[:16]
    cache = Path(tempfile.gettempdir()) / ("Shuffleboard-integrity-" + INITIAL_MAIN_SHA256[:16] + "-" + identity + ".json")
    if current == INITIAL_MAIN_SHA256:
        return
    try:
        if json.loads(cache.read_text()).get("sha256") == current:
            return
    except (OSError, ValueError):
        pass
    pending = {r["begin"]: r for r in EXPECTED}
    active = None
    parts = []
    total = 0
    hasher = None
    with path.open(encoding="utf-8") as source:
        for raw in source:
            line = raw.rstrip("\r\n")
            if active is None:
                if line in pending:
                    active = pending[line]
                    hasher = hashlib.sha256()
                    parts = []
                    total = 0
                continue
            if line == active["end"]:
                if parts:
                    block = base64.b64decode("".join(parts), validate=True)
                    hasher.update(block)
                    total += len(block)
                if hasher.hexdigest() != active["sha256"] or total != active["size"]:
                    raise ValueError("Damaged offline payload: " + active["begin"])
                del pending[active["begin"]]
                active = None
                continue
            if line.startswith("# ") and not line.startswith("# meta "):
                parts.append(line[2:])
                if len(parts) >= 4096:
                    block = base64.b64decode("".join(parts), validate=True)
                    hasher.update(block)
                    total += len(block)
                    parts.clear()
    if active is not None or pending:
        raise ValueError("Incomplete offline main file")
    cache.write_text(json.dumps({"sha256": current}))


if __name__ == "__main__":
    try:
        verify(Path(sys.argv[1]))
    except Exception as error:
        print("Offline integrity check failed:", error)
        raise SystemExit(13)
