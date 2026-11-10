"""Build-time downloads only; the resulting Windows application never downloads these."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request

FOLDER = Path(__file__).resolve().parent / "windows_prerequisites"
SOURCES = {
    "WebView2-x64.exe": "https://go.microsoft.com/fwlink/?linkid=2124701",
    "NET-Framework-4.8.exe": "https://go.microsoft.com/fwlink/?linkid=2088631",
}


def main():
    FOLDER.mkdir(exist_ok=True)
    records = {}
    for name, url in SOURCES.items():
        target = FOLDER / name
        if not target.exists():
            temporary = target.with_suffix(".download")
            print("Downloading official offline prerequisite:", name, flush=True)
            with urllib.request.urlopen(url, timeout=120) as response, temporary.open("wb") as out:
                while block := response.read(4 * 1024 * 1024):
                    out.write(block)
            temporary.replace(target)
        with target.open("rb") as source:
            if source.read(2) != b"MZ" or target.stat().st_size < 10_000_000:
                raise ValueError("Not a full offline Windows installer: " + name)
        if sys.platform == "win32":
            # Signature validation happens on the actual Windows build host.
            script = ("$s=Get-AuthenticodeSignature -LiteralPath $env:SHUFFLEBOARD_SIGNED_INSTALLER; "
                      "if ($s.Status -ne 'Valid' -or $s.SignerCertificate.Subject -notmatch "
                      "'O=Microsoft Corporation') { exit 1 }")
            subprocess.run(["powershell.exe", "-NoProfile", "-Command", script], check=True,
                           env=dict(os.environ, SHUFFLEBOARD_SIGNED_INSTALLER=str(target)))
        with target.open("rb") as source:
            digest = hashlib.file_digest(source, "sha256").hexdigest()
        records[name] = {"url": url, "size": target.stat().st_size, "sha256": digest}
        print(name, target.stat().st_size, digest, flush=True)
    metadata = FOLDER / "sources.json"
    if not metadata.exists() or json.loads(metadata.read_text()) != records:
        metadata.write_text(json.dumps(records, indent=2) + "\n")


if __name__ == "__main__":
    main()
