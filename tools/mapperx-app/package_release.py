"""Create portable UTF-8 ZIPs preserving .app symlinks, and release SHA-256 files."""
import hashlib
import os
from pathlib import Path
import stat
import sys
import zipfile


def package(source, destination=None):
    source = Path(source).resolve()
    if destination is not None:
        target = Path(destination).resolve()
        temporary = target.with_suffix(".zip.tmp")
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=3) as archive:
            def add(path):
                name = path.relative_to(source.parent).as_posix()
                if path.is_symlink():
                    info = zipfile.ZipInfo(name)
                    info.create_system = 3
                    info.external_attr = (stat.S_IFLNK | 0o777) << 16
                    archive.writestr(info, os.readlink(path).encode("utf-8"))
                elif path.is_dir():
                    archive.write(path, name + "/")
                    for child in sorted(path.iterdir()):
                        add(child)
                else:
                    archive.write(path, name)
            add(source)
        temporary.replace(target)
    else:
        target = source
    with target.open("rb") as stream:
        checksum = hashlib.file_digest(stream, "sha256").hexdigest()
    target.with_name(target.name + ".sha256").write_text(checksum + "  " + target.name + "\n")
    print(target.name, target.stat().st_size, "bytes", checksum, flush=True)


if __name__ == "__main__":
    package(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
