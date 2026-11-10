"""Validate bundled original WPILib 2020 icons without downloading assets."""
from pathlib import Path
import struct


def main():
    folder = Path(__file__).resolve().parent
    png = (folder / "icon.png").read_bytes()
    ico = (folder / "icon.ico").read_bytes()
    if not png.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("icon.png must be the bundled WPILib icon")
    if len(ico) < 6 or struct.unpack("<HHH", ico[:6])[:2] != (0, 1):
        raise ValueError("icon.ico must be a Windows icon")
    print("WPILib 2020 icons: icon.png, icon.ico")


if __name__ == "__main__":
    main()
