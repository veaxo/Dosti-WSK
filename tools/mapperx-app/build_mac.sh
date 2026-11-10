#!/bin/bash
# Builds "Shuffleboard.app" (macOS) into tools/mapperx-app/dist/.
set -e
cd "$(dirname "$0")"
PY=python3
[ -x /opt/homebrew/bin/python3.12 ] && PY=/opt/homebrew/bin/python3.12
[ -d .venv ] || "$PY" -m venv .venv
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_CACHE_DIR="$PWD/.pip-cache"
export PYINSTALLER_CONFIG_DIR="$PWD/.pyinstaller-cache"
.venv/bin/python -c 'import sys; assert sys.version_info[:2] == (3, 12), "Python 3.12 is required"'
.venv/bin/python -c 'import PyInstaller' || .venv/bin/python -m pip install -q pyinstaller
.venv/bin/python -m pip install -q --no-index --find-links ../../mapperx_wheels pywebview pynetworktables numpy opencv-python-headless
.venv/bin/python make_icon.py
.venv/bin/python fetch_windows_prereqs.py
.venv/bin/python prepare_bundle.py
rm -rf icon.iconset && mkdir icon.iconset
for s in 16 32 128 256 512; do
  sips -z $s $s icon.png --out icon.iconset/icon_${s}x${s}.png >/dev/null
  sips -z $((s*2)) $((s*2)) icon.png --out icon.iconset/icon_${s}x${s}@2x.png >/dev/null
done
iconutil -c icns icon.iconset -o icon.icns
rm -rf icon.iconset
.venv/bin/pyinstaller --noconfirm --clean --windowed --name "Shuffleboard" \
  --icon icon.icns --osx-bundle-identifier com.dosti.mapperxboard \
  --paths ../.. --add-data generated/field_planner.py:. --hidden-import networktables --collect-submodules networktables \
  --add-data splash.html:. --add-data WPILIB_LICENSE.txt:. --add-data icon.png:. --add-data icon.ico:. \
  --add-data bundle_payload:bundle_payload --add-data bundle-manifest.json:. \
  --hidden-import numpy --collect-all cv2 --copy-metadata pywebview --copy-metadata pynetworktables \
  mapperx_board.py
echo "Готово: $(pwd)/dist/Shuffleboard.app"
