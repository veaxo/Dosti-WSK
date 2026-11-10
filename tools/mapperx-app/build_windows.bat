@echo off
rem Builds "Shuffleboard.exe" (Windows 10/11 x64) into tools\mapperx-app\dist\.
rem Uses the bundled Windows Python 3.12. No installed Python or internet is needed.
setlocal
cd /d "%~dp0"
set "PIP_DISABLE_PIP_VERSION_CHECK=1"
set "PYTHONDONTWRITEBYTECODE=1"
set "PYTHONUTF8=1"
set "PYINSTALLER_CONFIG_DIR=%CD%\.pyinstaller-cache-windows"
set "PIP_CACHE_DIR=%CD%\.pip-cache-windows"
set "BUILD_PY=%CD%\.build-python-windows"
set "VENV=%CD%\.venv-windows-build"
if exist "%VENV%\Scripts\python.exe" goto environment_ready
if exist "%BUILD_PY%\python.exe" goto create_environment
if not exist "..\..\mapperx_python\x86_64-pc-windows-msvc.tar.gz" goto missing_python
echo Extracting bundled Python 3.12 x64...
if not exist "%BUILD_PY%" mkdir "%BUILD_PY%"
tar -xzf "..\..\mapperx_python\x86_64-pc-windows-msvc.tar.gz" -C "%BUILD_PY%" --strip-components 1 || goto :fail
:create_environment
"%BUILD_PY%\python.exe" -m venv "%VENV%" || goto :fail
:environment_ready
"%VENV%\Scripts\python.exe" -c "import sys, struct; assert sys.version_info[:2] == (3, 12) and struct.calcsize('P') == 8, 'Python 3.12 x64 is required'" || goto :fail
"%VENV%\Scripts\python.exe" verify_windows_kit.py || goto :fail
if not exist windows-build.lock goto missing_tools
echo Installing build tools and app libraries from bundled wheels...
"%VENV%\Scripts\python.exe" -m pip install --no-index --find-links windows_build_wheels --find-links ..\..\mapperx_wheels --require-hashes -r windows-build.lock || goto :fail
"%VENV%\Scripts\python.exe" make_icon.py || goto :fail
"%VENV%\Scripts\python.exe" fetch_windows_prereqs.py || goto :fail
"%VENV%\Scripts\python.exe" prepare_bundle.py || goto :fail
echo Building the single Shuffleboard.exe. This can take several minutes...
"%VENV%\Scripts\pyinstaller.exe" --noconfirm --clean --windowed --onefile --name "Shuffleboard" ^
  --icon icon.ico --paths ..\.. --add-data generated\field_planner.py;. --hidden-import networktables --collect-submodules networktables ^
  --add-data splash.html;. --add-data WPILIB_LICENSE.txt;. --add-data icon.png;. --add-data icon.ico;. ^
  --add-data bundle_payload;bundle_payload --add-data bundle-manifest.json;. ^
  --hidden-import numpy --collect-all cv2 --copy-metadata pywebview --copy-metadata pynetworktables ^
  --hidden-import clr --hidden-import clr_loader --hidden-import cffi --hidden-import _cffi_backend --collect-all pythonnet --collect-all clr_loader ^
  mapperx_board.py || goto :fail
"%VENV%\Scripts\python.exe" package_release.py dist\Shuffleboard.exe || goto :fail
echo DONE: %cd%\dist\Shuffleboard.exe
pause
exit /b 0
:missing_python
echo Missing mapperx_python\x86_64-pc-windows-msvc.tar.gz. Extract the complete Windows build kit.
goto :fail
:missing_tools
echo Missing windows-build.lock or offline build tools. Extract the complete Windows build kit.
goto :fail
:fail
echo BUILD FAILED. The error details are above.
pause
exit /b 1
