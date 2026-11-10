"""Package the native Windows builder with Python, wheels and source; no Mac caches."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import sys

from package_release import package
from project_snapshot import project_files


def main():
    app = Path(__file__).resolve().parent
    project = app.parents[1]
    stage = Path(tempfile.mkdtemp(prefix="windows-kit-", dir=app))
    root = stage / "Shuffleboard-Windows-Build"
    target_app = root / "tools/mapperx-app"
    target_app.mkdir(parents=True)
    for source in project_files(project):
        target = root / source.relative_to(project)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for name in ["mapperx_wheels", "mapperx_python"]:
        source = project / name
        if source.exists():
            shutil.copytree(source, root / name, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store"))
    scripts = ["build_windows.bat", "mapperx_board.py", "bundle_bootstrap.py", "prepare_bundle.py",
               "fetch_windows_prereqs.py", "make_icon.py", "offline_system_check.ps1", "offline_verify.py",
               "package_release.py", "project_snapshot.py", "verify_windows_kit.py", "windows-build.lock", "windows-build-dependencies.json",
               "icon.ico", "icon.png", "splash.html", "ICON_SOURCE.md", "WPILIB_LICENSE.txt"]
    for name in scripts:
        shutil.copy2(app / name, target_app / name)
    for folder in ["windows_build_wheels", "windows_prerequisites"]:
        shutil.copytree(app / folder, target_app / folder)
    launcher = ('@echo off\r\ncd /d "%~dp0"\r\n'
                'echo Native Windows EXE builder. Keep this window open until DONE.\r\n'
                'cmd /d /k call "%~dp0tools\\mapperx-app\\build_windows.bat"\r\n')
    (root / "BUILD_WINDOWS_EXE.cmd").write_bytes(launcher.encode("ascii"))
    readme = '''# Как получить Shuffleboard.exe

1. Нужна Windows 10/11 x64. Распакуйте ВЕСЬ архив в короткий путь, например C:\\Shuffleboard-Build.
2. Дважды щёлкните BUILD_WINDOWS_EXE.cmd.
3. Дождитесь строки DONE. Сборка большого автономного EXE может занять несколько минут.
4. Результат: tools\\mapperx-app\\dist\\Shuffleboard.exe.
5. Перенесите этот EXE на рабочий стол или другой компьютер Windows x64.

Для работы с этим проектом оставьте EXE в tools\\mapperx-app\\dist либо
скопируйте в корень проекта рядом с field_planner.py. Проект будет найден
автоматически. Java-лаунчер MapperX.java запускается командой java MapperX.java
из корня проекта (JDK 11+); инструкция — MAPPERX_LAUNCHER.md.

Python, инструменты сборки, библиотеки приложения, иконка и офлайн-установщики
включены. Интернет и заранее установленный Python для сборки не требуются.
Не перемещайте файлы до окончания сборки. Желательно около 10 ГБ свободного места.

Готовый EXE содержит иконку WPILib, название Shuffleboard, заставку
powered by Mapperx и автономные компоненты. Python на компьютере пользователя
не требуется. Отдельный .exe.sha256 содержит контрольную сумму результата.

Консоль после ошибки остаётся открытой. Если нет строки DONE, не считайте
сборку успешной: сохраните текст ошибки из окна.

Этот комплект подготовлен на Mac. Нативная Windows-сборка пока не проверена
на реальном Windows. Сборка Java и деплой на робота этим CMD не выполняются.
'''
    (root / "README_WINDOWS.md").write_text(readme, encoding="utf-8")
    records = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            with path.open("rb") as source:
                checksum = hashlib.file_digest(source, "sha256").hexdigest()
            records[path.relative_to(root).as_posix()] = {"size": path.stat().st_size, "sha256": checksum}
    (root / "WINDOWS_BUILD_MANIFEST.json").write_text(json.dumps({"schema": 1, "files": records}, indent=2) + "\n")
    (app / "windows-build-kit-path.txt").write_text(str(root))
    destination = Path(sys.argv[1]) if len(sys.argv) > 1 else app / "dist/Shuffleboard-Windows-EXE-Builder.zip"
    destination.parent.mkdir(parents=True, exist_ok=True)
    package(root, destination)
    print("Build kit:", root)
    print("Verified input inventory:", len(records), "files")


if __name__ == "__main__":
    main()
