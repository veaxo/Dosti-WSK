"""MapperX Board: MapperX in its own window (macOS / Windows), opening on the
Shuffleboard-style dashboard. The studios are in the menu «Вид».

The app runs field_planner.py from the robot project folder, so it always uses
the current MapperX code and edits that project. The folder is remembered in
~/.mapperx_board.json; the first start asks for it. Without a project the copy
of field_planner.py built into the app is used (the dashboard works, the
studios that change project files have nothing to change).
"""
import importlib.util
import json
import os
import sys
import threading
import time
from pathlib import Path

import bundle_bootstrap as bootstrap
import webview
from webview.menu import Menu, MenuAction, MenuSeparator

CONFIG = Path.home() / ".mapperx_board.json"
TITLE = "Shuffleboard"
# (studio key, menu title); Cmd/Ctrl+Shift+1…8 in this order, as in the page
STUDIOS = [("map", "MapperX — карта, маршрут"), ("detect", "DetectX — камера, HSV"), ("tune", "TuneX — параметры"),
           ("construct", "ConstructX — конструктор кода"), ("board", "BoardX — NetworkTables"),
           ("pin", "PinX — распиновка"), ("pit", "PitX — перед заездом"), ("vault", "VaultX — бэкап, сборка, деплой")]


def load_config():
    try:
        return json.loads(CONFIG.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_config(data):
    try:
        CONFIG.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
    except OSError:
        pass


def project_candidates():
    """Folders that may hold the robot project, best first."""
    out = []
    if os.environ.get("MAPPERX_PROJECT"):
        out.append(Path(os.environ["MAPPERX_PROJECT"]))
    saved = load_config().get("project")
    if saved:
        out.append(Path(saved))
    exe = Path(sys.executable).resolve()
    # the app inside the project (Dosti_WSK/tools/mapperx-app/dist/MapperX Board.app/
    # Contents/MacOS/<exe>) or next to it
    for parent in list(exe.parents)[:8]:
        out.append(parent)
    return out


def find_project():
    for folder in project_candidates():
        if (folder / "field_planner.py").is_file():
            return folder
    return None


def load_planner(folder):
    """field_planner.py of the project, or the built-in copy."""
    if folder is None:
        # the copy inside the app (build scripts add field_planner.py as a
        # file too, it reads its own size/time); this import also makes
        # PyInstaller pack everything MapperX needs
        import field_planner as bundled
        bootstrap.configure_planner(bundled, None)
        return bundled
    spec = importlib.util.spec_from_file_location("field_planner_project", folder / "field_planner.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    bootstrap.configure_planner(module, folder)
    return module


def ask_project(window):
    """First start: the user points at the project folder (Dosti_WSK)."""
    picked = window.create_file_dialog(webview.FOLDER_DIALOG)
    if not picked:
        return None
    folder = Path(picked[0] if isinstance(picked, (list, tuple)) else picked)
    if not (folder / "field_planner.py").is_file():
        window.evaluate_js("mxAlert('В этой папке нет field_planner.py — выберите папку проекта робота (Dosti_WSK).')")
        return None
    data = load_config()
    data["project"] = str(folder)
    save_config(data)
    return folder


def start_server(planner):
    os.environ["MAPPERX_PORT"] = "0"   # any free port: a MapperX in the browser may use 8765
    server = planner.PlannerServer()
    if hasattr(planner, "LIVEX"):
        planner.LIVEX.start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, "http://127.0.0.1:%d/?sb=1" % server.server_port


def app_menu(window):
    """Native menu «Вид»: the dashboard and the studios. A real menu works
    whatever the page does with the keyboard."""
    def panel():
        window.evaluate_js("if (typeof sbEnabled === 'function' && sbEnabled()) sbShow(true); else location.search = '?sb=1';")

    def studio(key):
        def go():
            window.evaluate_js("if (typeof sbEnabled === 'function' && sbEnabled()) sbOpenStudio('%s'); else setStudio('%s');" % (key, key))
        go.__name__ = "studio_" + key
        return go

    def full():
        window.evaluate_js("location.search = '';")

    items = [MenuAction("Панель Shuffleboard", panel), MenuSeparator()]
    items += [MenuAction(title, studio(key)) for key, title in STUDIOS]
    items += [MenuSeparator(), MenuAction("Полный интерфейс MapperX", full)]
    return [Menu("Вид", items)]


def mac_shortcuts():
    """pywebview menu items have no shortcuts: give «Вид» Cmd+Shift+0…8."""
    if sys.platform != "darwin":
        return
    import AppKit

    def apply():
        main = AppKit.NSApp.mainMenu()
        for i in range(main.numberOfItems()):
            sub = main.itemAtIndex_(i).submenu()
            if sub is None or sub.title() != "Вид":
                continue
            keys = {"Панель Shuffleboard": "0"}
            keys.update({title: str(n + 1) for n, (_k, title) in enumerate(STUDIOS)})
            for j in range(sub.numberOfItems()):
                item = sub.itemAtIndex_(j)
                if item.title() in keys:
                    item.setKeyEquivalent_(keys[item.title()])
                    item.setKeyEquivalentModifierMask_(AppKit.NSEventModifierFlagCommand | AppKit.NSEventModifierFlagShift)
    AppKit.NSOperationQueue.mainQueue().addOperationWithBlock_(apply)


def main():
    if "--self-check" in sys.argv:
        report = bootstrap.self_check()
        text = json.dumps(report, ensure_ascii=False, indent=2)
        if "--self-check-output" in sys.argv:
            Path(sys.argv[sys.argv.index("--self-check-output") + 1]).write_text(text, encoding="utf-8")
        if sys.stdout is not None:
            print(text)
        return 0 if report["ok"] else 1
    try:
        bootstrap.ensure_system_prerequisites()
        bootstrap.verify_files(prefix="splash.html")
    except Exception as exc:
        bootstrap.show_error(exc)
        return 1
    project = find_project()
    state = {"server": None, "url": None}
    splash_html = (Path(__file__).resolve().parent / "splash.html").read_text(encoding="utf-8")
    splash = webview.create_window(TITLE, html=splash_html, width=480, height=240,
                                   resizable=False, frameless=True, easy_drag=True,
                                   background_color="#2626F7")
    window = webview.create_window(TITLE, html="<html></html>", width=1400, height=900, min_size=(900, 600),
                                   hidden=True,
                                   background_color="#F5F5F5", text_select=True)
    splash_ready = threading.Event()
    dashboard_ready = threading.Event()
    splash.events.loaded += splash_ready.set

    def dashboard_loaded():
        if state["url"] and window.get_current_url() == state["url"]:
            dashboard_ready.set()

    window.events.loaded += dashboard_loaded

    def progress(text, fraction):
        splash.evaluate_js("setStartupState(%s, %s)" % (json.dumps(text), fraction))

    def first_start():
        splash_ready.wait()
        started = time.monotonic()
        try:
            progress("Checking offline package integrity", 0.03)
            bootstrap.verify_files(progress=lambda n, total, _name: progress("Checking offline package integrity", 0.03 + 0.35 * n / total))
            progress("Checking bundled libraries", 0.4)
            checks = bootstrap.dependency_check()
            failed = [c["module"] + ": " + c.get("error", "") for c in checks if not c["ok"]]
            if failed:
                raise RuntimeError("\n".join(failed))
            progress("Loading MapperX", 0.5)
            planner = load_planner(project)
            progress("Starting dashboard", 0.75)
            state["server"], state["url"] = start_server(planner)
            window.load_url(state["url"])
            if not dashboard_ready.wait(45):
                raise RuntimeError("Dashboard did not finish loading")
            progress("Ready", 1)
            # Keep the original startup animation visible on fast machines.
            time.sleep(max(0, 1.2 - (time.monotonic() - started)))
        except Exception as exc:
            progress("Startup error: " + str(exc), 0)
            window.destroy()
            return
        window.show()
        splash.destroy()
        mac_shortcuts()
        if project is None:
            folder = ask_project(window)
            if folder is not None:
                # restart on the chosen project's MapperX
                state["server"].shutdown()
                state["server"], new_url = start_server(load_planner(folder))
                window.load_url(new_url)

    webview.start(first_start, private_mode=False, menu=app_menu(window))
    if state["server"] is not None:
        state["server"].shutdown()


if __name__ == "__main__":
    raise SystemExit(main() or 0)
