# Инструкция для нейросети: собрать MapperX Board в `.exe` (Windows) и `.app` (macOS)

Проект: `/Users/hampthor/Desktop/4-wheels-aio-done/Dosti_WSK` (на Windows — та же папка, скопированная целиком).

## 0. Запреты (прочитать первым)
- **НЕ трогать** `~/MapperX_backup_2026-11-05/` — это резервная копия владельца (только чтение). Ничего там не
  менять, не удалять, не собирать, не запускать.
- Не менять логику `field_planner.py`, Java-код робота, `src/main/deploy/*`, `field_maps/*`. Задача — только сборка.
  Менять можно лишь файлы сборки в `tools/mapperx-app/` (см. шаг 2.1).
- Не коммитить, не пушить, не деплоить на робота.
- Не трогать встроенные блоки внутри `field_planner.py` (`MAPPERX VAULT`, `WHEELS`, `PYTHON`).
- Не трогать `~/.mapperx/` (ключ подписи обновлений) и `~/.mapperx_board.json`.
- MapperX должен работать **офлайн**: в приложение кладём библиотеки из `mapperx_wheels/`, ничего не качаем во время работы приложения.

## 1. Что собираем и из чего
**MapperX Board** — окно MapperX (pywebview) + запуск `field_planner.py`.
- Точка входа: `tools/mapperx-app/mapperx_board.py`.
  При запуске берёт `field_planner.py` **из папки проекта** (путь в `~/.mapperx_board.json`, при первом запуске
  спрашивает папку). Без проекта использует копию `field_planner.py`, вшитую в приложение (дашборд работает,
  но DetectX и деплою нужна папка проекта: DetectX грузит `src/main/deploy/calibrate_hsv.py` и `color_mask.py`).
- Файлы сборки (`tools/mapperx-app/`):
  - `build_mac.sh` — сборка `.app` (PyInstaller, папка-бандл);
  - `MapperX Board.spec` — spec для Mac (то же, что в `build_mac.sh`);
  - `build_windows.bat` — сборка `.exe` (PyInstaller `--onefile`);
  - `make_icon.py`, `icon.png`, `icon.icns` — иконка.
- Вшивается в приложение: `field_planner.py` (корень проекта) + библиотеки:
  `pywebview`, `pynetworktables`, `numpy`, `opencv-python-headless` (**именно headless**, не `opencv-python`:
  вместе они конфликтуют), на Windows ещё `pythonnet`, `clr_loader`, `cffi`.
- Офлайн-колёса для всех платформ лежат в `mapperx_wheels/` (Python 3.12: `numpy-2.5.4-cp312-*`,
  `opencv_python_headless-5.0.0.93-cp37-abi3-*`, `pywebview-6.2.1`, `pythonnet-3.2.1`, `cffi-2.1.1-cp312-win_amd64` и др.).
- В `mapperx_wheels/` **нет** `pyinstaller` и `pillow` — на машине сборки нужен интернет один раз, чтобы их поставить.

**Важно: PyInstaller не собирает кросс-платформенно.** `.exe` собирается только на Windows, `.app` — только на Mac.

## 2. Windows `.exe`
Требования: Windows 10/11 x64, Python **3.12 x64** с python.org (галочка «Add python.exe to PATH»),
WebView2 Runtime (в Windows 10/11 обычно уже есть).

### 2.1. Уже сделано: проверить, что `build_windows.bat` содержит numpy/OpenCV
Шаг выполнен в репозитории — просто убедиться (чек-лист):
- офлайн-установка `numpy` и `opencv-python-headless` из `..\..\mapperx_wheels` (через `windows-build.lock`
  или строкой `.venv\Scripts\pip install -q --no-index --find-links ..\..\mapperx_wheels numpy opencv-python-headless || goto :fail`);
- в команде `pyinstaller` есть флаги (как в `build_mac.sh`): `--hidden-import numpy --collect-all cv2`;
- окружение создаётся именно Python 3.12 x64 (колёса numpy в `mapperx_wheels` есть для cp312).

### 2.2. Сборка
```bat
cd tools\mapperx-app
build_windows.bat
```
Результат: `tools\mapperx-app\dist\MapperX Board.exe`.

### 2.3. Проверка
1. Запустить `MapperX Board.exe`, выбрать папку проекта `Dosti_WSK`.
2. Окно MapperX открылось, ошибок нет.
3. Открыть студию **DetectX** — нет сообщения «нужен OpenCV».
4. Проверить API в консоли страницы: `await api('/setup/status', {})` → `missing: []`.
5. SmartScreen/антивирус может ругаться на неподписанный `--onefile` exe — это ожидаемо
   («Подробнее» → «Выполнить в любом случае»).

## 3. macOS `.app`
Требования: Mac (Apple Silicon), Homebrew Python 3.12 (`/opt/homebrew/bin/python3.12`), Xcode Command Line Tools
(`sips`, `iconutil`).

### 3.1. Сборка
Перед сборкой перенести старые `dist/` и `build/` в сторону (не удалять — это рабочая сборка владельца):
```bash
cd tools/mapperx-app
mkdir -p ~/MapperX_old_builds && mv dist build ~/MapperX_old_builds/ 2>/dev/null
./build_mac.sh
```
Скрипт сам: создаёт `.venv`, ставит `pywebview pyinstaller pynetworktables`, офлайн ставит `numpy` и
`opencv-python-headless` из `../../mapperx_wheels`, делает иконку и собирает.
Результат: `tools/mapperx-app/dist/MapperX Board.app` (~305 МБ).

**Внимание:** значок в Dock «едет» за перемещённым `.app`. После сборки проверить, что Dock указывает на
`tools/mapperx-app/dist/MapperX Board.app`, иначе убрать старый значок и перетащить новый.

### 3.2. Проверка
1. `open "dist/MapperX Board.app"` — окно открылось, нет ошибок `PYI-…`.
2. `find "dist/MapperX Board.app" -name 'cv2*.so' | head -1` — OpenCV внутри.
3. DetectX открывается без «нужен OpenCV»; `api('/setup/status', {})` → `missing: []`.
4. Неподписанное приложение на другом Mac: правый клик → «Открыть» (Gatekeeper).

## 4. Если нужен «полностью офлайн» пакет без установки (обе ОС сразу)
Это уже есть и собирается одной командой на Mac (Python + все библиотеки внутри, Windows и Mac):
```bash
python3 field_planner.py --pack-offline
```
Результат: папка `MapperX_offline/` (~1 ГБ): `MapperX_offline.py` + «Запустить MapperX (Windows).cmd» +
«Запустить MapperX (Mac).command». Переносится целиком, первый запуск ставит всё без интернета.

## 5. Код робота
Java-код робота **не** входит в `.exe`/`.app`: он собирается и заливается на робота отдельно
(`./gradlew build`, деплой — только по команде владельца).

## 6. Отчёт владельцу (кратко, по-русски)
- что собрано и где лежит (путь, размер);
- что проверено (запуск, DetectX, `missing: []`) и что не проверено;
- какие файлы сборки изменены (дифф `build_windows.bat`).
