#!/usr/bin/env python3
"""HSV calibration in the browser (run on the PC, e.g. from PyCharm).

How a good profile is made:
  1. Get a frame from the robot camera, drag a rectangle tightly around the
     whole object and press "Add sample". Everything around the rectangle
     is automatically used as "not this object".
  2. Repeat on several frames: object near and far, in shadow and in bright
     light, on the floor and on the podium. Optionally mark background that
     must never match (switch the mode to "Background").
  3. "Calculate": the HSV range that covers the most of the object while
     matching almost none of the background is chosen. The mask the robot
     would see is painted green over the frame.
  4. "Check on new frame" a few times without adding samples. If something
     is missed, add a sample there and calculate again.
  5. "Save to project" writes one profile and removes old red_cube1 /
     red_cube2-style copies. Then deploy the project to the robot.

The robot's USB camera is never opened a second time: frames come from the
vision process's MJPEG stream (or from a PC webcam).
"""

import base64
import json
import secrets
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
DEPLOY_DIR = PROJECT_DIR / "src" / "main" / "deploy"
CONFIG_PATH = DEPLOY_DIR / "color_ranges.json"
sys.path.insert(0, str(DEPLOY_DIR))

from calibrate_hsv import (  # noqa: E402
    RESERVED_NAMES,
    mask_preview,
    read_stream_frame,
    sample_object_and_background,
    save_profile,
)
from color_mask import estimate_profile  # noqa: E402


PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Калибровка HSV</title>
<style>
:root { --bg:#14202a; --panel:#233441; --text:#f4f7f9; --muted:#c8d2da;
        --ok:#38b882; --warn:#e2b34a; --bad:#e2675a; }
body { font: 15px system-ui,sans-serif; background:var(--bg); color:var(--text);
       max-width:1000px; margin:20px auto; padding:0 16px; }
h1 { margin:0 0 6px; font-size:24px; }
p, li { color:var(--muted); }
ol { margin:6px 0 0; padding-left:20px; }
label { display:inline-flex; flex-direction:column; gap:4px; margin:6px 10px 6px 0; }
input,select,button { font:inherit; padding:7px 9px; border-radius:7px; border:1px solid #8092a0; }
input,select { background:#fff; color:#15232e; }
button { background:var(--ok); color:#092317; border:0; cursor:pointer; margin:6px 6px 6px 0; }
button:disabled { opacity:.45; cursor:not-allowed; }
button.secondary { background:#c7d5df; color:#14202a; }
button.danger { background:var(--bad); color:#fff; }
.panel { background:var(--panel); border-radius:12px; padding:14px 16px; margin:14px 0; }
.row { display:flex; flex-wrap:wrap; align-items:flex-end; }
.mode label { flex-direction:row; align-items:center; gap:6px; }
#view { display:none; width:100%; max-width:960px; height:auto; image-rendering:pixelated;
        border:2px solid #96a6b1; cursor:crosshair; touch-action:none; }
#status { min-height:24px; white-space:pre-wrap; }
.stats { display:flex; gap:18px; flex-wrap:wrap; margin-top:8px; }
.stat { background:#1a2832; padding:8px 12px; border-radius:8px; }
.stat b { display:block; font-size:20px; }
.good { color:var(--ok); } .meh { color:var(--warn); } .badc { color:var(--bad); }
#ranges { font-family:ui-monospace,monospace; font-size:13px; white-space:pre-wrap;
          overflow-wrap:anywhere; color:var(--muted); }
</style>
</head>
<body>
<h1>Калибровка цвета объекта</h1>
<ol>
  <li>«Получить кадр», обведите <b>весь объект</b> рамкой и нажмите «Добавить образец».
      Всё вокруг рамки автоматически считается фоном.</li>
  <li>Повторите на 3–6 кадрах: близко и далеко, в тени и на свету, на полу и на подиуме.</li>
  <li>«Рассчитать» — зелёным показана маска, которую увидит робот.</li>
  <li>«Проверить на новом кадре» несколько раз. Где не ловит — добавьте образец там и пересчитайте.</li>
  <li>«Сохранить в проект», затем деплой на робота.</li>
</ol>

<div class="panel">
  <div class="row">
    <label>Имя объекта
      <input id="name" list="names" placeholder="например, red_cube" style="width:170px">
      <datalist id="names"></datalist>
    </label>
    <label>Источник
      <select id="source">
        <option value="robot">Камера робота (рекомендуется)</option>
        <option value="pc">Камера ПК</option>
      </select>
    </label>
    <label>Адрес VMX-pi <input id="host" value="raspberrypi.local" style="width:160px"></label>
    <label>Порт <input id="port" type="number" value="1186" min="1" max="65535" style="width:80px"></label>
    <label>Камера ПК № <input id="camera" type="number" value="0" min="0" style="width:70px"></label>
  </div>
  <div class="row mode">
    <span style="margin-right:10px">Рамкой выделяю:</span>
    <label><input type="radio" name="mode" value="object" checked> объект</label>
    <label><input type="radio" name="mode" value="background"> фон, который НЕ должен ловиться</label>
    <label><input type="checkbox" id="showMask" checked> показывать маску</label>
  </div>
  <div class="row">
    <button id="capture">Получить кадр</button>
    <button id="add" disabled>Добавить образец</button>
    <button id="compute" disabled>Рассчитать</button>
    <button id="check" class="secondary" disabled>Проверить на новом кадре</button>
    <button id="save" disabled>Сохранить в проект</button>
    <button id="reset" class="danger">Сбросить образцы</button>
    <button id="close" class="secondary">Завершить</button>
  </div>
  <div id="status" role="status">Введите имя объекта и получите кадр.</div>
</div>

<div class="panel">
  <canvas id="view"></canvas>
  <div class="stats">
    <div class="stat">Образцов<b id="samples">0</b></div>
    <div class="stat">Объект покрыт<b id="coverage">—</b></div>
    <div class="stat">Фон ловится<b id="falsepos">—</b></div>
    <div class="stat">Маска в рамке<b id="inroi">—</b></div>
  </div>
  <div id="ranges"></div>
</div>

<script>
const token = "__TOKEN__";
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const $ = id => document.getElementById(id);
let frameImg = null, maskImg = null, frameId = null, drag = null;
let objectRoi = null, backgroundRois = [], computed = false;

function mode() { return document.querySelector('input[name=mode]:checked').value; }
function setStatus(text) { statusEl.textContent = text; }
function nameValue() { return $('name').value.trim(); }

async function post(path, data) {
  const response = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({token, ...data})});
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || 'Ошибка запроса');
  return payload;
}
function point(event) {
  const r = canvas.getBoundingClientRect();
  return {x:Math.max(0, Math.min(canvas.width, Math.round((event.clientX-r.left)*canvas.width/r.width))),
          y:Math.max(0, Math.min(canvas.height, Math.round((event.clientY-r.top)*canvas.height/r.height)))};
}
function rect(a, b) {
  return [Math.min(a.x,b.x), Math.min(a.y,b.y), Math.abs(b.x-a.x), Math.abs(b.y-a.y)];
}
function strokeRoi(roi, color) {
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.strokeRect(...roi);
}
function redraw(liveEnd) {
  if (!frameImg) return;
  const base = ($('showMask').checked && maskImg) ? maskImg : frameImg;
  ctx.drawImage(base, 0, 0);
  if (objectRoi) strokeRoi(objectRoi, '#00ff57');
  for (const r of backgroundRois) strokeRoi(r, '#ff4040');
  if (drag && liveEnd) strokeRoi(rect(drag, liveEnd), mode()==='object' ? '#00ff57' : '#ff4040');
}
function updateButtons() {
  $('add').disabled = !(frameId && objectRoi && nameValue());
  $('compute').disabled = !(Number($('samples').textContent) > 0);
  $('check').disabled = !computed;
  $('save').disabled = !computed;
}
function pct(value, digits) { return (100*value).toFixed(digits) + ' %'; }
function showStats(stats) {
  if (!stats) return;
  if (stats.samples !== undefined) $('samples').textContent = stats.samples;
  if (stats.objectCoverage !== undefined) {
    const c = stats.objectCoverage;
    $('coverage').textContent = pct(c, 1);
    $('coverage').className = c >= 0.9 ? 'good' : c >= 0.75 ? 'meh' : 'badc';
  }
  if (stats.backgroundFalsePositive !== undefined) {
    const f = stats.backgroundFalsePositive;
    $('falsepos').textContent = pct(f, 2);
    $('falsepos').className = f <= 0.003 ? 'good' : f <= 0.01 ? 'meh' : 'badc';
  }
  if (stats.maskInRoi !== undefined && stats.maskInRoi !== null) {
    $('inroi').textContent = pct(stats.maskInRoi, 0);
  }
}
async function loadImage(src) {
  const img = new Image(); img.src = src; await img.decode(); return img;
}
async function showFrame(data) {
  frameImg = await loadImage(data.image);
  maskImg = data.preview ? await loadImage(data.preview) : null;
  frameId = data.frameId;
  canvas.width = data.width; canvas.height = data.height;
  canvas.style.display = 'block';
  redraw();
}

canvas.addEventListener('pointerdown', e => {
  if (!frameImg) return;
  drag = point(e); canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointermove', e => { if (drag) redraw(point(e)); });
canvas.addEventListener('pointerup', e => {
  if (!drag) return;
  const roi = rect(drag, point(e)); drag = null;
  if (roi[2] < 4 || roi[3] < 4) { redraw(); return; }
  if (mode() === 'object') objectRoi = roi; else backgroundRois.push(roi);
  redraw(); updateButtons();
  setStatus(mode()==='object'
    ? 'Объект выделен. «Добавить образец», чтобы учесть этот кадр.'
    : 'Фон отмечен ('+backgroundRois.length+'). Он добавится вместе с образцом.');
});
$('showMask').onchange = () => redraw();
$('name').oninput = updateButtons;

async function capture(check) {
  setStatus(check ? 'Беру новый кадр и проверяю маску…' : 'Получаю кадр…');
  objectRoi = null; backgroundRois = [];
  try {
    const data = await post('/capture', {
      name: nameValue(), source: $('source').value, host: $('host').value,
      port: Number($('port').value), camera: Number($('camera').value)
    });
    await showFrame(data);
    showStats(data.stats);
    setStatus(data.preview
      ? 'Зелёным — маска текущего профиля. Обведите объект, если он пропущен или нужно добавить образец.'
      : 'Обведите рамкой весь объект.');
  } catch (error) { setStatus('Ошибка: '+error.message); }
  updateButtons();
}
$('capture').onclick = () => capture(false);
$('check').onclick = () => capture(true);

$('add').onclick = async () => {
  try {
    const data = await post('/add', {name:nameValue(), roi:objectRoi,
                                     backgroundRois, frameId});
    showStats(data.stats);
    objectRoi = null; backgroundRois = [];
    redraw();
    setStatus('Образец добавлен (всего '+data.stats.samples+'). Добавьте ещё с другим светом/расстоянием или «Рассчитать».');
  } catch (error) { setStatus('Ошибка: '+error.message); }
  updateButtons();
};
$('compute').onclick = async () => {
  setStatus('Подбираю диапазон…');
  try {
    const data = await post('/compute', {name:nameValue(), frameId, roi:objectRoi});
    computed = true;
    if (data.preview) { maskImg = await loadImage(data.preview); redraw(); }
    showStats(data.stats);
    $('ranges').textContent = 'HSV: ' + JSON.stringify(data.ranges);
    const c = data.stats.objectCoverage, f = data.stats.backgroundFalsePositive;
    let advice = 'Готово. Проверьте на новых кадрах, потом сохраните.';
    if (c < 0.75) advice = 'Покрытие низкое: выделяйте объект плотнее или добавьте образцы.';
    if (f > 0.01) advice = 'Фон ловится: отметьте мешающий фон (режим «фон») и пересчитайте.';
    setStatus(advice);
  } catch (error) { setStatus('Ошибка: '+error.message); }
  updateButtons();
};
$('save').onclick = async () => {
  try {
    const data = await post('/save', {name:nameValue()});
    let text = 'Сохранено «'+data.name+'» в '+data.path+'.';
    if (data.removed.length) text += '\nУдалены старые профили: '+data.removed.join(', ');
    setStatus(text + '\nТеперь задеплойте проект на робота.');
    loadNames();
  } catch (error) { setStatus('Ошибка: '+error.message); }
};
$('reset').onclick = async () => {
  try {
    await post('/reset', {name:nameValue()});
    computed = false; maskImg = null;
    $('samples').textContent = '0'; $('coverage').textContent = '—';
    $('falsepos').textContent = '—'; $('inroi').textContent = '—'; $('ranges').textContent = '';
    redraw(); setStatus('Образцы этого объекта сброшены.');
  } catch (error) { setStatus('Ошибка: '+error.message); }
  updateButtons();
};
$('close').onclick = async () => {
  try { await post('/shutdown', {}); } catch (_) {}
  setStatus('Скрипт остановлен. Вкладку можно закрыть.');
};
async function loadNames() {
  try {
    const data = await (await fetch('/profiles')).json();
    $('names').innerHTML = '';
    for (const name of data.names) {
      const option = document.createElement('option'); option.value = name;
      $('names').appendChild(option);
    }
  } catch (_) {}
}
loadNames();
</script>
</body></html>"""


def read_pc_camera(index):
    camera = cv2.VideoCapture(index)
    try:
        if not camera.isOpened():
            raise ValueError("Не удалось открыть камеру ПК %d" % index)
        frame = None
        for _ in range(15):
            ok, candidate = camera.read()
            if ok:
                frame = candidate
        if frame is None:
            raise ValueError("Камера ПК не вернула кадр")
        return frame
    finally:
        camera.release()


def encode_jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise ValueError("Не удалось закодировать кадр")
    return "data:image/jpeg;base64," + base64.b64encode(
        encoded.tobytes()).decode("ascii")


def saved_ranges(name):
    """Ranges of an already saved profile with this name, or None."""
    try:
        with CONFIG_PATH.open("r", encoding="utf-8") as source:
            for item in json.load(source)["objects"]:
                if item["name"].casefold() == name.casefold():
                    return item["ranges"]
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


class Session:
    """Samples collected for one object name."""

    def __init__(self):
        self.positives = []
        self.negatives = []
        self.ranges = None
        self.stats = None

    def sample_count(self):
        return len(self.positives)


class CalibrationServer(ThreadingHTTPServer):
    def __init__(self):
        super().__init__(("127.0.0.1", 0), CalibrationHandler)
        self.token = secrets.token_urlsafe(24)
        self.frame = None
        self.frame_id = None
        self.sessions = {}
        self.lock = threading.Lock()

    def session(self, name):
        return self.sessions.setdefault(name.casefold(), Session())


def check_name(name):
    if (not isinstance(name, str) or not name.strip()
            or name.strip().casefold() in RESERVED_NAMES):
        raise ValueError("Введите имя объекта (например, red_cube)")
    return name.strip()


def check_roi(roi):
    if (not isinstance(roi, list) or len(roi) != 4
            or any(type(value) is not int for value in roi)):
        raise ValueError("Выделите объект рамкой на кадре")
    return roi


def mask_share_in_roi(mask, roi):
    if roi is None:
        return None
    x, y, w, h = roi
    part = mask[y:y + h, x:x + w]
    return float((part > 0).mean()) if part.size else None


class CalibrationHandler(BaseHTTPRequestHandler):
    def log_message(self, _format, *_args):
        pass

    def send_data(self, code, payload, content_type="application/json"):
        if isinstance(payload, dict):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        else:
            body = payload.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self.send_data(
                200, PAGE.replace("__TOKEN__", self.server.token), "text/html")
        elif self.path == "/profiles":
            try:
                with CONFIG_PATH.open("r", encoding="utf-8") as source:
                    names = [item["name"] for item in json.load(source)["objects"]]
                self.send_data(200, {"names": names})
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.send_data(500, {"error": str(error)})
        else:
            self.send_data(404, {"error": "Не найдено"})

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 16384:
                raise ValueError("Неверный размер запроса")
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError("Неверный запрос")
            if request.get("token") != self.server.token:
                raise ValueError("Неверный код сессии")
            handler = {
                "/capture": self.capture, "/add": self.add,
                "/compute": self.compute, "/save": self.save,
                "/reset": self.reset, "/shutdown": self.shutdown_server,
            }.get(self.path)
            if handler is None:
                self.send_data(404, {"error": "Не найдено"})
            else:
                handler(request)
        except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
            self.send_data(400, {"error": str(error)})

    def current_frame(self, frame_id):
        with self.server.lock:
            if frame_id != self.server.frame_id or self.server.frame is None:
                raise ValueError("Кадр устарел — получите новый")
            return self.server.frame.copy()

    def capture(self, request):
        source = request.get("source")
        if source == "robot":
            host = str(request.get("host", "")).strip()
            port = request.get("port")
            if (not host or any(char in host for char in "/@?#")
                    or type(port) is not int or not 1 <= port <= 65535):
                raise ValueError("Укажите адрес VMX-pi и порт 1–65535")
            frame = read_stream_frame("http://%s:%d/?action=stream" % (host, port))
        elif source == "pc":
            index = request.get("camera")
            if type(index) is not int or not 0 <= index <= 8:
                raise ValueError("Индекс камеры ПК должен быть от 0 до 8")
            frame = read_pc_camera(index)
        else:
            raise ValueError("Выберите источник кадра")

        frame_id = secrets.token_urlsafe(12)
        with self.server.lock:
            self.server.frame = frame
            self.server.frame_id = frame_id

        # Preview with the profile being calibrated, else the saved one.
        reply = {"image": encode_jpeg(frame), "width": frame.shape[1],
                 "height": frame.shape[0], "frameId": frame_id}
        name = request.get("name")
        if isinstance(name, str) and name.strip():
            session = self.server.session(name.strip())
            ranges = session.ranges or saved_ranges(name.strip())
            if ranges:
                overlay, _ = mask_preview(frame, ranges)
                reply["preview"] = encode_jpeg(overlay)
            reply["stats"] = {"samples": session.sample_count()}
        self.send_data(200, reply)

    def add(self, request):
        name = check_name(request.get("name"))
        roi = check_roi(request.get("roi"))
        background = request.get("backgroundRois") or []
        if not isinstance(background, list) or len(background) > 20:
            raise ValueError("Слишком много областей фона")
        background = [check_roi(item) for item in background]
        frame = self.current_frame(request.get("frameId"))
        positives, negatives = sample_object_and_background(
            frame, roi, background, seed=len(self.server.sessions))
        with self.server.lock:
            session = self.server.session(name)
            session.positives.append(positives)
            session.negatives.append(negatives)
            count = session.sample_count()
        self.send_data(200, {"stats": {"samples": count}})

    def compute(self, request):
        name = check_name(request.get("name"))
        with self.server.lock:
            session = self.server.session(name)
            if not session.positives:
                raise ValueError("Сначала добавьте хотя бы один образец")
            positives = np.concatenate(session.positives)
            negatives = np.concatenate(session.negatives)
        ranges, stats = estimate_profile(positives, negatives)
        with self.server.lock:
            session.ranges = ranges
            session.stats = stats
            stats["samples"] = session.sample_count()
        reply = {"ranges": ranges, "stats": stats}
        try:
            frame = self.current_frame(request.get("frameId"))
            overlay, mask = mask_preview(frame, ranges)
            reply["preview"] = encode_jpeg(overlay)
            roi = request.get("roi")
            stats["maskInRoi"] = mask_share_in_roi(
                mask, check_roi(roi) if roi else None)
        except ValueError:
            pass
        self.send_data(200, reply)

    def save(self, request):
        name = check_name(request.get("name"))
        with self.server.lock:
            ranges = self.server.session(name).ranges
        if not ranges:
            raise ValueError("Сначала нажмите «Рассчитать»")
        saved, removed = save_profile(CONFIG_PATH, name, ranges)
        self.send_data(200, {"name": saved, "removed": removed,
                             "ranges": ranges, "path": str(CONFIG_PATH)})

    def reset(self, request):
        name = check_name(request.get("name"))
        with self.server.lock:
            self.server.sessions.pop(name.casefold(), None)
        self.send_data(200, {"ok": True})

    def shutdown_server(self, _request):
        self.send_data(200, {"ok": True})
        threading.Thread(target=self.server.shutdown, daemon=True).start()


def main():
    server = CalibrationServer()
    url = "http://127.0.0.1:%d/" % server.server_port
    print("HSV calibration page: %s" % url)
    webbrowser.open(url)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
