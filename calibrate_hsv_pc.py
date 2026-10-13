#!/usr/bin/env python3
"""Run from PyCharm to calibrate HSV in a browser, without terminal input.

The page reads the robot's existing video stream by default, or the PC
webcam. It saves named HSV profiles directly into this project's deploy
folder. It never opens the robot's USB camera a second time.
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


PROJECT_DIR = Path(__file__).resolve().parent
DEPLOY_DIR = PROJECT_DIR / "src" / "main" / "deploy"
CONFIG_PATH = DEPLOY_DIR / "color_ranges.json"
sys.path.insert(0, str(DEPLOY_DIR))

from calibrate_hsv import (  # noqa: E402
    estimate_ranges,
    read_stream_frame,
    save_profile,
    select_samples,
)


PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Калибровка HSV</title>
<style>
body { font: 16px system-ui,sans-serif; background:#14202a; color:#f4f7f9;
       max-width:940px; margin:24px auto; padding:0 16px; }
h1 { margin-bottom:8px; }
p { color:#c8d2da; }
label { display:inline-flex; flex-direction:column; gap:5px; margin:8px 12px 8px 0; }
input,select,button { font:inherit; padding:8px; border-radius:7px; border:1px solid #8092a0; }
input,select { background:#fff; color:#15232e; }
button { background:#38b882; color:#092317; border:0; cursor:pointer; margin:8px 8px 8px 0; }
button:disabled { opacity:.45; cursor:not-allowed; }
button.secondary { background:#c7d5df; color:#14202a; }
.panel { background:#233441; border-radius:12px; padding:16px; margin:16px 0; }
.hint { font-size:14px; }
#view { display:none; max-width:100%; width:800px; height:auto; image-rendering:pixelated;
        border:2px solid #96a6b1; cursor:crosshair; touch-action:none; }
#status { min-height:28px; white-space:pre-wrap; }
#result { white-space:pre-wrap; overflow-wrap:anywhere; }
</style>
</head>
<body>
<h1>Калибровка цвета объекта</h1>
<p>Запустите кадр, выделите мышью <b>небольшой участок внутри объекта</b>
без фона и нажмите «Сохранить». Значения HSV запишутся в проект на ПК.</p>
<div class="panel">
  <label>Имя объекта
    <input id="name" list="names" placeholder="Например, Red ball">
    <datalist id="names"></datalist>
  </label>
  <label>Источник
    <select id="source">
      <option value="robot">Камера робота (рекомендуется)</option>
      <option value="pc">Камера ПК</option>
    </select>
  </label>
  <label>Адрес VMX-pi <input id="host" value="raspberrypi.local"></label>
  <label>Порт <input id="port" type="number" value="1186" min="1" max="65535" style="width:90px"></label>
  <label>Камера ПК № <input id="camera" type="number" value="0" min="0" style="width:80px"></label>
  <div>
    <button id="capture">Получить кадр</button>
    <button id="save" disabled>Сохранить HSV в проект</button>
    <button id="close" class="secondary">Завершить</button>
  </div>
  <div id="status" role="status">Выберите объект и получите кадр.</div>
</div>
<div class="panel">
  <canvas id="view"></canvas>
  <p class="hint">Тяните мышью по цветной части объекта. ROI выбирается прямо здесь,
  нажимать Enter не нужно. Если камера ПК показывает другие оттенки, используйте поток робота.</p>
  <div id="result"></div>
</div>
<script>
const token = "__TOKEN__";
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const status = document.getElementById('status');
const result = document.getElementById('result');
const saveButton = document.getElementById('save');
let image = null, frameId = null, start = null, roi = null;

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
function redraw(end) {
  if (!image) return;
  ctx.drawImage(image, 0, 0);
  if (start && end) {
    ctx.strokeStyle = '#00ff57'; ctx.lineWidth = 2;
    ctx.strokeRect(start.x, start.y, end.x-start.x, end.y-start.y);
  } else if (roi) {
    ctx.strokeStyle = '#00ff57'; ctx.lineWidth = 2;
    ctx.strokeRect(...roi);
  }
}
canvas.addEventListener('pointerdown', e => {
  if (!image) return;
  start = point(e); roi = null; saveButton.disabled = true;
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointermove', e => {
  if (start) redraw(point(e));
});
canvas.addEventListener('pointerup', e => {
  if (!start) return;
  const end = point(e);
  roi = [Math.min(start.x,end.x), Math.min(start.y,end.y),
         Math.abs(end.x-start.x), Math.abs(end.y-start.y)];
  start = null;
  redraw();
  if (roi[2] < 4 || roi[3] < 4) {
    roi = null; status.textContent = 'Выделите область побольше.';
  } else {
    saveButton.disabled = false;
    status.textContent = 'ROI: x='+roi[0]+' y='+roi[1]+' ширина='+roi[2]+' высота='+roi[3];
  }
});
document.getElementById('capture').onclick = async () => {
  status.textContent = 'Получаю кадр…'; result.textContent = '';
  saveButton.disabled = true; roi = null; image = null; frameId = null;
  try {
    const data = await post('/capture', {
      source:document.getElementById('source').value,
      host:document.getElementById('host').value,
      port:Number(document.getElementById('port').value),
      camera:Number(document.getElementById('camera').value)
    });
    const loaded = new Image();
    loaded.src = data.image;
    await loaded.decode();
    image = loaded; frameId = data.frameId;
    canvas.width = data.width; canvas.height = data.height;
    canvas.style.display = 'block';
    redraw();
    status.textContent = 'Выделите мышью участок внутри объекта.';
  } catch (error) { status.textContent = 'Ошибка: '+error.message; }
};
saveButton.onclick = async () => {
  const name = document.getElementById('name').value.trim();
  if (!name) { status.textContent = 'Введите имя объекта.'; return; }
  if (!roi || !frameId) { status.textContent = 'Сначала выделите ROI.'; return; }
  saveButton.disabled = true;
  try {
    const data = await post('/save', {name, roi, frameId});
    result.textContent = 'Сохранено в проект: '+data.path+'\nHSV: '+JSON.stringify(data.ranges);
    status.textContent = 'Готово. Теперь разверните проект на VMX-pi.';
    document.getElementById('name').value = data.name;
    loadNames();
  } catch (error) { status.textContent = 'Ошибка: '+error.message; }
  finally { saveButton.disabled = false; }
};
document.getElementById('close').onclick = async () => {
  try { await post('/shutdown', {}); } catch (_) {}
  status.textContent = 'Скрипт остановлен. Эту вкладку можно закрыть.';
};
async function loadNames() {
  try {
    const response = await fetch('/profiles');
    const data = await response.json();
    document.getElementById('names').innerHTML = '';
    for (const name of data.names) {
      const option = document.createElement('option'); option.value = name;
      document.getElementById('names').appendChild(option);
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


class CalibrationServer(ThreadingHTTPServer):
    def __init__(self):
        super().__init__(("127.0.0.1", 0), CalibrationHandler)
        self.token = secrets.token_urlsafe(24)
        self.frame = None
        self.frame_id = None
        self.lock = threading.Lock()


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
                200, PAGE.replace("__TOKEN__", self.server.token),
                "text/html",
            )
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
            if not 0 < length <= 8192:
                raise ValueError("Неверный размер запроса")
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError("Неверный запрос")
            if request.get("token") != self.server.token:
                raise ValueError("Неверный код сессии")

            if self.path == "/capture":
                self.capture(request)
            elif self.path == "/save":
                self.save(request)
            elif self.path == "/shutdown":
                self.send_data(200, {"ok": True})
                threading.Thread(
                    target=self.server.shutdown, daemon=True,
                ).start()
            else:
                self.send_data(404, {"error": "Не найдено"})
        except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
            self.send_data(400, {"error": str(error)})

    def capture(self, request):
        source = request.get("source")
        if source == "robot":
            host = str(request.get("host", "")).strip()
            port = request.get("port")
            if (not host or any(char in host for char in "/@?#")
                    or type(port) is not int or not 1 <= port <= 65535):
                raise ValueError("Укажите адрес VMX-pi и порт 1–65535")
            frame = read_stream_frame(
                "http://%s:%d/?action=stream" % (host, port)
            )
        elif source == "pc":
            index = request.get("camera")
            if type(index) is not int or not 0 <= index <= 8:
                raise ValueError("Индекс камеры ПК должен быть от 0 до 8")
            frame = read_pc_camera(index)
        else:
            raise ValueError("Выберите источник кадра")

        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92],
        )
        if not ok:
            raise ValueError("Не удалось показать кадр")
        frame_id = secrets.token_urlsafe(12)
        with self.server.lock:
            self.server.frame = frame
            self.server.frame_id = frame_id
        image = "data:image/jpeg;base64," + base64.b64encode(
            encoded.tobytes()
        ).decode("ascii")
        self.send_data(200, {
            "image": image, "width": frame.shape[1],
            "height": frame.shape[0], "frameId": frame_id,
        })

    def save(self, request):
        name = request.get("name")
        roi = request.get("roi")
        if (not isinstance(name, str) or not name.strip()
                or name.strip().casefold() in ("none", "color", "color_none")):
            raise ValueError("Введите название объекта")
        if (not isinstance(roi, list) or len(roi) != 4
                or any(type(value) is not int for value in roi)):
            raise ValueError("Выделите область объекта на кадре")
        with self.server.lock:
            if request.get("frameId") != self.server.frame_id:
                raise ValueError("Кадр устарел — получите новый")
            frame = self.server.frame.copy()

        samples = select_samples(
            frame, roi, min_saturation=40, min_value=40,
        )
        ranges = estimate_ranges(
            samples, hue_margin=6, sv_margin=20,
            min_saturation=40, min_value=40,
        )
        actual_name = save_profile(
            CONFIG_PATH, name.strip(), ranges, dry_run=False,
        )
        self.send_data(200, {
            "name": actual_name, "ranges": ranges, "path": str(CONFIG_PATH),
        })


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
