#!/usr/bin/env python3
"""Field map and route planner in the browser (run on the PC).

    python3 field_planner.py

  1. Enter the real field size in cm, draw walls, obstacles, podiums, zones
     and objects (drag a rectangle, then type exact cm values in the panel).
     A top-down photo or plan of the field can be put underneath.
  2. Set the robot start point and the direction it faces at Start.
  3. Click the route points. Each point: drive forward or back, barrier()
     (obstacle avoidance), stop by the front Sharp instead of the encoder,
     and actions after arriving (visionPickup, putDownObject, turn, drive by
     the side ultrasonics, ...).
  4. The planner simulates the robot with the same sensor logic as
     DriveMotor (side ultrasonics and the front Sharp measure distances to
     the drawn walls), shows collisions and gives ready "case" code: paste it
     into the switch (stateAutomatic) in DriveMotor.java and deploy.

Maps are saved in field_maps/ next to this script (JSON, with the photo).
"""

import json
import os
import re
import secrets
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
MAPS_DIR = PROJECT_DIR / "field_maps"
# three.js for the 3D view, kept in the project so it works without internet
VENDOR_DIR = PROJECT_DIR / "vendor"
VENDOR_FILES = {"three.min.js", "OrbitControls.js"}
NAME_PATTERN = re.compile(r"^[\w\- ]{1,60}$")
# A map with a field photo can be several MB.
MAX_REQUEST_BYTES = 25_000_000


PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Карта поля и маршрут</title>
<style>
:root { --bg:#0b0714; --glass:rgba(24,15,42,.62); --glass2:rgba(14,9,26,.80);
        --line:rgba(196,181,253,.14); --line2:rgba(196,181,253,.28);
        --text:#ece8f6; --muted:#958ab3; --accent:#a78bfa; --accent2:#f0abfc;
        --ok:#6ee7b7; --warn:#fbbf24; --bad:#fb7185; }
* { box-sizing:border-box; }
html, body { height:100%; }
body { margin:0; font:13px/1.35 "Inter", system-ui, sans-serif; background:var(--bg);
       color:var(--text); display:flex; flex-direction:column; gap:10px;
       padding:12px clamp(28px, 7vw, 120px); overflow:hidden; }
#sky { position:fixed; inset:0; width:100%; height:100%; z-index:0; pointer-events:none; }
header, #tlBar, main { position:relative; z-index:1; }
.glass, header, #tlBar, aside, #stage { background:var(--glass);
       border:1px solid var(--line); border-radius:14px;
       backdrop-filter:blur(14px) saturate(120%); -webkit-backdrop-filter:blur(14px) saturate(120%); }
header { display:flex; flex-wrap:wrap; gap:6px; align-items:center; padding:8px 12px; }
header h1 { font-size:14px; font-weight:600; letter-spacing:.06em; text-transform:uppercase;
            margin:0 12px 0 2px; color:var(--accent); }
button, select, input { font:inherit; color:var(--text); background:transparent;
         border:1px solid var(--line2); border-radius:8px; padding:4px 9px;
         transition:background .15s, border-color .15s, color .15s; }
select option { background:#140c24; }
button { cursor:pointer; }
button:hover { border-color:var(--accent); color:#fff; background:rgba(167,139,250,.10); }
button.on { background:var(--accent); border-color:var(--accent); color:#120a22; font-weight:600; }
button.bad { border-color:rgba(251,113,133,.45); color:#fecdd3; }
button.bad:hover { background:rgba(251,113,133,.14); border-color:var(--bad); }
input:focus, select:focus { outline:none; border-color:var(--accent); }
input[type=number] { width:74px; }
input[type=text] { width:150px; }
input[type=range] { accent-color:var(--accent); border:none; padding:0; }
input[type=checkbox] { accent-color:var(--accent); }
input[type=file] { border:none; padding:0; color:var(--muted); }
main { flex:1; display:flex; gap:10px; min-height:0; }
#stage { flex:1; position:relative; min-width:0; overflow:hidden; background:rgba(11,7,20,.72); }
canvas#cv { position:absolute; inset:0; width:100%; height:100%; display:block; }
#hint, #cursor { position:absolute; bottom:10px; background:rgba(11,7,20,.75);
        border:1px solid var(--line); padding:4px 9px; border-radius:8px;
        color:var(--muted); pointer-events:none; font-size:12px; }
#hint { left:10px; max-width:70%; } #cursor { right:10px; color:var(--text); }
aside { width:380px; overflow:auto; padding:10px; scrollbar-width:thin;
        scrollbar-color:var(--line2) transparent; }
section { border-bottom:1px solid var(--line); padding:8px 4px 12px; margin-bottom:6px; }
section:last-child { border-bottom:none; }
section h2 { font-size:11px; font-weight:600; letter-spacing:.08em; text-transform:uppercase;
             color:var(--accent); margin:0 0 8px; }
.row { display:flex; flex-wrap:wrap; gap:6px; align-items:center; margin:5px 0; }
.row label { color:var(--muted); }
.muted { color:var(--muted); font-size:11.5px; }
.act { display:flex; gap:4px; align-items:center; margin:4px 0; }
.act select { flex:1; min-width:0; }
textarea { width:100%; height:240px; font:11.5px ui-monospace,SFMono-Regular,monospace;
           background:var(--glass2); color:#ddd3f5; border:1px solid var(--line);
           border-radius:10px; padding:8px; resize:vertical; }
#warnings div { padding:4px 8px; border-radius:7px; margin:3px 0; cursor:pointer;
                border-left:3px solid; background:rgba(255,255,255,.03); }
.w-bad { border-color:var(--bad) !important; } .w-warn { border-color:var(--warn) !important; }
.w-ok { border-color:var(--ok) !important; }
#status { margin-left:auto; color:var(--muted); }
#tlBar { display:flex; gap:10px; align-items:stretch; padding:8px 10px; }
#tlControls { display:flex; flex-direction:column; gap:5px; justify-content:center; min-width:150px; }
#tlControls .row { margin:0; }
#tlScrub { width:150px; }
#timeline { flex:1; display:flex; gap:6px; overflow-x:auto; overflow-y:hidden;
            padding:2px 2px 6px; scroll-behavior:smooth; scrollbar-width:thin;
            scrollbar-color:var(--line2) transparent; }
.card { flex:0 0 auto; width:116px; background:rgba(255,255,255,.035);
        border:1px solid var(--line); border-top:3px solid var(--line2); border-radius:10px;
        padding:5px 8px; cursor:pointer; display:flex; flex-direction:column; gap:2px;
        position:relative; transition:background .15s, border-color .15s; }
.card:hover { background:rgba(167,139,250,.08); }
.card.sel { width:252px; border-color:var(--accent); background:rgba(167,139,250,.10); cursor:default; }
.card .top { display:flex; justify-content:space-between; color:var(--muted); font-size:10.5px; }
.card .big { font-size:14px; font-weight:600; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.card .code { font:10.5px ui-monospace,monospace; color:#b4a8d6; white-space:nowrap;
              overflow:hidden; text-overflow:ellipsis; }
.card .ed { display:flex; flex-wrap:wrap; gap:4px; align-items:center; margin-top:3px; font-size:12px; }
.card .ed input[type=number] { width:62px; padding:2px 5px; }
.card .ed button { padding:2px 7px; }
.card.move { border-top-color:#a78bfa; } .card.back { border-top-color:#f0abfc; }
.card.rotate { border-top-color:#818cf8; } .card.pickup { border-top-color:#6ee7b7; }
.card.place { border-top-color:#5eead4; } .card.sonic { border-top-color:#fb7185; }
.card.other { border-top-color:#8b84a8; }
.card .badge { position:absolute; top:-6px; right:-6px; background:var(--bad); color:#fff;
               border-radius:9px; font-size:10.5px; padding:0 5px; }
.card .badge.warn { background:var(--warn); color:#1a1205; }
.wpGap { flex:0 0 auto; width:1px; background:var(--line2); margin:8px 3px; }
#view3d { position:absolute; inset:0; display:none; }
#view3d canvas { display:block; }
#sec3d input[type=range] { width:150px; }
#hud { position:absolute; display:none; gap:6px; padding:6px 8px; z-index:5;
       background:rgba(19,11,36,.94); border:1px solid var(--accent); border-radius:9px;
       box-shadow:0 6px 24px rgba(0,0,0,.45); }
#hud label { display:flex; flex-direction:column; gap:2px; font-size:10.5px; color:var(--muted); }
#hud input { width:78px; padding:3px 6px; font-size:13px; }
#skTools { display:none; flex-wrap:wrap; gap:4px; align-items:center; }
#skTools .sep { width:1px; height:22px; background:var(--line2); margin:0 4px; }
#skTools button.con { min-width:30px; padding:4px 6px; }
@keyframes fadeUp { from { opacity:0; transform:translateY(6px); } to { opacity:1; transform:none; } }
@keyframes popIn { from { opacity:0; transform:scale(.93); } to { opacity:1; transform:none; } }
.fadein { animation:fadeUp .32s cubic-bezier(.2,.8,.2,1) both; }
button { transition:background .18s, border-color .18s, color .18s, transform .12s, box-shadow .18s; }
button:active { transform:scale(.96); }
button.on { box-shadow:0 0 0 3px rgba(167,139,250,.18); }
.card { transition:background .18s, border-color .18s, transform .18s, box-shadow .18s; }
.card:hover { transform:translateY(-2px); box-shadow:0 6px 18px rgba(0,0,0,.35); }
.card.sel { box-shadow:0 0 0 1px var(--accent), 0 8px 24px rgba(124,58,237,.35); }
.card.just { animation:popIn .24s cubic-bezier(.2,.8,.2,1) both; }
#hud { animation:popIn .16s ease; }
#cv, #view3d { transition:opacity .35s ease; }
#status.flash { animation:fadeUp .3s ease; }
input, select { transition:border-color .18s, background .18s; }
@media (prefers-reduced-motion: reduce) { *, *::before, *::after { animation:none !important; transition:none !important; } }
@media (max-width:900px) { body { padding:8px; } aside { width:300px; } }
</style>
</head>
<body>
<canvas id="sky"></canvas>
<header>
  <h1>Карта поля</h1>
 <button id="mode3d" title="3D: поле, маршрут и робот с лифтом, рукой и клешнёй">3D</button>
  <button id="modeSketch" title="Эскиз: точное черчение по размерам и зависимостям">✎ Эскиз</button>
  <span class="row" id="tools"></span>
  <span id="skTools"></span>
  <label class="muted">привязка</label>
  <select id="snap"><option value="1">1 см</option><option value="5" selected>5 см</option>
    <option value="10">10 см</option></select>
  <label class="muted">углы</label>
  <select id="angleSnap"><option value="0">любые</option><option value="45">45°</option>
    <option value="90" selected>90°</option></select>
  <button id="undo" title="Ctrl+Z">Отменить</button>
  <button id="fit">Вписать</button>
  <span id="status"></span>
</header>
<div id="tlBar">
  <div id="tlControls">
    <div class="row"><button id="tlPrev" title="←">◀</button><button id="tlPlay">▶ Пуск</button>
      <button id="tlNext" title="→">▶</button></div>
    <input type="range" id="tlScrub" min="0" max="0" step="0.01" value="0">
    <div class="row"><span class="muted" id="tlInfo">шагов: 0</span>
      <select id="tlSpeed"><option value="0.5">×0.5</option><option value="1" selected>×1</option>
        <option value="2">×2</option><option value="4">×4</option></select></div>
  </div>
  <div id="timeline"></div>
</div>
<main>
  <div id="stage"><canvas id="cv"></canvas><div id="view3d"></div><div id="hud"></div><div id="hint"></div><div id="cursor"></div></div>
  <aside>
    <section id="sec3d" style="display:none">
      <h2>3D и механизмы</h2>
      <div class="row"><label>камера</label><button data-cam="orbit">Свободно</button><button data-cam="top">Сверху</button>
        <button data-cam="follow">За роботом</button></div>
      <div class="muted">Мышь: левая — вращать, правая — сдвигать, колесо — ближе/дальше. Таймлайн и «Пуск» двигают робота.</div>
      <div class="row"><label>лифт</label><input type="range" id="mech_lift" min="15" max="300"><span id="mechv_lift" class="muted"></span></div>
      <div class="row"><label>рука</label><input type="range" id="mech_arm" min="10" max="165"><span id="mechv_arm" class="muted"></span></div>
      <div class="row"><label>клешня</label><input type="range" id="mech_claw" min="18" max="170"><span id="mechv_claw" class="muted"></span></div>
      <div class="row"><label><input type="checkbox" id="mechManual"> двигать вручную (иначе — по таймлайну)</label></div>
      <h2 style="margin-top:10px">Модель робота (.3mf)</h2>
      <div class="row"><input type="text" id="modelPath" style="flex:1;min-width:0" placeholder="/путь/к/Robot.3mf">
        <button id="modelLoad">Загрузить</button></div>
      <div id="model3dInfo"></div>
      <div class="row"><button id="modelApply">Взять размеры и датчики из модели</button></div>
      <div id="jointBox" style="display:none">
        <div class="row"><label>лифт: см на градус</label><input type="number" id="jLiftCm" step="0.005">
          <label>в модели угол</label><input type="number" id="jLiftModel" step="1"></div>
        <div class="row"><label>рука: в модели угол</label><input type="number" id="jArmModel" step="1">
          <label>знак</label><input type="number" id="jArmSign" step="2" min="-1" max="1"></div>
        <div class="row"><label>ось руки x,y,z см</label><input type="number" id="jPx" step="0.5"><input type="number" id="jPy" step="0.5"><input type="number" id="jPz" step="0.5"></div>
        <div class="row"><label>ход пальца клешни, см</label><input type="number" id="jClaw" step="0.1"></div>
        <div class="row"><label><input type="checkbox" id="pickMode"> выбирать детали кликом</label></div>
        <div class="row"><span id="pickedName" class="muted">—</span>
          <select id="pickedGroup"><option value="base">корпус</option><option value="lift">лифт</option>
            <option value="arm">рука</option><option value="clawL">клешня, левый палец</option>
            <option value="clawR">клешня, правый палец</option><option value="wheel">колесо</option></select></div>
        <div class="row"><button id="jointsSave">Сохранить механизмы</button></div>
        <div class="muted">Если деталь двигается не с тем механизмом — включите выбор кликом, кликните её и поменяйте группу. Ось руки — где серво руки (муфта).</div>
      </div>
    </section>
    <section id="secFile">
      <h2>Файл</h2>
      <div class="row"><input type="text" id="mapName" value="field">
        <button id="save">Сохранить</button>
        <select id="mapList"><option value="">— открыть —</option></select>
        <button id="newMap" class="bad">Новая</button></div>
      <div class="row"><label>Поле, см</label>
        <input type="number" id="fieldW" min="20" step="1"> ×
        <input type="number" id="fieldH" min="20" step="1"></div>
      <div class="row"><label>Фото/план поля</label>
        <input type="file" id="bgFile" accept="image/*" style="width:190px">
        <label>прозр.</label><input type="range" id="bgOpacity" min="0" max="1" step="0.05" style="width:70px">
        <button id="bgClear">убрать</button></div>
      <div class="muted">Фото растягивается на всё поле: обрежьте его по краям поля.</div>
    </section>
    <section id="secRobot">
      <h2>Робот</h2>
      <div class="row"><label>ширина</label><input type="number" id="rW" step="0.5">
        <label>длина</label><input type="number" id="rL" step="0.5"></div>
      <div class="row"><label>соник от центра вбок</label><input type="number" id="rSonicSide" step="0.5"></div>
      <div class="row"><label>соник от центра вперёд (− = сзади)</label><input type="number" id="rSonicAlong" step="0.5"></div>
      <div class="row"><label>Sharp от центра вперёд</label><input type="number" id="rSharp" step="0.5">
        <label>и вбок</label><input type="number" id="rSharpSide" step="0.5"></div>
      <div class="row"><label>объезд: стоп при Sharp ≤</label><input type="number" id="aObs" step="1">
        <label>чисто ≥</label><input type="number" id="aClear" step="1"></div>
      <div class="row"><label>запас вбок</label><input type="number" id="aExtra" step="1">
        <label>проезд</label><input type="number" id="aFwd" step="1"></div>
      <div class="row"><label>старт: x</label><input type="number" id="sX" step="1">
        <label>y</label><input type="number" id="sY" step="1">
        <label>смотрит</label><input type="number" id="sB" step="1" title="0 = вверх, 90 = вправо, по часовой"></div>
      <div class="muted">Направление: 0° = вверх по карте, 90° = вправо (по часовой, как гироскоп).
        На старте гироскоп = 0, все повороты считаются от этого направления.</div>
    </section>
    <section id="secDrive">
      <h2>Езда и маршрут</h2>
      <div class="row"><label>режим</label><button id="driveNormal">Обычная</button><button id="driveSmooth">Smooth motion</button></div>
      <div class="muted" id="driveHint"></div>
      <div class="row"><label><input type="checkbox" id="optCorrect"> коррекция по датчикам</label>
        <label><input type="checkbox" id="optBarrier"> barrier на всех отрезках</label></div>
      <h2 style="margin-top:10px">Автомаршрут по объектам</h2>
      <div class="row"><label>подъезд</label><button id="apClose">Близко</button><button id="apFar">Далеко</button>
        <input type="number" id="apCloseCm" step="1" title="близко, см от Sharp до объекта">
        <input type="number" id="apFarCm" step="1" title="далеко, см"></div>
      <div class="row"><label><input type="checkbox" id="optZone"> отвозить в зону</label>
        <label><input type="checkbox" id="optReturn"> вернуться на старт</label></div>
      <div class="row"><button id="buildRoute">Построить маршрут</button></div>
      <div class="muted">Порядок — ближайший следующий; путь — в обход стен и подиумов с запасом на ширину робота; углы — по настройке «углы» вверху.</div>
    </section>
    <section id="secMethods">
      <h2>Методы проекта</h2>
      <div class="row"><input type="text" id="projPath" style="flex:1;min-width:0" placeholder="папка проекта">
        <button id="scanMethods">Найти</button></div>
      <div id="methodList" class="muted">Укажите папку проекта и нажмите «Найти».</div>
    </section>
    <section id="selPanel"><h2>Выбрано</h2><div class="muted">Ничего. Кликните по элементу.</div></section>
    <section>
      <h2>Код маршрута</h2>
      <div class="row"><label>первый case</label><input type="number" id="firstCase" min="0" step="1">
        <button id="copy">Копировать</button><button id="saveCode">Сохранить .java.txt</button></div>
      <div id="warnings"></div>
      <textarea id="code" readonly></textarea>
      <div class="muted">Вставьте внутрь switch (stateAutomatic) в DriveMotor.java вместо старых case
        (default: оставьте). Затем deploy.</div>
    </section>
  </aside>
</main>
<script>
"use strict";
const TOKEN = "__TOKEN__";
const TYPES = {
  wall:     {name:"Стена",      fill:"#8b84a8", solid:true},
  obstacle: {name:"Препятствие",fill:"#fb923c", solid:true},
  podium:   {name:"Подиум",     fill:"#c084fc", solid:true},
  zone:     {name:"Зона",       fill:"#5eead4", solid:false},
  sudden:   {name:"Внезапное",  fill:"#f43f5e", solid:true},
};
const ACTIONS = {
  pickup:    {name:"visionPickup() — забрать объект", value:false},
  place:     {name:"putDownObject() — положить", value:false},
  rotate:    {name:"повернуться на направление, °", value:true, def:0},
  sharp:     {name:"вперёд до Sharp, см", value:true, def:20},
  left:      {name:"влево до стены (левый соник), см", value:true, def:20},
  right:     {name:"вправо до стены (правый соник), см", value:true, def:20},
  leftAway:  {name:"влево, пока правый соник ≥, см", value:true, def:50},
  rightAway: {name:"вправо, пока левый соник ≥, см", value:true, def:50},
  back:      {name:"назад, см", value:true, def:20},
  alignF:    {name:"встать по Sharp спереди (в обе стороны), см", value:true, def:20},
  alignL:    {name:"встать по левому сонику (в обе стороны), см", value:true, def:20},
  alignR:    {name:"встать по правому сонику (в обе стороны), см", value:true, def:20},
  method:    {name:"метод проекта…", value:"method"},
  wait:      {name:"подождать, с", value:true, def:1},
  custom:    {name:"свой код Java", value:"text", def:"stateAutomatic++;"},
};
const TOOLS = [["select","Выбор"],["wall","Стена"],["obstacle","Препятствие"],
  ["podium","Подиум"],["zone","Зона"],["sudden","Внезапное"],["object","Объект"],["start","Старт"],
  ["route","Маршрут"],["measure","Линейка"]];
const HINTS = {
  select:"Клик — выбрать, тянуть — двигать (угол прямоугольника — размер). Пустое место — сдвиг карты, колесо — масштаб. Delete — удалить.",
  wall:"Тяните прямоугольник. Точные размеры — в панели справа.",
  obstacle:"Препятствие можно объехать: включите barrier у отрезка, который через него идёт.",
  podium:"Подиум — твёрдый, робот в него не въезжает.",
  zone:"Зона (например, куда класть) — робот через неё ездит.",
  sudden:"Внезапное препятствие: автомаршрут о нём не знает. Проверка, объедет ли робот его с barrier().",
  object:"Клик — поставить объект.",
  start:"Клик — центр робота на старте, тяните — направление.",
  route:"Клик — следующая точка маршрута. Действия в точке — в панели справа.",
  measure:"Два клика — расстояние между точками.",
};

let S = null;            // the map (saved state)
let tool = "select";
let selected = null;     // {kind:"rect"|"object"|"route"|"start", id}
let view = {scale:2, ox:40, oy:400};
let drag = null;
let measure = null;      // {a:{x,y}, b:{x,y}}
let cursorCm = null;
let sim = null;
let bgImage = null;
const undoStack = [];
let nextId = 1;
let viewTouched = false;

function defaultMap() {
  return {version:1, field:{w:300, h:200},
    robot:{w:36.9, l:40.4, sonicSide:6.9, sonicAlong:-16.2, sharpFront:11.7, sharpSide:14.1,
      avoidObstacle:15, avoidClear:35, avoidExtra:12, avoidForward:40},
    opts:defaultOpts(), methods:{path:"", list:[], hidden:[]},
    start:{x:30, y:30, b:0}, rects:[], objects:[], route:[], firstCase:0,
    bg:{data:null, opacity:0.5},
    sketch:{points:[], lines:[], circles:[], arcs:[], cons:[]}};
}
function uid() { return "e" + (nextId++) + "_" + Math.random().toString(36).slice(2, 6); }

// ── geometry ────────────────────────────────────────────────────────────
const rad = d => d * Math.PI / 180;
const norm360 = a => ((a % 360) + 360) % 360;
const angDiff = (a, b) => { let d = norm360(a - b); return d > 180 ? d - 360 : d; };
const dirOf = b => ({x:Math.sin(rad(b)), y:Math.cos(rad(b))});   // 0 = up, clockwise
const bearingTo = (a, b) => norm360(Math.atan2(b.x - a.x, b.y - a.y) * 180 / Math.PI);
const r1 = v => Math.round(v * 10) / 10;
const fmt = v => (Math.abs(v - Math.round(v)) < 0.05 ? String(Math.round(v)) : r1(v).toFixed(1));

function solids() {
  return S.rects.filter(r => TYPES[r.type].solid);
}
/** Distance from p along bearing b to the first solid edge or field border. */
/** Corners of an oriented rectangle: center c, bearing b, half length hl
 *  (along b), half width hw. */
function orientedRect(c, b, hl, hw) {
  const f = dirOf(b), s = dirOf(b + 90);
  return [[1,1],[1,-1],[-1,-1],[-1,1]].map(([i, j]) =>
    ({x:c.x + f.x * hl * i + s.x * hw * j, y:c.y + f.y * hl * i + s.y * hw * j}));
}
function polyHitsRect(poly, r) {
  const rp = [{x:r.x,y:r.y},{x:r.x+r.w,y:r.y},{x:r.x+r.w,y:r.y+r.h},{x:r.x,y:r.y+r.h}];
  const axes = [{x:1,y:0},{x:0,y:1}];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    axes.push({x:-(b.y - a.y), y:b.x - a.x});
  }
  for (const ax of axes) {
    const proj = pts => pts.map(p => p.x * ax.x + p.y * ax.y);
    const p1 = proj(poly), p2 = proj(rp);
    if (Math.max(...p1) <= Math.min(...p2) + 1e-6 || Math.max(...p2) <= Math.min(...p1) + 1e-6) return false;
  }
  return true;
}
function polyOutside(poly) {
  return poly.some(p => p.x < -0.01 || p.y < -0.01 || p.x > S.field.w + 0.01 || p.y > S.field.h + 0.01);
}
function circleHitsRect(c, rr, r) {
  const nx = Math.max(r.x, Math.min(c.x, r.x + r.w)), ny = Math.max(r.y, Math.min(c.y, r.y + r.h));
  return Math.hypot(c.x - nx, c.y - ny) < rr;
}

/** First wall hit from p along bearing b: {t, n} (n = wall normal). With
 *  onlyTypes, only those rectangle types count (and no border / sketch). */
function rayHit(p, b, onlyTypes) {
  const d = dirOf(b);
  let best = {t:Infinity, n:null};
  const take = (t, n) => { if (t >= 0 && t < best.t) best = {t, n}; };
  if (!onlyTypes) {
    if (d.x > 1e-9) take((S.field.w - p.x) / d.x, {x:-1, y:0});
    if (d.x < -1e-9) take((0 - p.x) / d.x, {x:1, y:0});
    if (d.y > 1e-9) take((S.field.h - p.y) / d.y, {x:0, y:-1});
    if (d.y < -1e-9) take((0 - p.y) / d.y, {x:0, y:1});
  }
  for (const r of solids()) {
    if (onlyTypes && !onlyTypes.includes(r.type)) continue;
    let t0 = -Infinity, t1 = Infinity, n0 = null, miss = false;
    const ax = [[p.x, d.x, r.x, r.x + r.w, "x"], [p.y, d.y, r.y, r.y + r.h, "y"]];
    for (const [o, v, lo, hi, axis] of ax) {
      if (Math.abs(v) < 1e-9) { if (o < lo || o > hi) { miss = true; break; } continue; }
      let a = (lo - o) / v, c = (hi - o) / v, na = -1;
      if (a > c) { [a, c] = [c, a]; na = 1; }
      if (a > t0) { t0 = a; n0 = axis === "x" ? {x:na, y:0} : {x:0, y:na}; }
      t1 = Math.min(t1, c);
    }
    if (miss || t0 > t1 || t1 < 0) continue;
    take(Math.max(0, t0), n0 || {x:-d.x, y:-d.y});
  }
  if (!onlyTypes) for (const [a, c] of skWallsCache) {
    const t = skRaySeg(p, d, a, c);
    if (t < best.t) {
      const u = skUnit(a, c);
      let n = {x:-u.y, y:u.x};
      if (n.x * d.x + n.y * d.y > 0) n = {x:-n.x, y:-n.y};
      best = {t, n};
    }
  }
  return best;
}
function rayCast(p, b) { return rayHit(p, b).t; }
const perpendicularHit = (hit, b) => {
  if (!hit.n || !Number.isFinite(hit.t)) return false;
  const d = dirOf(b);
  return Math.abs(hit.n.x * d.x + hit.n.y * d.y) >= Math.cos(rad(8));
};
const AVOID_TYPES = ["obstacle", "sudden"];
function segRectDist(a, b, r) {
  const poly = [{x:r.x, y:r.y}, {x:r.x + r.w, y:r.y}, {x:r.x + r.w, y:r.y + r.h}, {x:r.x, y:r.y + r.h}];
  if (skSegHitsPoly(a, b, poly)) return 0;
  const dSeg = (p, q, c) => { const dx = q.x - p.x, dy = q.y - p.y, L2 = dx * dx + dy * dy || 1e-12;
    const t = Math.max(0, Math.min(1, ((c.x - p.x) * dx + (c.y - p.y) * dy) / L2));
    return Math.hypot(p.x + dx * t - c.x, p.y + dy * t - c.y); };
  let m = Infinity;
  for (const c of poly) m = Math.min(m, dSeg(a, b, c));
  for (const e of [a, b]) {
    const nx = Math.max(r.x, Math.min(e.x, r.x + r.w)), ny = Math.max(r.y, Math.min(e.y, r.y + r.h));
    m = Math.min(m, Math.hypot(e.x - nx, e.y - ny));
  }
  return m;
}
function segSegDist(a, b, c, d) {
  const cross = (o, p, q) => (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
  if (cross(a, b, c) * cross(a, b, d) < 0 && cross(c, d, a) * cross(c, d, b) < 0) return 0;
  const dSeg = (p, q, e) => { const dx = q.x - p.x, dy = q.y - p.y, L2 = dx * dx + dy * dy || 1e-12;
    const t = Math.max(0, Math.min(1, ((e.x - p.x) * dx + (e.y - p.y) * dy) / L2));
    return Math.hypot(p.x + dx * t - e.x, p.y + dy * t - e.y); };
  return Math.min(dSeg(a, b, c), dSeg(a, b, d), dSeg(c, d, a), dSeg(c, d, b));
}
function defaultOpts() {
  return {drive:"normal", correct:true, barrierAll:false, approach:"close", closeCm:12, farCm:35,
    toZone:false, returnStart:false};
}
function robotParams() {
  const R = S.robot;
  return {sharpSide:R.sharpSide !== undefined ? R.sharpSide : Math.max(0, R.w / 2 - 3),
    obstacle:R.avoidObstacle || 15, clear:R.avoidClear || 35, extra:R.avoidExtra || 12,
    forward:R.avoidForward || 40, sideWall:12};
}

// ── simulation and code ─────────────────────────────────────────────────
function simulate() {
  const R = S.robot, O = Object.assign(defaultOpts(), S.opts || {}), A = robotParams();
  const smooth = O.drive === "smooth";
  const out = {steps:[], warnings:[], moves:[], stops:[], rotations:[], detours:[]};
  skWallsCache = skWallSegments();
  let pose = {x:S.start.x, y:S.start.y, b:norm360(S.start.b)};
  let headingKnown = true;
  const hl = R.l / 2, hw = R.w / 2, rr = Math.hypot(hl, hw);
  const f0 = dirOf(S.start.b), r0 = dirOf(S.start.b + 90);
  const toStart = p => ({x:(p.x - S.start.x) * r0.x + (p.y - S.start.y) * r0.y,
    y:(p.x - S.start.x) * f0.x + (p.y - S.start.y) * f0.y});
  const relH = b => norm360(b - S.start.b);
  const offset = (p, side, dist) => { const s = dirOf(p.b + 90); return {x:p.x + s.x * side * dist, y:p.y + s.y * side * dist}; };
  const sensorPoint = p => { const f = dirOf(p.b); return {x:p.x + f.x * R.sonicAlong, y:p.y + f.y * R.sonicAlong}; };
  const sonicHit = (p, side) => rayHit(sensorPoint(p), p.b + side * 90);
  const sonicReading = (p, side) => sonicHit(p, side).t - R.sonicSide;
  const sharpHit = (p, side, types) => rayHit(offset(p, side, A.sharpSide), p.b, types);
  const sharpReading = p => Math.min(sharpHit(p, -1).t, sharpHit(p, 1).t) - R.sharpFront;
  let curSrc = null;      // what in the map made this step (for the timeline)
  const step = (code, wp, note) => {
    const last = out.steps[out.steps.length - 1];
    if (last) last.after = {...pose};
    out.steps.push({code, wp, note, src:curSrc, before:{...pose}});
  };
  const warn = (level, wp, text) => {
    if (!out.warnings.some(w => w.wp === wp && w.text === text)) out.warnings.push({level, wp, text});
  };
  const nameOf = r => TYPES[r.type].name.toLowerCase() + (r.label ? " «" + r.label + "»" : "");

  /** Swept area of a straight move; hits are reported. skipAvoid: the
   *  obstacle types barrier() handles are left to the detour check. */
  function checkSwept(from, to, b, wp, alongMove, skipAvoid, rotating, drawIt = true) {
    const dist = Math.hypot(to.x - from.x, to.y - from.y);
    const mid = {x:(from.x + to.x) / 2, y:(from.y + to.y) / 2};
    if (rotating) {
      if (drawIt) out.moves.push({from:{...from}, to:{...to}, b, capsule:rr, wp});
      for (const r of solids()) {
        if (skipAvoid && AVOID_TYPES.includes(r.type)) continue;
        if (segRectDist(from, to, r) < rr) warn("bad", wp, "столкновение (поворот на ходу): " + nameOf(r));
      }
      if (skWallsCache.some(([a, c]) => segSegDist(from, to, a, c) < rr)) warn("bad", wp, "столкновение со стеной эскиза (поворот на ходу)");
      if ([from, to].some(p => p.x < rr - 0.5 || p.y < rr - 0.5 || p.x > S.field.w - rr + 0.5 || p.y > S.field.h - rr + 0.5))
        warn("warn", wp, "при повороте на ходу может задеть край поля");
      return;
    }
    const poly = alongMove ? orientedRect(mid, b, hl + dist / 2, hw) : orientedRect(mid, b, hl, hw + dist / 2);
    if (drawIt) out.moves.push({from:{...from}, to:{...to}, b, poly, wp});
    if (polyOutside(poly)) warn("bad", wp, "робот выезжает за поле");
    if (skWallsCache.some(([a, c]) => skSegHitsPoly(a, c, poly))) warn("bad", wp, "столкновение со стеной эскиза");
    for (const r of solids()) {
      if (skipAvoid && AVOID_TYPES.includes(r.type)) continue;
      if (!polyHitsRect(poly, r)) continue;
      warn("bad", wp, "столкновение: " + nameOf(r)
        + (AVOID_TYPES.includes(r.type) ? " (включите barrier — объедет)" : ""));
    }
  }
  function rotateTo(b, wp, force) {
    b = norm360(b);
    if (!force && headingKnown && Math.abs(angDiff(b, pose.b)) < 0.5) return;
    step("rotateTheRobot(" + fmt(relH(b)) + ");", wp);
    out.rotations.push({c:{x:pose.x, y:pose.y}, r:rr, wp});
    for (const r of solids()) if (circleHitsRect(pose, rr, r)) { warn("warn", wp, "при повороте может задеть " + nameOf(r)); break; }
    if (skWallsCache.some(([a, c]) => skSegCircle(a, c, pose, rr))) warn("warn", wp, "при повороте может задеть стену эскиза");
    pose.b = b; headingKnown = true;
  }
  function lateral(dir, distance, wp) {           // dir -1 left, +1 right
    const d = dirOf(pose.b + dir * 90);
    const to = {x:pose.x + d.x * distance, y:pose.y + d.y * distance};
    checkSwept(pose, to, pose.b, wp, false, false, false);
    pose.x = to.x; pose.y = to.y;
  }
  function along(distance, wp) {
    const d = dirOf(pose.b);
    const to = {x:pose.x + d.x * distance, y:pose.y + d.y * distance};
    checkSwept(pose, to, pose.b, wp, true, false, false);
    pose.x = to.x; pose.y = to.y;
  }
  /** Free circle around p (field border, walls, obstacles, sketch). */
  function roomAt(p, radius) {
    return p.x >= radius && p.y >= radius && p.x <= S.field.w - radius && p.y <= S.field.h - radius
      && !solids().some(r => circleHitsRect(p, radius, r))
      && !skWallsCache.some(([a, c]) => skSegCircle(a, c, p, radius));
  }
  /** Turning while moving from a to b: the half diagonal sweeps a capsule. */
  function capsuleHits(a, b) {
    if ([a, b].some(p => p.x < rr || p.y < rr || p.x > S.field.w - rr || p.y > S.field.h - rr)) return true;
    return solids().some(r => segRectDist(a, b, r) < rr) || skWallsCache.some(([c, d]) => segSegDist(a, b, c, d) < rr);
  }
  /** Area of the robot moving a -> b with heading h (any direction). */
  function hullOf(a, b, h) {
    const pts = orientedRect(a, h, hl, hw).concat(orientedRect(b, h, hl, hw))
      .sort((p, q) => p.x - q.x || p.y - q.y);
    const cross = (o, p, q) => (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
    const lower = [], upper = [];
    for (const p of pts) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
    for (const p of pts.slice().reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
    return lower.slice(0, -1).concat(upper.slice(0, -1));
  }
  function hullHits(a, b, h) {
    const hull = hullOf(a, b, h);
    return polyOutside(hull) || skWallsCache.some(([c, d]) => skSegHitsPoly(c, d, hull)) || solids().some(r => polyHitsRect(hull, r));
  }
  function sweptHull(a, b, h, wp) {
    const hull = hullOf(a, b, h);
    out.moves.push({from:{...a}, to:{...b}, b:h, poly:hull, wp});
    if (polyOutside(hull)) warn("bad", wp, "робот выезжает за поле");
    if (skWallsCache.some(([c, d]) => skSegHitsPoly(c, d, hull))) warn("bad", wp, "столкновение со стеной эскиза");
    for (const r of solids()) if (polyHitsRect(hull, r))
      warn("bad", wp, "столкновение: " + nameOf(r) + (AVOID_TYPES.includes(r.type) ? " (включите barrier — объедет)" : ""));
  }
  /** Drives forward `total` cm with barrier(): the same avoidance as the
   *  robot (stop at the Sharp threshold, sidestep to the side with more
   *  room, extra step, pass AVOID_FORWARD, return; retry while the obstacle
   *  is still beside). The detour is drawn and checked for collisions. */
  function forwardWithBarrier(total, wp) {
    const start = {...pose}, f = dirOf(pose.b), sd = dirOf(pose.b + 90);
    const pts = [{x:pose.x, y:pose.y}];
    let travelled = 0, guard = 0;
    const sweptPoly = (from, to, alongMove) => {
      const dist = Math.hypot(to.x - from.x, to.y - from.y), mid = {x:(from.x + to.x) / 2, y:(from.y + to.y) / 2};
      return alongMove ? orientedRect(mid, pose.b, hl + dist / 2, hw) : orientedRect(mid, pose.b, hl, hw + dist / 2);
    };
    const moveTo = (to, alongMove, final) => {
      checkSwept({x:pose.x, y:pose.y}, to, pose.b, wp, alongMove, true, false, false);
      const poly = sweptPoly(pose, to, alongMove);
      for (const r of solids()) if (AVOID_TYPES.includes(r.type) && polyHitsRect(poly, r))
        warn("bad", wp, final
          ? "Sharp не увидят " + nameOf(r) + " (оно уже, чем расстояние между датчиками, или сбоку) — врежется"
          : "объезд задевает " + nameOf(r) + " — увеличьте запас/проезд объезда");
      pose.x = to.x; pose.y = to.y; pts.push({x:to.x, y:to.y});
    };
    while (guard++ < 4) {
      const remaining = total - travelled;
      const near = Math.min(...[-1, 1].map(s => sharpHit(pose, s, AVOID_TYPES).t - R.sharpFront));
      // like the robot: an obstacle at the very end of the step is its target wall
      if (!Number.isFinite(near) || remaining <= near + 5) break;
      const s = Math.max(0, near - A.obstacle);
      moveTo({x:pose.x + f.x * s, y:pose.y + f.y * s}, true); travelled += s;
      const blocked = [-1, 1].map(side => sharpHit(pose, side, AVOID_TYPES).t - R.sharpFront <= A.obstacle + 0.5);
      let dir = blocked[0] && blocked[1]
        ? (sonicReading(pose, 1) >= sonicReading(pose, -1) ? 1 : -1)
        : (blocked[0] ? 1 : -1);
      if (sonicReading(pose, dir) < A.sideWall) dir = -dir;
      if (sonicReading(pose, dir) < A.sideWall) { warn("bad", wp, "объезд: обе стороны заняты — робот будет стоять"); break; }
      let side = 0;
      while (side < 150) {
        const p = {x:pose.x + sd.x * dir * side, y:pose.y + sd.y * dir * side, b:pose.b};
        if ([-1, 1].every(k => sharpHit(p, k).t - R.sharpFront > A.clear)) break;
        side += 1;
      }
      side += A.extra;
      moveTo({x:pose.x + sd.x * dir * side, y:pose.y + sd.y * dir * side}, false);
      let pass = A.forward, tries = 0;
      for (;;) {
        moveTo({x:pose.x + f.x * pass, y:pose.y + f.y * pass}, true); travelled += pass;
        const back = {x:pose.x - sd.x * dir * side, y:pose.y - sd.y * dir * side};
        const poly = sweptPoly(pose, back, false);
        const beside = solids().some(r => AVOID_TYPES.includes(r.type) && polyHitsRect(poly, r));
        if (!beside || tries >= 6) break;
        pass = 15; tries++;
      }
      moveTo({x:pose.x - sd.x * dir * side, y:pose.y - sd.y * dir * side}, false);
    }
    const rest = total - travelled;
    if (rest > 0.01) moveTo({x:pose.x + f.x * rest, y:pose.y + f.y * rest}, true, true);
    const end = {x:start.x + f.x * Math.max(total, travelled), y:start.y + f.y * Math.max(total, travelled)};
    if (pts.length > 2) out.detours.push({pts, wp});
    out.moves.push({from:{x:start.x, y:start.y}, to:end, b:pose.b,
      poly:orientedRect({x:(start.x + end.x) / 2, y:(start.y + end.y) / 2}, pose.b, hl + Math.hypot(end.x - start.x, end.y - start.y) / 2, hw), wp});
    if (travelled > total + 0.5) warn("warn", wp, "после объезда робот окажется дальше точки на " + fmt(travelled - total) + " см");
    pose.x = end.x; pose.y = end.y;
  }
  /** Sensor alignment at a stop: front Sharp, then the side sonics, each
   *  only where it sees a wall square to the robot in its reliable range. */
  function correctHere(n) {
    const planned = {...pose};
    const fixes = [];
    const front = [-1, 1].map(s => sharpHit(planned, s));
    const fr = Math.min(front[0].t, front[1].t) - R.sharpFront;
    if (fr >= 10 && fr <= 60 && front.every(h => perpendicularHit(h, planned.b))
        && Math.abs(front[0].t - front[1].t) < 2) fixes.push({code:"alignFrontSharp(" + fmt(fr) + ");", n:front[0].n});
    let bestSide = null;
    for (const side of [-1, 1]) {
      const h = sonicHit(planned, side), rd = h.t - R.sonicSide;
      if (rd >= 5 && rd <= 120 && perpendicularHit(h, planned.b + side * 90) && (!bestSide || rd < bestSide.rd))
        bestSide = {side, rd, h};
    }
    if (bestSide) fixes.push({code:(bestSide.side < 0 ? "alignLeftSonic(" : "alignRightSonic(") + fmt(bestSide.rd) + ");", n:bestSide.h.n});
    if (!fixes.length) { warn("warn", n, "коррекция: рядом нет ровной стены для датчиков"); return; }
    for (const fx of fixes) {
      step(fx.code, n, "коррекция");
      if (smooth) {
        const nS = {x:fx.n.x * r0.x + fx.n.y * r0.y, y:fx.n.x * f0.x + fx.n.y * f0.y}, ps = toStart(planned);
        if (Math.abs(nS.x) > Math.cos(rad(5))) step("setPoseX(" + fmt(ps.x) + ");", n);
        else if (Math.abs(nS.y) > Math.cos(rad(5))) step("setPoseY(" + fmt(ps.y) + ");", n);
      }
    }
  }
  function simAction(a, n) {
    const v = Number(a.value);
    switch (a.type) {
      case "pickup":
        step("visionPickup();", n, "захват");
        headingKnown = false;
        warn("ok", n, "после visionPickup робот мог сдвинуться и повернуть: следующий шаг начнётся с поворота");
        break;
      case "place": step("putDownObject();", n); break;
      case "rotate": rotateTo(v, n, true); break;
      case "wait": step("advanceAfterDelay(" + fmt(v) + ");", n); break;
      case "custom": step(String(a.value || "").trim() || "stateAutomatic++;", n); break;
      case "back": step("goBackDistance(" + fmt(v) + ");", n); along(-v, n); break;
      case "sharp": {
        const reading = sharpReading(pose);
        step("goForwardSharp(" + fmt(v) + ");", n);
        if (v < 10) warn("warn", n, "Sharp ближе 10 см не меряет, остановится на 10");
        if (reading > 80) warn("warn", n, "Sharp сейчас видит " + fmt(reading) + " см — может не увидеть стену");
        const move = reading - Math.max(10, v);
        if (move > 0) along(move, n);
        break;
      }
      case "alignF": {
        const reading = sharpReading(pose);
        step("alignFrontSharp(" + fmt(v) + ");", n);
        if (reading > 80) warn("warn", n, "Sharp не видит стену впереди — шаг пропустится");
        else along(reading - Math.max(10, v), n);
        break;
      }
      case "alignL": case "alignR": {
        const side = a.type === "alignL" ? -1 : 1, reading = sonicReading(pose, side);
        step((side < 0 ? "alignLeftSonic(" : "alignRightSonic(") + fmt(v) + ");", n);
        if (reading > 150) warn("warn", n, "соник не видит стену — шаг пропустится");
        else lateral(side, reading - v, n);
        break;
      }
      case "left": case "right": {
        const side = a.type === "left" ? -1 : 1;
        const reading = sonicReading(pose, side);
        step((side < 0 ? "goLeftSonic(" : "goRightSonic(") + fmt(v) + ");", n);
        if (reading > 150) warn("warn", n, "соник видит " + fmt(reading) + " см — дальше 150 см ненадёжно");
        if (reading > v) lateral(side, reading - v, n);
        else warn("warn", n, "соник уже показывает " + fmt(reading) + " см ≤ " + fmt(v) + " — шаг сразу закончится");
        break;
      }
      case "leftAway": case "rightAway": {
        const side = a.type === "leftAway" ? -1 : 1;      // direction of motion
        const reading = sonicReading(pose, -side);        // sonic on the other side
        step((side < 0 ? "goLeftWithRightSonic(" : "goRigtWithLeftSonic(") + fmt(v) + ");", n);
        if (reading < v) lateral(side, v - reading, n);
        else warn("warn", n, "соник уже показывает " + fmt(reading) + " см ≥ " + fmt(v) + " — шаг сразу закончится");
        break;
      }
      case "method": {
        const m = (S.methods && S.methods.list || []).find(x => x.name === a.method);
        const raw = a.args || [];
        const args = raw.map((x, k) => {
          const t = m && m.params[k] ? m.params[k].type : "double";
          if (t === "String") return JSON.stringify(String(x || ""));
          if (t === "boolean") return x ? "true" : "false";
          return fmt(Number(x) || 0);
        });
        const known = {goLeftSonic:"left", goRightSonic:"right", goLeftWithRightSonic:"leftAway",
          goRigtWithLeftSonic:"rightAway", goForwardSharp:"sharp", goBackDistance:"back",
          alignLeftSonic:"alignL", alignRightSonic:"alignR", alignFrontSharp:"alignF",
          visionPickup:"pickup", putDownObject:"place"};
        const first = Number(raw[0]) || 0;
        if (known[a.method]) simAction({type:known[a.method], value:first}, n);
        else if (a.method === "rotateTheRobot") rotateTo(S.start.b + first, n, true);
        else {
          step((a.method || "method") + "(" + args.join(", ") + ");", n);
          if (a.method === "goForwardDistance") along(first, n);
          else warn("ok", n, "как двигает «" + a.method + "» — симуляция не знает");
        }
        break;
      }
    }
  }

  S.route.forEach((wp, i) => {
    const n = i + 1;
    const target = {x:wp.x, y:wp.y};
    const dist = Math.hypot(target.x - pose.x, target.y - pose.y);
    const next = S.route[i + 1];
    const stopHere = !next || wp.actions.length > 0 || wp.align || !!wp.back !== !!next.back;
    const barrier = !wp.back && (wp.barrier || O.barrierAll);
    if (dist >= 0.5) {
      const segB = bearingTo(pose, target);
      const robotB = wp.back ? segB + 180 : segB;
      if (smooth) {
        // Smooth motion only where the robot's body fits: turning on the
        // move sweeps a circle of its half diagonal, a pass-through point
        // cuts the corner by up to SMOOTH_VIA_RADIUS (8 cm).
        let stopPt = stopHere;
        if (!stopPt && !roomAt(target, rr + 8)) {
          stopPt = true;
          warn("ok", n, "тесный угол — здесь остановка (smoothTo вместо smoothVia), чтобы не срезать угол");
        }
        let endB = norm360(barrier || stopPt || !next ? robotB
          : (next.back ? bearingTo(target, next) + 180 : bearingTo(target, next)));
        const ps = toStart(target), from = {x:pose.x, y:pose.y};
        const goCmd = (h, stopNow) => (stopNow ? "smoothTo(" : "smoothVia(") + fmt(ps.x) + ", " + fmt(ps.y) + ", " + fmt(relH(h)) + ");";
        curSrc = {kind:"move", wp:i};
        const turning = Math.abs(angDiff(endB, pose.b)) > 1 || !headingKnown;
        if (barrier) {
          // with barrier() the robot faces the segment and drives straight (the
          // Sharps only look ahead); the corner turn happens on the next one
          step("barrier()." + goCmd(endB, stopPt), n, "точка " + n + (stopPt ? "" : " (без остановки)"));
          if (turning) {   // turns toward the segment at its start, then straight
            out.rotations.push({c:{x:pose.x, y:pose.y}, r:rr, wp:n});
            if (!roomAt(pose, rr)) warn("warn", n, "при повороте в начале отрезка может задеть препятствие");
          }
          pose.b = endB;
          forwardWithBarrier(dist, n);
        } else if (turning && capsuleHits(from, target)) {
          if (headingKnown && !hullHits(from, target, pose.b)) {
            // no room to turn on the move: get there keeping the heading,
            // then turn on the spot (if it fits there)
            step(goCmd(pose.b, true), n, "точка " + n + ", без поворота на ходу (тесно)");
            sweptHull(from, target, pose.b, n);
            pose.x = target.x; pose.y = target.y;
            warn("ok", n, "тесно для поворота на ходу — доедет без поворота и повернётся на месте");
            curSrc = {kind:"rotate", wp:i};
            if (Math.abs(angDiff(endB, pose.b)) > 1) rotateTo(endB, n, false);
          } else {
            // tight everywhere: turn on the spot toward the point, then straight
            curSrc = {kind:"rotate", wp:i};
            rotateTo(robotB, n, !headingKnown);
            curSrc = {kind:"move", wp:i};
            endB = norm360(robotB);
            step(goCmd(endB, true), n, "точка " + n + ", поворот на месте и прямо (тесно)");
            sweptHull(from, target, endB, n);
            pose.x = target.x; pose.y = target.y;
            warn("ok", n, "тесно для smooth — здесь поворот на месте и прямо, как в обычной езде");
          }
        } else {
          step(goCmd(endB, stopPt), n, "точка " + n + (stopPt ? "" : " (без остановки)"));
          if (turning) checkSwept(from, target, pose.b, n, true, false, true);
          else sweptHull(from, target, pose.b, n);
          pose.x = target.x; pose.y = target.y;
        }
        pose.b = endB; headingKnown = true;
        if (wp.sharp) warn("ok", n, "в smooth вместо «стоп по Sharp» используется коррекция по датчикам");
      } else {
        curSrc = {kind:"rotate", wp:i};
        rotateTo(robotB, n, !headingKnown);
        curSrc = {kind:"move", wp:i};
        if (wp.back) {
          step("goBackDistance(" + fmt(dist) + ");", n, "точка " + n);
          along(-dist, n);
        } else if (wp.sharp) {
          const reading = sharpReading({x:target.x, y:target.y, b:pose.b});
          step("goForwardSharp(" + fmt(reading) + ");", n, "точка " + n + ", по Sharp");
          if (reading < 10) warn("bad", n, "Sharp: " + fmt(reading) + " см — ближе 10 см он не меряет (DriveMotor поднимет до 10)");
          else if (reading > 80) warn("warn", n, "Sharp: " + fmt(reading) + " см — дальше ~80 см ненадёжно, лучше энкодер");
          along(dist, n);
        } else {
          step((barrier ? "barrier()." : "") + "goForwardDistance(" + fmt(dist) + ");", n, "точка " + n);
          if (barrier) forwardWithBarrier(dist, n); else along(dist, n);
        }
      }
    } else if (!headingKnown && wp.actions.some(a => a.type !== "rotate")) {
      warn("warn", n, "после захвата направление робота неизвестно — добавьте поворот");
    }
    const firstRotate = wp.actions.length && wp.actions[0].type === "rotate";
    if (firstRotate) { curSrc = {kind:"action", wp:i, ai:0}; simAction(wp.actions[0], n); }
    if (O.correct && (wp.align || wp.actions.some(a => a.type === "pickup" || a.type === "place"))) {
      curSrc = {kind:"correct", wp:i};
      correctHere(n);
    }
    wp.actions.forEach((a, ai) => {
      if (firstRotate && ai === 0) return;
      curSrc = {kind:"action", wp:i, ai};
      simAction(a, n);
    });
    out.stops.push({pose:{...pose}, wp:n,
      sonicL:sonicReading(pose, -1), sonicR:sonicReading(pose, 1), sharp:sharpReading(pose)});
  });
  if (out.steps.length) out.steps[out.steps.length - 1].after = {...pose};
  out.final = pose;
  return out;
}

// ── automatic route over all objects ────────────────────────────────────
function autoRoute() {
  const R = S.robot, O = Object.assign(defaultOpts(), S.opts || {});
  const W = S.field.w, H = S.field.h, cell = 5;
  skWallsCache = skWallSegments();
  const clearR = R.w / 2 + 3, turnR = Math.hypot(R.w, R.l) / 2 + 1;
  // "Внезапное" = a place where an obstacle may appear. The route may pass
  // it with barrier() when there is room to go around on some side; with
  // no room it would block the robot, so it counts as a wall.
  const AP = robotParams();
  const baseStatics = S.rects.filter(r => TYPES[r.type].solid && r.type !== "sudden");
  const overlaps = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
  const stripFree = st => st.x >= 0 && st.y >= 0 && st.x + st.w <= W && st.y + st.h <= H
    && !baseStatics.some(r => overlaps(st, r))
    && !skWallsCache.some(([a, b]) => skSegHitsPoly(a, b, [{x:st.x, y:st.y}, {x:st.x + st.w, y:st.y}, {x:st.x + st.w, y:st.y + st.h}, {x:st.x, y:st.y + st.h}]));
  const need = R.w + AP.extra;
  // room to sidestep: beside it left/right (when driving up/down the map)
  // or below/above (when driving left/right)
  const roomX = r => [{x:r.x - need, y:r.y, w:need, h:r.h}, {x:r.x + r.w, y:r.y, w:need, h:r.h}].some(stripFree);
  const roomY = r => [{x:r.x, y:r.y - need, w:r.w, h:need}, {x:r.x, y:r.y + r.h, w:r.w, h:need}].some(stripFree);
  const hasRoom = r => roomX(r) || roomY(r);
  const suddens = S.rects.filter(r => r.type === "sudden");
  // the two front Sharps must see it: smaller than their spacing in both
  // directions = invisible to the robot, so it is avoided like a wall
  const seen = r => Math.max(r.w, r.h) >= 2 * AP.sharpSide + 2;
  const suddenOpen = suddens.filter(r => hasRoom(r) && seen(r));
  const statics = baseStatics.concat(suddens.filter(r => !suddenOpen.includes(r)));
  const freeAt = (p, rad0) => p.x >= rad0 && p.y >= rad0 && p.x <= W - rad0 && p.y <= H - rad0
    && !statics.some(r => circleHitsRect(p, rad0, r)) && !skWallsCache.some(([a, b]) => skSegCircle(a, b, p, rad0));
  const nx = Math.floor(W / cell) + 1, ny = Math.floor(H / cell) + 1;
  const free = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) free[j * nx + i] = freeAt({x:i * cell, y:j * cell}, clearR) ? 1 : 0;
  // turning needs the half diagonal of the robot: corners only where it fits
  const freeTurn = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++)
    freeTurn[j * nx + i] = free[j * nx + i] && freeAt({x:i * cell, y:j * cell}, turnR) ? 1 : 0;
  // cells where a sudden obstacle may stand cost more: go around when cheap
  let strictSudden = true;
  const pen = new Float32Array(nx * ny);
  const passX = new Uint8Array(nx * ny).fill(1), passY = new Uint8Array(nx * ny).fill(1);
  const rooms = new Map(suddenOpen.map(r => [r, {x:roomX(r), y:roomY(r)}]));
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    for (const r of suddenOpen) {
      if (!circleHitsRect({x:i * cell, y:j * cell}, clearR + 10, r)) continue;
      pen[j * nx + i] = 8;
      // driving up/down past it needs room left/right of it, and the other way
      if (!rooms.get(r).x) passY[j * nx + i] = 0;
      if (!rooms.get(r).y) passX[j * nx + i] = 0;
    }
  }
  const lineFree = (a, b) => {
    const d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(d / 2));
    for (let k = 0; k <= n; k++) if (!freeAt({x:a.x + (b.x - a.x) * k / n, y:a.y + (b.y - a.y) * k / n}, clearR)) return false;
    return true;
  };
  const snap = Number(document.getElementById("angleSnap").value);
  const dirs = snap === 90 ? [[1, 0], [-1, 0], [0, 1], [0, -1]]
    : [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];
  const cellOf = p => [Math.round(p.x / cell), Math.round(p.y / cell)];
  function astar(a, b) {
    const [si, sj] = cellOf(a), [gi, gj] = cellOf(b);
    if (si < 0 || sj < 0 || si >= nx || sj >= ny || gi < 0 || gj < 0 || gi >= nx || gj >= ny) return null;
    const key = (i, j, d) => (j * nx + i) * 9 + d;
    const g = new Map(), came = new Map(), open = [];
    const h = (i, j) => Math.hypot(gi - i, gj - j) * cell;
    g.set(key(si, sj, 8), 0);
    open.push({i:si, j:sj, d:8, f:h(si, sj)});
    let found = null, guardN = 0;
    while (open.length && guardN++ < 60000) {
      let bi = 0;
      for (let k = 1; k < open.length; k++) if (open[k].f < open[bi].f) bi = k;
      const cur = open.splice(bi, 1)[0];
      if (cur.i === gi && cur.j === gj) { found = cur; break; }
      const gc = g.get(key(cur.i, cur.j, cur.d));
      dirs.forEach(([di, dj], dIdx) => {
        const i = cur.i + di, j = cur.j + dj;
        if (i < 0 || j < 0 || i >= nx || j >= ny) return;
        const isGoal = i === gi && j === gj;
        if (!free[j * nx + i] && !isGoal) return;
        if (di && dj && (!free[cur.j * nx + i] || !free[j * nx + cur.i])) return;
        const turn = cur.d !== 8 && cur.d !== dIdx ? 12 : 0;
        if (turn && !freeTurn[cur.j * nx + cur.i]) return;   // no room to turn here
        // where a sudden obstacle may appear, only directions it can be
        // driven around in (a sidestep needs room across the way)
        const c2 = j * nx + i;
        if (strictSudden && ((dj && !passY[c2]) || (di && !passX[c2]))) return;
        const ng = gc + Math.hypot(di, dj) * cell + turn + pen[j * nx + i];
        const k2 = key(i, j, dIdx);
        if (g.has(k2) && g.get(k2) <= ng) return;
        g.set(k2, ng); came.set(k2, key(cur.i, cur.j, cur.d));
        open.push({i, j, d:dIdx, f:ng + h(i, j)});
      });
    }
    if (!found) return null;
    const pts = [];
    let k = key(found.i, found.j, found.d);
    while (k !== undefined) {
      const c = Math.floor(k / 9);
      pts.push({x:(c % nx) * cell, y:Math.floor(c / nx) * cell});
      k = came.get(k);
    }
    pts.reverse();
    pts[0] = {x:a.x, y:a.y}; pts[pts.length - 1] = {x:b.x, y:b.y};
    // grid steps next to the exact start/end points: no tiny zigzags (only
    // where the shortcut keeps the allowed angles)
    const angleOk = (p, q) => !snap || Math.abs(angDiff(bearingTo(p, q), Math.round(bearingTo(p, q) / snap) * snap)) < 0.5;
    const canDrop = q => lineFree(pts[q - 1], pts[q + 1]) && angleOk(pts[q - 1], pts[q + 1]);
    for (let q = pts.length - 2; q >= 1; q--) {
      if (Math.hypot(pts[q].x - pts[q + 1].x, pts[q].y - pts[q + 1].y) < 8 && canDrop(q)) pts.splice(q, 1);
    }
    for (let q = 1; q < pts.length - 1; q++) {
      if (Math.hypot(pts[q].x - pts[q - 1].x, pts[q].y - pts[q - 1].y) < 8 && canDrop(q)) { pts.splice(q, 1); q--; }
    }
    let simp = [pts[0]];
    for (let q = 1; q < pts.length - 1; q++) {
      const p0 = simp[simp.length - 1], p1 = pts[q], p2 = pts[q + 1];
      const c = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
      if (Math.abs(c) > 1e-6) simp.push(p1);
    }
    if (pts.length > 1) simp.push(pts[pts.length - 1]);
    if (snap === 0) {   // any angle: pull the string where the way is free
      const pulled = [simp[0]];
      let q = 0;
      while (q < simp.length - 1) {
        let r = simp.length - 1;
        while (r > q + 1 && !lineFree(simp[q], simp[r])) r--;
        pulled.push(simp[r]); q = r;
      }
      simp = pulled;
    }
    let len = 0;
    for (let q = 1; q < simp.length; q++) len += Math.hypot(simp[q].x - simp[q - 1].x, simp[q].y - simp[q - 1].y) + 12;
    return {pts:simp, len};
  }
  const stopCm = O.approach === "far" ? Number(O.farCm) || 35 : Number(O.closeCm) || 12;
  const angles = snap === 90 ? [0, 90, 180, 270] : [0, 45, 90, 135, 180, 225, 270, 315];
  /** Best way to stand in front of target (robot facing it). */
  function approach(from, target, stop) {
    let best = null;
    for (const a of angles) {
      const f = dirOf(a);
      // as close as allowed: back off from the target (e.g. an object in
      // the middle of a podium) until the robot fits
      let back = stop, stand = null;
      for (; back <= stop + 80; back += 2) {
        const p = {x:target.x - f.x * (back + R.l / 2), y:target.y - f.y * (back + R.l / 2)};
        if (freeAt(p, clearR)) { stand = p; break; }
      }
      if (!stand) continue;
      // a little farther so the stop point lies on the 5 cm grid (no
      // 2-3 degree kinks next to grid paths)
      for (let dd = 0; dd < cell; dd += 0.5) {
        const p = {x:target.x - f.x * (back + dd + R.l / 2), y:target.y - f.y * (back + dd + R.l / 2)};
        const onGrid = v => Math.abs(v / cell - Math.round(v / cell)) < 0.002;
        if (onGrid(p.x) && onGrid(p.y) && freeAt(p, clearR)) { stand = p; back += dd; break; }
      }
      const pre = {x:stand.x - f.x * 20, y:stand.y - f.y * 20};
      const usePre = freeAt(pre, turnR) && lineFree(pre, stand);
      const path = astar(from, usePre ? pre : stand);
      if (!path) continue;
      const turnOk = freeAt(stand, turnR) || usePre;
      const cost = path.len + (usePre ? 20 : 30) + (turnOk ? 0 : 200) + (back - stop) * 2;
      if (!best || cost < best.cost) best = {a, stand, pre:usePre ? pre : null, path, cost};
    }
    return best;
  }
  const route = [];
  const newWp = p => ({id:uid(), x:r1(p.x), y:r1(p.y), back:false, barrier:!!O.barrierAll, sharp:false, align:false, actions:[]});
  const addPath = pts => { for (let q = 1; q < pts.length; q++) route.push(newWp(pts[q])); };
  const arrive = (ap, action) => {
    addPath(ap.path.pts);
    if (ap.pre || !route.length) route.push(newWp(ap.stand));
    const wp = route[route.length - 1];
    wp.align = true;
    if (!ap.pre) wp.actions.push({type:"rotate", value:ap.a});
    wp.actions.push({type:action, value:null});
  };
  let cur = {x:S.start.x, y:S.start.y};
  const left = S.objects.slice(), missed = [];
  const zones = S.rects.filter(r => r.type === "zone");
  while (left.length) {
    let best = null, bi = -1;
    left.forEach((o, k) => { const ap = approach(cur, o, stopCm); if (ap && (!best || ap.cost < best.cost)) { best = ap; bi = k; } });
    let risky = false;
    if (!best) {
      // only past a sudden obstacle with no room to go around: still drive
      // there, but say that the robot stops if it really appears
      strictSudden = false;
      left.forEach((o, k) => { const ap = approach(cur, o, stopCm); if (ap && (!best || ap.cost < best.cost)) { best = ap; bi = k; } });
      strictSudden = true;
      risky = !!best;
    }
    if (!best) { missed.push(...left.map(o => o.name)); break; }
    const obj = left.splice(bi, 1)[0];
    if (risky) missed.push("«" + obj.name + "» — путь мимо внезапного без места для объезда: если оно появится, робот встанет");
    arrive(best, "pickup");
    cur = best.stand;
    if (O.toZone && zones.length) {
      let zb = null;
      for (const z of zones) {
        const ap = approach(cur, {x:z.x + z.w / 2, y:z.y + z.h / 2}, 3);
        if (ap && (!zb || ap.cost < zb.cost)) zb = ap;
      }
      if (zb) { arrive(zb, "place"); cur = zb.stand; }
      else missed.push("зона после «" + obj.name + "»");
    }
  }
  if (O.returnStart) {
    const back = astar(cur, {x:S.start.x, y:S.start.y});
    if (back) { addPath(back.pts); if (route.length) route[route.length - 1].actions.push({type:"rotate", value:S.start.b}); }
    else missed.push("возврат на старт");
  }
  // (almost) straight runs: a plain point that bends the way by less than
  // 10 degrees is not needed if the straight line is free
  for (let k = route.length - 2; k >= 0; k--) {
    const w = route[k];
    if (w.actions.length || w.align) continue;
    const a = k === 0 ? S.start : route[k - 1], b = route[k + 1];
    const bend = Math.abs(angDiff(bearingTo(a, w), bearingTo(w, b)));
    const snapA = Number(document.getElementById("angleSnap").value);
    const keepsAngle = !snapA || Math.abs(angDiff(bearingTo(a, b), Math.round(bearingTo(a, b) / snapA) * snapA)) < 0.5;
    if (bend < 10 && keepsAngle && lineFree(a, b)) route.splice(k, 1);
  }
  // barrier() on every segment that passes where a sudden obstacle may be
  let guarded = 0;
  route.forEach((w, k) => {
    const a = k === 0 ? S.start : route[k - 1];
    if (!w.back && suddenOpen.some(r => segRectDist(a, w, r) < clearR + 10)) { if (!w.barrier) guarded++; w.barrier = true; }
  });
  return {route, missed, guarded, walls:suddens.length - suddenOpen.length};
}

function buildCode(simOut) {
  const lines = [];
  let c = Math.max(0, Math.round(Number(S.firstCase) || 0));
  let lastWp = null;
  for (const s of simOut.steps) {
    if (s.wp !== lastWp) { lines.push("        // ── точка " + s.wp + " ──"); lastWp = s.wp; }
    lines.push("        case " + c + ":" + (s.note ? "   // " + s.note : ""));
    for (const l of s.code.split("\n")) lines.push("          " + l);
    lines.push("          break;");
    c++;
  }
  if (!lines.length) return "// Добавьте точки маршрута (инструмент «Маршрут»).";
  lines.push("        // следующий свободный case: " + c);
  return lines.join("\n");
}

// ── drawing ─────────────────────────────────────────────────────────────
const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");
const toScreen = p => ({x:view.ox + p.x * view.scale, y:view.oy - p.y * view.scale});
const toCm = (sx, sy) => ({x:(sx - view.ox) / view.scale, y:(view.oy - sy) / view.scale});

// ── smooth interface: tweens and a short redraw loop ──
const reduceMotion = !!(window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches);
const easeOut = t => 1 - Math.pow(1 - t, 3);
const easeBack = t => { const c1 = 1.4, c3 = c1 + 1; return 1 + c3 * Math.pow(t - 1, 3) + c1 * Math.pow(t - 1, 2); };
const clamp01 = t => Math.max(0, Math.min(1, t));
const bornAt = new Map();      // element id -> time it appeared (pop-in)
let animUntil = 0, animRunning = false, viewAnim = null, fieldAnim = null, tlAnim = null, lastTlPose = null;
function kick(ms) {
  if (reduceMotion) { viewAnim = null; fieldAnim = null; tlAnim = null; draw(); return; }
  animUntil = Math.max(animUntil, performance.now() + ms);
  if (!animRunning) { animRunning = true; requestAnimationFrame(animTick); }
}
function animTick(now) {
  draw();
  if (now < animUntil) requestAnimationFrame(animTick);
  else { animRunning = false; viewAnim = null; draw(); }
}
function tweenView(to, ms) {
  if (reduceMotion || !ms) { Object.assign(view, to); draw(); return; }
  viewAnim = {from:{...view}, to, t0:performance.now(), ms};
  kick(ms + 30);
}
function stepViewAnim() {
  if (!viewAnim) return;
  const t = clamp01((performance.now() - viewAnim.t0) / viewAnim.ms), e = easeOut(t);
  for (const k of ["scale", "ox", "oy"]) view[k] = viewAnim.from[k] + (viewAnim.to[k] - viewAnim.from[k]) * e;
  if (t >= 1) viewAnim = null;
}
const pop = id => { const t0 = bornAt.get(id); if (t0 === undefined) return 1;
  const t = clamp01((performance.now() - t0) / 300); if (t >= 1) bornAt.delete(id); return Math.max(0, easeBack(t)); };
function born(id) { bornAt.set(id, performance.now()); kick(360); }
function fitView(animated) {
  const r = cv.getBoundingClientRect();
  if (r.width < 50 || r.height < 50) return;   // not laid out yet
  const pad = 40;
  const scale = Math.max(0.2, Math.min((r.width - 2 * pad) / S.field.w, (r.height - 2 * pad) / S.field.h));
  const to = {scale, ox:(r.width - S.field.w * scale) / 2, oy:r.height - (r.height - S.field.h * scale) / 2};
  tweenView(to, animated === true ? 450 : 0);
}
function poly(points, fill, stroke, width) {
  ctx.beginPath();
  points.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width || 1; ctx.stroke(); }
}
function label(text, p, color, align) {
  const s = toScreen(p);
  ctx.font = "12px system-ui"; ctx.textAlign = align || "center"; ctx.textBaseline = "middle";
  ctx.lineWidth = 3; ctx.strokeStyle = "#0b0714"; ctx.strokeText(text, s.x, s.y);
  ctx.fillStyle = color || "#fff"; ctx.fillText(text, s.x, s.y);
}
function drawRobot(p, stroke, fill, dash) {
  const pts = orientedRect(p, p.b, S.robot.l / 2, S.robot.w / 2);
  ctx.setLineDash(dash || []);
  poly(pts, fill, stroke, 2);
  ctx.setLineDash([]);
  const f = dirOf(p.b);
  const a = toScreen(p), b = toScreen({x:p.x + f.x * S.robot.l * 0.6, y:p.y + f.y * S.robot.l * 0.6});
  ctx.strokeStyle = stroke; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
}

function draw() {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== Math.round(r.width * dpr) || cv.height !== Math.round(r.height * dpr)) {
    cv.width = Math.round(r.width * dpr); cv.height = Math.round(r.height * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  stepViewAnim();
  const F = S.field;
  const o = toScreen({x:0, y:F.h});
  // A bigger field appears as a wave of purple squares from the old edge.
  const FA = fieldAnim, faT = FA ? (performance.now() - FA.t0) / FA.ms : 1;
  const growing = FA && faT < 1 && (F.w > FA.w0 || F.h > FA.h0);
  const revealCells = [];
  if (growing) {
    const maxD = Math.max(F.w - FA.w0, F.h - FA.h0, 1);
    for (let x = 0; x < F.w; x += 10) for (let y = 0; y < F.h; y += 10) {
      if (x + 10 <= FA.w0 && y + 10 <= FA.h0) continue;
      const d = Math.max(0, x - FA.w0, y - FA.h0);
      revealCells.push({x, y, p:clamp01((faT - 0.55 * d / maxD) / 0.45)});
    }
    ctx.save(); ctx.beginPath();
    const oo = toScreen({x:0, y:Math.min(FA.h0, F.h)});
    ctx.rect(oo.x, oo.y, Math.min(FA.w0, F.w) * view.scale, Math.min(FA.h0, F.h) * view.scale);
    for (const c of revealCells) if (c.p >= 1) { const q = toScreen({x:c.x, y:c.y + 10}); ctx.rect(q.x, q.y, 10 * view.scale, 10 * view.scale); }
    ctx.clip();
  } else if (FA && faT >= 1) fieldAnim = null;
  ctx.fillStyle = "#130b24"; ctx.fillRect(o.x, o.y, F.w * view.scale, F.h * view.scale);
  if (bgImage) {
    ctx.globalAlpha = S.bg.opacity;
    ctx.drawImage(bgImage, o.x, o.y, F.w * view.scale, F.h * view.scale);
    ctx.globalAlpha = 1;
  }
  // grid: 10 cm, stronger every 50 cm
  const minor = view.scale * 10 >= 6;
  for (let x = 0; x <= F.w + 1e-6; x += 10) {
    if (!minor && x % 50) continue;
    const a = toScreen({x, y:0}), b = toScreen({x, y:F.h});
    ctx.strokeStyle = x % 50 ? "#c4b5fd12" : "#c4b5fd2a"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    if (x % 50 === 0) label(String(x), {x, y:-6}, "#958ab3");
  }
  for (let y = 0; y <= F.h + 1e-6; y += 10) {
    if (!minor && y % 50) continue;
    const a = toScreen({x:0, y}), b = toScreen({x:F.w, y});
    ctx.strokeStyle = y % 50 ? "#c4b5fd12" : "#c4b5fd2a"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    if (y % 50 === 0) label(String(y), {x:-6, y}, "#958ab3", "right");
  }
  if (growing) {
    ctx.restore();
    for (const c of revealCells) {
      if (c.p <= 0 || c.p >= 1) continue;
      const k = 0.2 + 0.8 * Math.max(0, easeBack(c.p)), sz = 10 * view.scale * k;
      const q = toScreen({x:c.x + 5, y:c.y + 5});
      ctx.globalAlpha = c.p; ctx.fillStyle = "#130b24";
      ctx.fillRect(q.x - 5 * view.scale, q.y - 5 * view.scale, 10 * view.scale, 10 * view.scale);
      ctx.globalAlpha = 0.25 + 0.65 * (1 - c.p); ctx.fillStyle = "#a78bfa";
      ctx.beginPath(); ctx.roundRect ? ctx.roundRect(q.x - sz / 2, q.y - sz / 2, sz, sz, Math.min(4, sz / 4)) : ctx.rect(q.x - sz / 2, q.y - sz / 2, sz, sz);
      ctx.fill(); ctx.globalAlpha = 1;
    }
  }
  const bw = FA && faT < 1 ? FA.w0 + (F.w - FA.w0) * easeOut(clamp01(faT)) : F.w;
  const bh = FA && faT < 1 ? FA.h0 + (F.h - FA.h0) * easeOut(clamp01(faT)) : F.h;
  const ob0 = toScreen({x:0, y:bh});
  ctx.strokeStyle = "#c4b5fd80"; ctx.lineWidth = 1.5;
  ctx.strokeRect(ob0.x, ob0.y, bw * view.scale, bh * view.scale);

  for (const rc of S.rects) {
    const t = TYPES[rc.type];
    const pts = [{x:rc.x,y:rc.y},{x:rc.x+rc.w,y:rc.y},{x:rc.x+rc.w,y:rc.y+rc.h},{x:rc.x,y:rc.y+rc.h}];
    const sel = selected && selected.kind === "rect" && selected.id === rc.id;
    const k = pop(rc.id), cxr = rc.x + rc.w / 2, cyr = rc.y + rc.h / 2;
    const ptsK = k === 1 ? pts : pts.map(q => ({x:cxr + (q.x - cxr) * k, y:cyr + (q.y - cyr) * k}));
    ctx.globalAlpha = clamp01(k);
    poly(ptsK, t.fill + (rc.type === "zone" ? "55" : "cc"), sel ? "#fff" : t.fill, sel ? 2 : 1);
    if (rc.type === "sudden") { ctx.setLineDash([5, 4]); poly(ptsK, null, "#fecdd3", 1.5); ctx.setLineDash([]); }
    ctx.globalAlpha = 1;
    label((rc.label ? rc.label + " " : "") + fmt(rc.w) + "×" + fmt(rc.h), {x:rc.x + rc.w / 2, y:rc.y + rc.h / 2});
  }
  for (const ob of S.objects) {
    const s = toScreen(ob);
    const sel = selected && selected.kind === "object" && selected.id === ob.id;
    ctx.beginPath(); ctx.arc(s.x, s.y, Math.max(5, 2.5 * view.scale) * pop(ob.id), 0, 2 * Math.PI);
    ctx.fillStyle = ob.color || "#f472b6"; ctx.fill();
    ctx.strokeStyle = sel ? "#fff" : "#0b0714"; ctx.lineWidth = 2; ctx.stroke();
    label(ob.name || "объект", {x:ob.x, y:ob.y + 6 + 8 / view.scale});
  }

  drawSketch();
  // simulated path
  if (sim) {
    for (const m of sim.moves) {
      if (m.poly) poly(m.poly, "#a78bfa14", null);
      else {   // smooth move with turning: swept circle
        const a = toScreen(m.from), b = toScreen(m.to);
        ctx.strokeStyle = "#a78bfa14"; ctx.lineCap = "round"; ctx.lineWidth = 2 * m.capsule * view.scale;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.lineCap = "butt";
      }
    }
    for (const dt of sim.detours || []) {
      ctx.strokeStyle = "#fb923c"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.beginPath();
      dt.pts.forEach((q, k) => { const s2 = toScreen(q); k ? ctx.lineTo(s2.x, s2.y) : ctx.moveTo(s2.x, s2.y); });
      ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.strokeStyle = "#a78bfa"; ctx.lineWidth = 2;
    for (const m of sim.moves) {
      const a = toScreen(m.from), b = toScreen(m.to);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    for (const st of sim.stops) drawRobot(st.pose, "#a78bfa99", null, [4, 4]);
  }
  // waypoints
  S.route.forEach((wp, i) => {
    const s = toScreen(wp);
    const sel = selected && selected.kind === "route" && selected.id === wp.id;
    const kw = pop(wp.id);
    if (kw <= 0.01) return;
    ctx.beginPath(); ctx.arc(s.x, s.y, (sel ? 10 : 8) * kw, 0, 2 * Math.PI);
    ctx.fillStyle = wp.back ? "#f0abfc" : "#a78bfa"; ctx.fill();
    ctx.strokeStyle = sel ? "#fff" : "#0b0714"; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = "#120a22"; ctx.font = "bold 11px system-ui"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(String(i + 1), s.x, s.y);
    if (wp.actions.length) label(wp.actions.map(a => actionShort(a)).join(", "), {x:wp.x, y:wp.y - 14 / view.scale}, "#f0abfc");
  });
  if (sim) {
    const bad = new Set(sim.warnings.filter(w => w.level === "bad").map(w => w.wp));
    for (const m of sim.moves) if (bad.has(m.wp) && m.poly) poly(m.poly, "#fb718533", "#fb7185", 1);
  }
  // timeline: selected step / playback
  const tlPose = timelinePose();
  if (tlPose) {
    const stp = sim.steps[tlPose.index];
    if (stp && stp.before && stp.after) {
      const a = toScreen(stp.before), b = toScreen(stp.after);
      ctx.strokeStyle = "#fde68a"; ctx.lineWidth = 4;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    drawRobot(tlPose.pose, "#fde68a", "#fde68a40");
  }
  // start robot
  const st = {x:S.start.x, y:S.start.y, b:S.start.b};
  const selStart = selected && selected.kind === "start";
  drawRobot(st, selStart ? "#ffffff" : "#6ee7b7", "#6ee7b733");
  label("СТАРТ", {x:st.x, y:st.y});

  if (measure && measure.a) {
    const b = measure.b || cursorCm;
    if (b) {
      const a = toScreen(measure.a), c = toScreen(b);
      ctx.strokeStyle = "#fde68a"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(c.x, c.y); ctx.stroke(); ctx.setLineDash([]);
      label(fmt(Math.hypot(b.x - measure.a.x, b.y - measure.a.y)) + " см",
        {x:(measure.a.x + b.x) / 2, y:(measure.a.y + b.y) / 2}, "#fde68a");
    }
  }
  if (drag && drag.kind === "newRect" && drag.cur) {
    const r2 = rectFrom(drag.a, drag.cur);
    poly([{x:r2.x,y:r2.y},{x:r2.x+r2.w,y:r2.y},{x:r2.x+r2.w,y:r2.y+r2.h},{x:r2.x,y:r2.y+r2.h}],
      TYPES[drag.type].fill + "88", "#fff", 1);
    label(fmt(r2.w) + "×" + fmt(r2.h), {x:r2.x + r2.w / 2, y:r2.y + r2.h / 2});
  }
}
function actionShort(a) {
  const v = a.value;
  return {pickup:"взять", place:"положить", rotate:"↻" + v + "°", sharp:"Sharp " + v,
    left:"←" + v, right:"→" + v, leftAway:"← пока П≥" + v, rightAway:"→ пока Л≥" + v,
    back:"назад " + v, wait:v + " с", custom:"код"}[a.type] || a.type;
}

// ── editing ─────────────────────────────────────────────────────────────
function pushUndo() {
  undoStack.push(JSON.stringify(S));
  if (undoStack.length > 100) undoStack.shift();
}
function undo() {
  if (!undoStack.length) return;
  S = JSON.parse(undoStack.pop());
  selected = null;
  if (sketchMode) { skSel = []; skDraft = null; hudHide(); skAnalyze(); }
  loadBg();
  changed(false);
}
function snapV(v) { const s = Number(document.getElementById("snap").value); return Math.round(v / s) * s; }
function snapP(p) { return {x:snapV(p.x), y:snapV(p.y)}; }
function rectFrom(a, b) {
  return {x:Math.min(a.x, b.x), y:Math.min(a.y, b.y), w:Math.abs(b.x - a.x), h:Math.abs(b.y - a.y)};
}
function snapAngleFrom(from, p) {
  const step = Number(document.getElementById("angleSnap").value);
  if (!step || !from) return p;
  const d = Math.hypot(p.x - from.x, p.y - from.y);
  const b = Math.round(bearingTo(from, p) / step) * step;
  const f = dirOf(b);
  return {x:from.x + f.x * d, y:from.y + f.y * d};
}
function routeEnd() {
  if (sim && sim.final && S.route.length) return sim.final;
  return S.route.length ? S.route[S.route.length - 1] : S.start;
}
function hitTest(p) {
  const tol = 8 / view.scale;
  for (let i = S.route.length - 1; i >= 0; i--) {
    const w = S.route[i];
    if (Math.hypot(w.x - p.x, w.y - p.y) <= tol + 2) return {kind:"route", id:w.id};
  }
  for (let i = S.objects.length - 1; i >= 0; i--) {
    const o = S.objects[i];
    if (Math.hypot(o.x - p.x, o.y - p.y) <= tol + 3) return {kind:"object", id:o.id};
  }
  if (Math.hypot(S.start.x - p.x, S.start.y - p.y) <= Math.max(S.robot.w, S.robot.l) / 2) return {kind:"start"};
  for (let i = S.rects.length - 1; i >= 0; i--) {
    const r = S.rects[i];
    if (p.x >= r.x - tol && p.x <= r.x + r.w + tol && p.y >= r.y - tol && p.y <= r.y + r.h + tol)
      return {kind:"rect", id:r.id};
  }
  return null;
}
const find = (kind, id) => kind === "rect" ? S.rects.find(r => r.id === id)
  : kind === "object" ? S.objects.find(o => o.id === id)
  : kind === "route" ? S.route.find(w => w.id === id) : kind === "start" ? S.start : null;

cv.addEventListener("contextmenu", e => e.preventDefault());
cv.addEventListener("mousedown", e => {
  const p = toCm(e.offsetX, e.offsetY), sp = snapP(p);
  if (e.button === 1 || e.button === 2) { drag = {kind:"pan", sx:e.offsetX, sy:e.offsetY, ox:view.ox, oy:view.oy}; return; }
  if (sketchMode) { skMouseDown(e, p); return; }
  if (tool === "measure") {
    if (!measure || measure.b) measure = {a:sp, b:null}; else measure.b = sp;
    draw(); return;
  }
  if (TYPES[tool]) { drag = {kind:"newRect", type:tool, a:sp, cur:sp}; return; }
  if (tool === "object") {
    pushUndo();
    const ob = {id:uid(), x:sp.x, y:sp.y, name:"объект " + (S.objects.length + 1), color:"#f472b6"};
    born(ob.id);
    S.objects.push(ob); selected = {kind:"object", id:ob.id}; changed(); return;
  }
  if (tool === "start") {
    pushUndo(); S.start.x = sp.x; S.start.y = sp.y; selected = {kind:"start"};
    drag = {kind:"startDir"}; changed(); return;
  }
  if (tool === "route") {
    pushUndo();
    const from = routeEnd();
    const q = snapAngleFrom(from, sp);
    const wp = {id:uid(), x:r1(q.x), y:r1(q.y), back:false, barrier:false, sharp:false, actions:[]};
    born(wp.id);
    S.route.push(wp); selected = {kind:"route", id:wp.id}; changed(); return;
  }
  // select
  const hit = hitTest(p);
  selected = hit;
  if (hit) {
    const el = find(hit.kind, hit.id);
    pushUndo();
    if (hit.kind === "rect") {
      const tol = 10 / view.scale;
      const corners = [[el.x, el.y], [el.x + el.w, el.y], [el.x + el.w, el.y + el.h], [el.x, el.y + el.h]];
      const ci = corners.findIndex(([cx, cy]) => Math.hypot(cx - p.x, cy - p.y) <= tol);
      if (ci >= 0) {
        const opp = corners[(ci + 2) % 4];
        drag = {kind:"resize", el, fixed:{x:opp[0], y:opp[1]}};
      } else drag = {kind:"move", el, dx:p.x - el.x, dy:p.y - el.y};
    } else drag = {kind:"move", el, dx:p.x - el.x, dy:p.y - el.y};
  } else drag = {kind:"pan", sx:e.offsetX, sy:e.offsetY, ox:view.ox, oy:view.oy};
  changed(false);
});
cv.addEventListener("mousemove", e => {
  const p = toCm(e.offsetX, e.offsetY);
  cursorCm = p;
  document.getElementById("cursor").textContent = "x " + fmt(p.x) + "  y " + fmt(p.y) + " см";
  if (sketchMode && !(drag && drag.kind === "pan")) { skMouseMove(e, p); return; }
  if (!drag) { if (tool === "measure" && measure && !measure.b) draw(); return; }
  const sp = snapP(p);
  if (drag.kind === "pan") {
    viewTouched = true;
    view.ox = drag.ox + e.offsetX - drag.sx; view.oy = drag.oy + e.offsetY - drag.sy; draw();
  } else if (drag.kind === "newRect") { drag.cur = sp; draw(); }
  else if (drag.kind === "move") {
    drag.el.x = snapV(p.x - drag.dx); drag.el.y = snapV(p.y - drag.dy); drag.moved = true; changed(false, true);
  } else if (drag.kind === "resize") {
    Object.assign(drag.el, rectFrom(drag.fixed, sp)); drag.moved = true; changed(false, true);
  } else if (drag.kind === "startDir") {
    if (Math.hypot(p.x - S.start.x, p.y - S.start.y) > 3) {
      const step = Number(document.getElementById("angleSnap").value) || 1;
      S.start.b = norm360(Math.round(bearingTo(S.start, p) / step) * step);
      changed(false, true);
    }
  }
});
cv.addEventListener("dblclick", e => { if (sketchMode) skDblClick(e); });
window.addEventListener("mouseup", () => {
  if (sketchMode && !(drag && drag.kind === "pan")) { skMouseUp(); return; }
  if (drag && drag.kind === "newRect" && drag.cur) {
    const r = rectFrom(drag.a, drag.cur);
    if (r.w >= 1 && r.h >= 1) {
      pushUndo();
      const rc = {id:uid(), type:drag.type, label:"", ...r};
      born(rc.id);
      S.rects.push(rc); selected = {kind:"rect", id:rc.id};
    }
    drag = null; changed(); return;
  }
  if (drag && (drag.moved || drag.kind === "startDir")) { drag = null; changed(); return; }
  if (drag && drag.kind === "move" && !drag.moved) undoStack.pop();
  drag = null;
});
cv.addEventListener("wheel", e => {
  e.preventDefault();
  const k = Math.exp(-e.deltaY * 0.0015);
  const p = toCm(e.offsetX, e.offsetY);
  viewTouched = true;
  const base = viewAnim ? viewAnim.to : view;
  const scale = Math.max(0.2, Math.min(40, base.scale * k));
  const pc = {x:(e.offsetX - base.ox) / base.scale, y:(base.oy - e.offsetY) / base.scale};
  void p;
  tweenView({scale, ox:e.offsetX - pc.x * scale, oy:e.offsetY + pc.y * scale}, 140);
}, {passive:false});
window.addEventListener("keydown", e => {
  if (sketchMode && !(e.target.tagName === "INPUT" && !e.target.closest("#hud")) && e.target.tagName !== "TEXTAREA"
      && e.target.tagName !== "SELECT" && skKeyDown(e)) { e.stopImmediatePropagation(); return; }
  if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA" || e.target.tagName === "SELECT") return;
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); undo(); }
  else if ((e.key === "Delete" || e.key === "Backspace") && selected) { e.preventDefault(); deleteSelected(); }
  else if (e.key === "Escape") { selected = null; measure = null; tlSel = null; tlT = 0; changed(false); }
});
function deleteSelected() {
  if (!selected || selected.kind === "start") return;
  pushUndo();
  const list = {rect:"rects", object:"objects", route:"route"}[selected.kind];
  S[list] = S[list].filter(x => x.id !== selected.id);
  selected = null; changed();
}

// ── panels ──────────────────────────────────────────────────────────────
const $ = id => document.getElementById(id);
function numInput(value, onChange, step) {
  const i = document.createElement("input");
  i.type = "number"; i.step = step || "1"; i.value = value;
  i.addEventListener("change", () => { const v = Number(i.value); if (Number.isFinite(v)) { pushUndo(); onChange(v); changed(); } });
  return i;
}
function textInput(value, onChange) {
  const i = document.createElement("input");
  i.type = "text"; i.value = value || "";
  i.addEventListener("change", () => { pushUndo(); onChange(i.value); changed(); });
  return i;
}
function row(...items) {
  const d = document.createElement("div"); d.className = "row";
  for (const it of items) {
    if (typeof it === "string") { const l = document.createElement("label"); l.textContent = it; d.appendChild(l); }
    else d.appendChild(it);
  }
  return d;
}
function checkbox(text, value, onChange) {
  const l = document.createElement("label");
  const c = document.createElement("input"); c.type = "checkbox"; c.checked = !!value;
  c.addEventListener("change", () => { pushUndo(); onChange(c.checked); changed(); });
  l.appendChild(c); l.append(" " + text); return l;
}
function button(text, onClick, cls) {
  const b = document.createElement("button"); b.textContent = text; if (cls) b.className = cls;
  b.addEventListener("click", onClick); return b;
}
function renderSelPanel() {
  if (sketchMode) { renderSketchPanel(); return; }
  const panel = $("selPanel");
  panel.innerHTML = "<h2>Выбрано</h2>";
  const el = selected && find(selected.kind, selected.id);
  if (!el) { panel.insertAdjacentHTML("beforeend", '<div class="muted">Ничего. Кликните по элементу.</div>'); return; }
  if (selected.kind === "rect") {
    const type = document.createElement("select");
    for (const [k, t] of Object.entries(TYPES)) type.add(new Option(t.name, k, false, k === el.type));
    type.addEventListener("change", () => { pushUndo(); el.type = type.value; changed(); });
    panel.append(row(type, textInput(el.label, v => el.label = v)),
      row("x", numInput(el.x, v => el.x = v, "0.5"), "y", numInput(el.y, v => el.y = v, "0.5")),
      row("ширина", numInput(el.w, v => el.w = Math.max(0.5, v), "0.5"), "высота", numInput(el.h, v => el.h = Math.max(0.5, v), "0.5")),
      row(button("Удалить", deleteSelected, "bad")));
    panel.insertAdjacentHTML("beforeend", '<div class="muted">x, y — левый нижний угол, от левого нижнего угла поля.</div>');
  } else if (selected.kind === "object") {
    const color = document.createElement("input"); color.type = "color"; color.value = el.color || "#f472b6";
    color.addEventListener("change", () => { pushUndo(); el.color = color.value; changed(); });
    panel.append(row(textInput(el.name, v => el.name = v), color),
      row("x", numInput(el.x, v => el.x = v, "0.5"), "y", numInput(el.y, v => el.y = v, "0.5")),
      row(button("Удалить", deleteSelected, "bad")));
  } else if (selected.kind === "start") {
    panel.insertAdjacentHTML("beforeend", '<div class="muted">Старт робота — правьте в разделе «Робот».</div>');
  } else if (selected.kind === "route") {
    const n = S.route.indexOf(el) + 1;
    panel.querySelector("h2").textContent = "Точка маршрута " + n;
    panel.append(row("x", numInput(el.x, v => el.x = v, "0.5"), "y", numInput(el.y, v => el.y = v, "0.5")),
      row(checkbox("ехать назад", el.back, v => el.back = v),
          checkbox("barrier (объезд)", el.barrier, v => el.barrier = v),
          checkbox("стоп по Sharp", el.sharp, v => el.sharp = v),
          checkbox("выровняться по датчикам", el.align, v => el.align = v)));
    const stop = sim && sim.stops.find(s => s.wp === n);
    if (stop) panel.insertAdjacentHTML("beforeend", '<div class="muted">После точки (расчёт по карте): соник Л '
      + fmt(stop.sonicL) + ' см, соник П ' + fmt(stop.sonicR) + ' см, Sharp ' + fmt(stop.sharp)
      + ' см, направление ' + fmt(norm360(stop.pose.b - S.start.b)) + '° от старта</div>');
    panel.insertAdjacentHTML("beforeend", "<h2 style='margin-top:8px'>Действия в точке</h2>");
    el.actions.forEach((a, i) => {
      const d = document.createElement("div"); d.className = "act";
      const sel = document.createElement("select");
      for (const [k, t] of Object.entries(ACTIONS)) sel.add(new Option(t.name, k, false, k === a.type));
      sel.addEventListener("change", () => {
        pushUndo(); a.type = sel.value; const def = ACTIONS[a.type];
        a.value = def.value && def.value !== "method" ? def.def : null;
        if (def.value === "method") initMethodAction(a);
        changed();
      });
      d.appendChild(sel);
      const def = ACTIONS[a.type];
      if (def.value === "method") { d.style.flexWrap = "wrap"; for (const x of methodEditor(a, () => changed())) d.appendChild(x); }
      else if (def.value === "text") d.appendChild(textInput(a.value, v => a.value = v));
      else if (def.value) { const ni = numInput(a.value, v => a.value = v, "0.5"); ni.style.width = "64px"; d.appendChild(ni); }
      d.append(button("↑", () => { if (i > 0) { pushUndo(); [el.actions[i - 1], el.actions[i]] = [el.actions[i], el.actions[i - 1]]; changed(); } }),
        button("✕", () => { pushUndo(); el.actions.splice(i, 1); changed(); }));
      panel.appendChild(d);
    });
    panel.append(row(button("+ действие", () => { pushUndo(); el.actions.push({type:"pickup", value:null}); changed(); }),
      button("Вставить точку перед", () => {
        pushUndo(); const i = S.route.indexOf(el); const prev = i > 0 ? S.route[i - 1] : S.start;
        const wp = {id:uid(), x:r1((prev.x + el.x) / 2), y:r1((prev.y + el.y) / 2), back:false, barrier:false, sharp:false, actions:[]};
        S.route.splice(i, 0, wp); selected = {kind:"route", id:wp.id}; changed();
      }),
      button("Удалить", deleteSelected, "bad")));
  }
}
function renderWarnings() {
  const box = $("warnings"); box.innerHTML = "";
  const list = sim.warnings.length ? sim.warnings
    : [{level:"ok", wp:null, text:S.route.length ? "Столкновений нет" : "Маршрута пока нет"}];
  for (const w of list) {
    const d = document.createElement("div"); d.className = "w-" + w.level;
    d.textContent = (w.wp ? "Точка " + w.wp + ": " : "") + w.text;
    if (w.wp) d.addEventListener("click", () => { selected = {kind:"route", id:S.route[w.wp - 1].id}; changed(false); });
    box.appendChild(d);
  }
}
function syncInputs() {
  const set = (id, v) => { const el = $(id); if (document.activeElement !== el) el.value = v; };
  set("fieldW", S.field.w); set("fieldH", S.field.h);
  set("rW", S.robot.w); set("rL", S.robot.l); set("rSonicSide", S.robot.sonicSide);
  set("rSonicAlong", S.robot.sonicAlong); set("rSharp", S.robot.sharpFront);
  set("sX", S.start.x); set("sY", S.start.y); set("sB", S.start.b);
  set("firstCase", S.firstCase); set("bgOpacity", S.bg.opacity);
  const A = robotParams(), O = Object.assign(defaultOpts(), S.opts || {});
  set("rSharpSide", A.sharpSide); set("aObs", A.obstacle); set("aClear", A.clear); set("aExtra", A.extra); set("aFwd", A.forward);
  set("apCloseCm", O.closeCm); set("apFarCm", O.farCm);
  $("driveNormal").classList.toggle("on", O.drive !== "smooth"); $("driveSmooth").classList.toggle("on", O.drive === "smooth");
  $("apClose").classList.toggle("on", O.approach !== "far"); $("apFar").classList.toggle("on", O.approach === "far");
  $("optCorrect").checked = !!O.correct; $("optBarrier").checked = !!O.barrierAll;
  $("optZone").checked = !!O.toZone; $("optReturn").checked = !!O.returnStart;
  $("driveHint").textContent = O.drive === "smooth"
    ? "smoothTo/smoothVia: робот едет к точке по прямой и поворачивается на ходу, углы без остановки. Координаты — от старта (x вправо, y вперёд)."
    : "Поворот на месте + прямой отрезок, как раньше.";
  if (typeof renderMethods === "function") renderMethods();
}
function changed(record, light) {
  sim = simulate();
  if (tlSel !== null && tlSel >= sim.steps.length) tlSel = sim.steps.length ? sim.steps.length - 1 : null;
  if (tlT > sim.steps.length) tlT = sim.steps.length;
  if (!light) { renderSelPanel(); syncInputs(); }
  if (typeof renderTimeline === "function" && !tlPlaying) renderTimeline(false);
  if (typeof rebuild3d === "function" && is3d) rebuild3d();
  renderWarnings();
  $("code").value = buildCode(sim);
  draw();
  try { localStorage.setItem("fieldPlannerAutosave", JSON.stringify(S)); } catch (e) {}
}
function bindNumber(id, apply) {
  $(id).addEventListener("change", () => {
    const v = Number($(id).value);
    if (!Number.isFinite(v)) return;
    pushUndo(); apply(v); changed();
  });
}
function startFieldAnim() {
  const running = fieldAnim && performance.now() - fieldAnim.t0 < fieldAnim.ms;
  fieldAnim = {w0:running ? Math.min(fieldAnim.w0, S.field.w) : S.field.w,
    h0:running ? Math.min(fieldAnim.h0, S.field.h) : S.field.h, t0:performance.now(), ms:1100};
  setTimeout(() => { if (!viewTouched) fitView(true); kick(1200); }, 0);
}
bindNumber("fieldW", v => { startFieldAnim(); S.field.w = Math.max(20, v); });
bindNumber("fieldH", v => { startFieldAnim(); S.field.h = Math.max(20, v); });
bindNumber("rW", v => S.robot.w = Math.max(5, v));
bindNumber("rL", v => S.robot.l = Math.max(5, v));
bindNumber("rSonicSide", v => S.robot.sonicSide = v);
bindNumber("rSonicAlong", v => S.robot.sonicAlong = v);
bindNumber("rSharp", v => S.robot.sharpFront = v);
bindNumber("rSharpSide", v => S.robot.sharpSide = Math.max(0, v));
bindNumber("aObs", v => S.robot.avoidObstacle = Math.max(8, v));
bindNumber("aClear", v => S.robot.avoidClear = Math.max(10, v));
bindNumber("aExtra", v => S.robot.avoidExtra = Math.max(0, v));
bindNumber("aFwd", v => S.robot.avoidForward = Math.max(5, v));
const opts = () => { S.opts = Object.assign(defaultOpts(), S.opts || {}); return S.opts; };
const setOpt = (k, v) => { pushUndo(); opts()[k] = v; changed(); };
$("driveNormal").addEventListener("click", () => setOpt("drive", "normal"));
$("driveSmooth").addEventListener("click", () => setOpt("drive", "smooth"));
$("apClose").addEventListener("click", () => setOpt("approach", "close"));
$("apFar").addEventListener("click", () => setOpt("approach", "far"));
for (const [id, key] of [["optCorrect", "correct"], ["optBarrier", "barrierAll"], ["optZone", "toZone"], ["optReturn", "returnStart"]])
  $(id).addEventListener("change", () => setOpt(key, $(id).checked));
bindNumber("apCloseCm", v => opts().closeCm = Math.max(0, v));
bindNumber("apFarCm", v => opts().farCm = Math.max(0, v));
$("buildRoute").addEventListener("click", () => {
  if (!S.objects.length) { status("Нет объектов: поставьте их инструментом «Объект»", true); return; }
  if (S.route.length && !confirm("Заменить текущий маршрут автоматическим? (Ctrl+Z вернёт)")) return;
  const res = autoRoute();
  pushUndo();
  S.route = res.route; selected = null; tlSel = null; tlT = 0;
  changed();
  const extra = (res.guarded ? ", barrier() у " + res.guarded + " отрезков у внезапных" : "")
    + (res.walls ? ", внезапных обойдено как стена (нет места для объезда или Sharp их не увидят): " + res.walls : "");
  status(res.missed.length ? "Не получилось подъехать: " + res.missed.join(", ") + extra : "Маршрут построен: точек " + res.route.length + extra, !!res.missed.length);
  const now0 = performance.now();
  S.route.forEach((w, k) => bornAt.set(w.id, now0 + k * 45));
  kick(S.route.length * 45 + 400);
});
function initMethodAction(a) {
  const list = enabledMethods();
  a.method = list.length ? list[0].name : "";
  a.args = list.length ? list[0].params.map(p => p.type === "boolean" ? false : p.type === "String" ? "" : 0) : [];
}
const enabledMethods = () => ((S.methods && S.methods.list) || []).filter(m => !(S.methods.hidden || []).includes(m.name));
/** Select of the project's methods + one input per parameter. */
function methodEditor(a, onChange) {
  const list = enabledMethods(), els = [];
  const sel = document.createElement("select");
  sel.style.maxWidth = "160px";
  if (!list.length) sel.add(new Option("— найдите методы проекта —", ""));
  for (const m of list) sel.add(new Option(m.name + "(" + m.params.map(p => p.name).join(", ") + ")", m.name, false, m.name === a.method));
  if (a.method && !list.some(m => m.name === a.method)) sel.add(new Option(a.method + " (нет в списке)", a.method, false, true));
  sel.addEventListener("change", () => {
    pushUndo(); a.method = sel.value;
    const m = list.find(x => x.name === a.method);
    a.args = m ? m.params.map(p => p.type === "boolean" ? false : p.type === "String" ? "" : 0) : [];
    onChange();
  });
  els.push(sel);
  const m = list.find(x => x.name === a.method);
  (m ? m.params : []).forEach((p, k) => {
    const i = document.createElement("input");
    if (p.type === "boolean") { i.type = "checkbox"; i.checked = !!(a.args || [])[k]; }
    else { i.type = p.type === "String" ? "text" : "number"; i.step = "0.5"; i.value = (a.args || [])[k] ?? ""; i.style.width = "64px"; }
    i.title = p.type + " " + p.name;
    i.placeholder = p.name;
    i.addEventListener("change", () => {
      pushUndo(); a.args = a.args || [];
      a.args[k] = p.type === "boolean" ? i.checked : p.type === "String" ? i.value : Number(i.value) || 0;
      onChange();
    });
    els.push(i);
  });
  if (m && m.doc) { const t = document.createElement("span"); t.className = "muted"; t.textContent = m.doc; t.style.flexBasis = "100%"; els.push(t); }
  return els;
}
function renderMethods() {
  const box = $("methodList");
  const M = S.methods || {list:[]};
  if (document.activeElement !== $("projPath")) $("projPath").value = M.path || projectDirDefault;
  if (!M.list || !M.list.length) { box.innerHTML = M.junk ? "Шагов маршрута не найдено." : "Укажите папку проекта и нажмите «Найти»."; return; }
  box.innerHTML = "";
  const head = document.createElement("div"); head.className = "muted";
  head.textContent = "Шаги маршрута: " + M.list.length + " (галочка — показывать в «метод проекта…»)";
  box.appendChild(head);
  for (const m of M.list) {
    const l = document.createElement("label"); l.style.display = "block";
    const c = document.createElement("input"); c.type = "checkbox"; c.checked = !(M.hidden || []).includes(m.name);
    c.addEventListener("change", () => {
      pushUndo(); M.hidden = M.hidden || [];
      M.hidden = c.checked ? M.hidden.filter(x => x !== m.name) : M.hidden.concat([m.name]);
      changed();
    });
    l.append(c, " " + m.name + "(" + m.params.map(p => p.type + " " + p.name).join(", ") + ")" + (m.used ? " · в маршруте" : ""));
    l.title = m.doc || "";
    box.appendChild(l);
  }
  if (M.modifiers && M.modifiers.length) {
    const d = document.createElement("div"); d.className = "muted";
    d.textContent = "Модификаторы: " + M.modifiers.join(", ") + " — barrier() ставится галочкой у отрезка.";
    box.appendChild(d);
  }
  if (M.junk && M.junk.length) {
    const det = document.createElement("details");
    det.innerHTML = "<summary class='muted'>Скрыто как мусор: " + M.junk.length + "</summary>";
    for (const j of M.junk) { const r = document.createElement("div"); r.className = "muted"; r.textContent = j.name + " — " + j.reason; det.appendChild(r); }
    box.appendChild(det);
  }
}
let projectDirDefault = "";
$("scanMethods").addEventListener("click", async () => {
  try {
    const path = $("projPath").value.trim() || projectDirDefault;
    const data = await api("/scan", {path});
    pushUndo();
    const hidden = (S.methods && S.methods.hidden) || [];
    S.methods = {path, list:data.methods, junk:data.junk, modifiers:data.modifiers, hidden};
    changed();
    status("Найдено шагов: " + data.methods.length + " (" + data.files.join(", ") + ")");
  } catch (e) { status(e.message, true); }
});
bindNumber("sX", v => S.start.x = v);
bindNumber("sY", v => S.start.y = v);
bindNumber("sB", v => S.start.b = norm360(v));
bindNumber("firstCase", v => S.firstCase = Math.max(0, Math.round(v)));
$("bgOpacity").addEventListener("input", () => { S.bg.opacity = Number($("bgOpacity").value); draw(); });
$("bgFile").addEventListener("change", () => {
  const f = $("bgFile").files[0]; if (!f) return;
  const reader = new FileReader();
  reader.onload = () => { pushUndo(); S.bg.data = reader.result; loadBg(); changed(); };
  reader.readAsDataURL(f);
});
$("bgClear").addEventListener("click", () => { pushUndo(); S.bg.data = null; bgImage = null; changed(); });
function loadBg() {
  bgImage = null;
  if (!S.bg.data) return;
  const img = new Image(); img.onload = () => { bgImage = img; draw(); }; img.src = S.bg.data;
}
$("undo").addEventListener("click", undo);
$("fit").addEventListener("click", () => { viewTouched = false; fitView(true); });
$("copy").addEventListener("click", async () => {
  try { await navigator.clipboard.writeText($("code").value); status("Код скопирован"); }
  catch (e) { $("code").select(); document.execCommand("copy"); status("Код скопирован"); }
});

// ── sketch: Fusion-360-like constrained 2D sketch ──────────────────────
// Geometry: points (x, y), lines (p1, p2), circles (c, r), arcs (c, p1, p2,
// counter-clockwise from p1 to p2). Constraints and driving dimensions are
// solved together by a Levenberg-Marquardt least-squares solver. Sketch
// curves that are not construction geometry are walls for the robot.
let sketchMode = false;
let skTool = "select";
let skSel = [];              // keys "p:id" "l:id" "c:id" "a:id" "k:id"
let skHover = null;
let skSnapNow = null;        // {x, y, kind, ref, guides}
let skDraft = null;
let skDrag = null;
let skFree = new Set();      // "x:id" "y:id" "r:id" still free after solving
let skDof = 0;
let skBoxes = [];            // screen hit boxes for dimension labels and glyphs
let skConstruction = false;  // new geometry is construction geometry
let skWallsCache = [];
let skDraftTrim = null;
const SK_K_ANG = 100;        // weight of angle residuals (radians -> cm-ish)

const SK_TOOLS = [["select","Выбор",""],["line","Линия","L"],["rect","Прямоуг.","R"],
  ["crect","Прямоуг. от центра",""],["circle","Окружность","C"],["arc","Дуга 3 т.","A"],
  ["polygon","Многоуг.",""],["point","Точка","P"],["dim","Размер","D"],["trim","Обрезка","T"],
  ["fillet","Скругление","F"],["offset","Смещение","O"],["mirror","Зеркало","M"]];
const SK_CONS = [["horizontal","━","Горизонтально"],["vertical","┃","Вертикально"],
  ["coincident","⦿","Совпадение / точка на кривой"],["parallel","∥","Параллельно"],
  ["perpendicular","⊥","Перпендикулярно"],["equal","=","Равенство"],["tangent","◠","Касание"],
  ["midpoint","△","Середина"],["concentric","◎","Концентричность"],["collinear","≡","Коллинеарность"],
  ["symmetric","⇋","Симметрия (2 точки + линия)"],["fix","🔒","Фиксация"]];
const SK_GLYPH = {horizontal:"━", vertical:"┃", parallel:"∥", perpendicular:"⊥", equal:"=",
  tangent:"◠", tangentCC:"◠", tangentEnd:"◠", midpoint:"△", concentric:"◎", collinear:"≡", symmetric:"⇋",
  coincident:"⦿", onLine:"⦿", onCircle:"⦿"};
const SK_NAMES = {horizontal:"горизонтально", vertical:"вертикально", parallel:"параллельно",
  perpendicular:"перпендикулярно", equal:"равенство", tangent:"касание", tangentCC:"касание", tangentEnd:"касание",
  midpoint:"середина", concentric:"концентричность", collinear:"коллинеарность",
  symmetric:"симметрия", coincident:"совпадение", onLine:"точка на линии", onCircle:"точка на окружности",
  dist:"расстояние", hdist:"гориз. расстояние", vdist:"верт. расстояние", pointLine:"точка–линия",
  radius:"радиус", diameter:"диаметр", angle:"угол", angleH:"угол к горизонту"};
const SK_HINTS = {
  select:"Клик — выбрать (Shift — добавить), рамка — выбрать несколько. Тяните точку/линию — решатель держит зависимости. Двойной клик по размеру — изменить.",
  line:"Клик — точки цепочки. Наберите число — длина, Tab — угол, Enter. Esc / двойной клик — закончить. Клик в начало — замкнуть.",
  rect:"Два угла. Наберите ширину, Tab — высоту.", crect:"Центр, затем угол. Ширина/высота — с клавиатуры.",
  circle:"Центр, затем радиус. Наберите диаметр.", arc:"Начало, конец, затем точка на дуге.",
  polygon:"Центр, затем вершина. Число сторон — с клавиатуры.", point:"Клик — точка.",
  dim:"Выберите линию / две точки / окружность / две линии, затем кликните, где поставить размер, и введите значение.",
  trim:"Клик по куску кривой между пересечениями — он удалится.",
  fillet:"Клик рядом с углом двух линий, введите радиус, Enter.",
  offset:"Клик по линии или окружности, введите расстояние (сторона — по курсору), Enter.",
  mirror:"Выделите элементы, включите Зеркало и кликните ось симметрии (линию).",
};

function skData() {
  if (!S.sketch) S.sketch = {points:[], lines:[], circles:[], arcs:[], cons:[]};
  for (const k of ["points","lines","circles","arcs","cons"]) if (!S.sketch[k]) S.sketch[k] = [];
  return S.sketch;
}
const skNewId = p => p + Math.random().toString(36).slice(2, 9);
function skVirtualPoints() {
  const W = S.field.w, H = S.field.h;
  return [{id:"O", x:0, y:0, fixed:true, virtual:true}, {id:"OX", x:W, y:0, fixed:true, virtual:true},
    {id:"OY", x:0, y:H, fixed:true, virtual:true}, {id:"OXY", x:W, y:H, fixed:true, virtual:true}];
}
function skVirtualLines() {
  return [{id:"EB", p1:"O", p2:"OX", construction:true, virtual:true},
    {id:"ER", p1:"OX", p2:"OXY", construction:true, virtual:true},
    {id:"ET", p1:"OY", p2:"OXY", construction:true, virtual:true},
    {id:"EL", p1:"O", p2:"OY", construction:true, virtual:true}];
}
function skIndex() {
  const d = skData();
  const P = new Map(), L = new Map(), C = new Map();
  for (const p of skVirtualPoints()) P.set(p.id, p);
  for (const p of d.points) P.set(p.id, p);
  for (const l of skVirtualLines()) L.set(l.id, l);
  for (const l of d.lines) L.set(l.id, l);
  for (const c of d.circles) C.set(c.id, c);
  for (const a of d.arcs) C.set(a.id, a);
  return {P, L, C};
}
const skLP = (I, id) => { const l = I.L.get(id); return [I.P.get(l.p1), I.P.get(l.p2)]; };
function skCirc(I, id) {
  const c = I.C.get(id), ctr = I.P.get(c.c);
  if (c.p1) { const s = I.P.get(c.p1); return {c:ctr, r:Math.hypot(s.x - ctr.x, s.y - ctr.y), obj:c, arc:true}; }
  return {c:ctr, r:c.r, obj:c, arc:false};
}
function skSdist(a, b, p) {
  const dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy) || 1e-9;
  return ((p.x - a.x) * dy - (p.y - a.y) * dx) / L;
}
function skUnit(a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy) || 1e-9;
  return {x:dx / L, y:dy / L};
}
const skWrap = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };
const sk2pi = a => ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
function skArcAngles(I, a) {
  const c = I.P.get(a.c), p = I.P.get(a.p1), q = I.P.get(a.p2);
  const a1 = Math.atan2(p.y - c.y, p.x - c.x), a2 = Math.atan2(q.y - c.y, q.x - c.x);
  return {a1, sweep:sk2pi(a2 - a1) || 2 * Math.PI};
}
function skAngleOnArc(I, a, ang) {
  const {a1, sweep} = skArcAngles(I, a);
  return sk2pi(ang - a1) <= sweep + 1e-9;
}
const skIsLine = (I, id) => I.L.has(id);

// ── residuals ──
function skConRes(I, k, out) {
  const P = id => I.P.get(id);
  switch (k.type) {
    case "coincident": { const a = P(k.a), b = P(k.b); out.push(a.x - b.x, a.y - b.y); break; }
    case "horizontal": { const [a, b] = k.l ? skLP(I, k.l) : [P(k.a), P(k.b)]; out.push(a.y - b.y); break; }
    case "vertical": { const [a, b] = k.l ? skLP(I, k.l) : [P(k.a), P(k.b)]; out.push(a.x - b.x); break; }
    case "parallel": case "perpendicular": {
      const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2);
      const u = skUnit(a, b), v = skUnit(c, d);
      out.push((k.type === "parallel" ? u.x * v.y - u.y * v.x : u.x * v.x + u.y * v.y) * SK_K_ANG);
      break;
    }
    case "collinear": { const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2); out.push(skSdist(a, b, c), skSdist(a, b, d)); break; }
    case "equal": {
      if (skIsLine(I, k.e1)) {
        const [a, b] = skLP(I, k.e1), [c, d] = skLP(I, k.e2);
        out.push(Math.hypot(b.x - a.x, b.y - a.y) - Math.hypot(d.x - c.x, d.y - c.y));
      } else out.push(skCirc(I, k.e1).r - skCirc(I, k.e2).r);
      break;
    }
    case "midpoint": { const p = P(k.p), [a, b] = skLP(I, k.l); out.push(p.x - (a.x + b.x) / 2, p.y - (a.y + b.y) / 2); break; }
    case "onLine": { const [a, b] = skLP(I, k.l); out.push(skSdist(a, b, P(k.p))); break; }
    case "onCircle": { const c = skCirc(I, k.c), p = P(k.p); out.push(Math.hypot(p.x - c.c.x, p.y - c.c.y) - c.r); break; }
    case "tangent": { const [a, b] = skLP(I, k.l), c = skCirc(I, k.c); out.push(k.s * skSdist(a, b, c.c) - c.r); break; }
    case "tangentEnd": {
      // arc end shared with the line: the radius to that end is
      // perpendicular to the line (well-conditioned, unlike distance = r)
      const [a, b] = skLP(I, k.l), c = skCirc(I, k.c).c, p = P(k.p), u = skUnit(a, b);
      out.push((p.x - c.x) * u.x + (p.y - c.y) * u.y);
      break;
    }
    case "tangentCC": {
      const c1 = skCirc(I, k.c1), c2 = skCirc(I, k.c2), d = Math.hypot(c2.c.x - c1.c.x, c2.c.y - c1.c.y);
      out.push(k.mode === "in" ? d - k.s * (c1.r - c2.r) : d - (c1.r + c2.r));
      break;
    }
    case "concentric": { const a = skCirc(I, k.c1).c, b = skCirc(I, k.c2).c; out.push(a.x - b.x, a.y - b.y); break; }
    case "symmetric": {
      const a = P(k.a), b = P(k.b), [c, d] = skLP(I, k.l), u = skUnit(c, d);
      out.push(skSdist(c, d, {x:(a.x + b.x) / 2, y:(a.y + b.y) / 2}), (b.x - a.x) * u.x + (b.y - a.y) * u.y);
      break;
    }
    case "dist": { const a = P(k.a), b = P(k.b); out.push(Math.hypot(b.x - a.x, b.y - a.y) - k.v); break; }
    case "hdist": { const a = P(k.a), b = P(k.b); out.push(k.s * (b.x - a.x) - k.v); break; }
    case "vdist": { const a = P(k.a), b = P(k.b); out.push(k.s * (b.y - a.y) - k.v); break; }
    case "pointLine": { const [a, b] = skLP(I, k.l); out.push(k.s * skSdist(a, b, P(k.p)) - k.v); break; }
    case "radius": out.push(skCirc(I, k.c).r - k.v); break;
    case "diameter": out.push(2 * skCirc(I, k.c).r - k.v); break;
    case "angle": {
      const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2);
      const u = {x:(b.x - a.x) * k.s1, y:(b.y - a.y) * k.s1}, v = {x:(d.x - c.x) * k.s2, y:(d.y - c.y) * k.s2};
      const ang = Math.abs(Math.atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y));
      out.push((ang - rad(k.v)) * SK_K_ANG);
      break;
    }
    case "angleH": { const [a, b] = skLP(I, k.l); out.push(skWrap(Math.atan2(b.y - a.y, b.x - a.x) - rad(k.v)) * SK_K_ANG); break; }
  }
}
function skMeasure(I, k) {
  const P = id => I.P.get(id);
  switch (k.type) {
    case "dist": { const a = P(k.a), b = P(k.b); return Math.hypot(b.x - a.x, b.y - a.y); }
    case "hdist": return Math.abs(P(k.b).x - P(k.a).x);
    case "vdist": return Math.abs(P(k.b).y - P(k.a).y);
    case "pointLine": { const [a, b] = skLP(I, k.l); return Math.abs(skSdist(a, b, P(k.p))); }
    case "radius": return skCirc(I, k.c).r;
    case "diameter": return 2 * skCirc(I, k.c).r;
    case "angle": {
      const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2);
      const u = {x:(b.x - a.x) * k.s1, y:(b.y - a.y) * k.s1}, v = {x:(d.x - c.x) * k.s2, y:(d.y - c.y) * k.s2};
      return Math.abs(Math.atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y)) * 180 / Math.PI;
    }
    case "angleH": { const [a, b] = skLP(I, k.l); return Math.atan2(b.y - a.y, b.x - a.x) * 180 / Math.PI; }
  }
  return 0;
}
function skVars() {
  const d = skData(), vars = [];
  for (const p of d.points) if (!p.fixed) vars.push([p, "x"], [p, "y"]);
  for (const c of d.circles) if (!c.fixedR) vars.push([c, "r"]);
  return vars;
}
function skResAll(I, soft) {
  const d = skData(), out = [];
  for (const k of d.cons) if (!(k.dim && k.driven)) skConRes(I, k, out);
  for (const a of d.arcs) {
    const c = I.P.get(a.c), p = I.P.get(a.p1), q = I.P.get(a.p2);
    out.push(Math.hypot(q.x - c.x, q.y - c.y) - Math.hypot(p.x - c.x, p.y - c.y));
  }
  const hard = out.length;
  if (soft) for (const s of soft) { const p = I.P.get(s.id); if (p) out.push(s.w * (p.x - s.x), s.w * (p.y - s.y)); }
  return {r:out, hard};
}
function skGauss(M, b) {
  const n = b.length;
  for (let i = 0; i < n; i++) {
    let piv = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(M[r][i]) > Math.abs(M[piv][i])) piv = r;
    if (Math.abs(M[piv][i]) < 1e-14) return null;
    [M[i], M[piv]] = [M[piv], M[i]]; [b[i], b[piv]] = [b[piv], b[i]];
    for (let r = i + 1; r < n; r++) {
      const f = M[r][i] / M[i][i];
      if (f === 0) continue;
      for (let c = i; c < n; c++) M[r][c] -= f * M[i][c];
      b[r] -= f * b[i];
    }
  }
  const x = new Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i];
    for (let c = i + 1; c < n; c++) s -= M[i][c] * x[c];
    x[i] = s / M[i][i];
  }
  return x;
}
function skJacobian(I, vars, soft, r) {
  const n = vars.length, m = r.length, J = new Array(n);
  for (let j = 0; j < n; j++) {
    const [o, key] = vars[j], v = o[key], h = 1e-6 * Math.max(1, Math.abs(v));
    o[key] = v + h;
    const r2 = skResAll(I, soft).r;
    o[key] = v;
    const col = new Float64Array(m);
    for (let i = 0; i < m; i++) col[i] = (r2[i] - r[i]) / h;
    J[j] = col;
  }
  return J;
}
/** Solves the sketch in place; returns the remaining error of the hard
 *  constraints (0 = all satisfied). soft = [{id, x, y, w}] drag targets. */
function skSolve(soft, iters) {
  const I = skIndex(), vars = skVars(), n = vars.length;
  let {r, hard} = skResAll(I, soft);
  const sq = rr => { let s = 0; for (const v of rr) s += v * v; return s; };
  let f = sq(r), lambda = 1e-3;
  for (let it = 0; it < iters && n && f > 1e-18; it++) {
    const m = r.length, J = skJacobian(I, vars, soft, r);
    const A = [], g = new Array(n);
    for (let a = 0; a < n; a++) A.push(new Float64Array(n));
    for (let a = 0; a < n; a++) {
      const ca = J[a];
      let s = 0;
      for (let i = 0; i < m; i++) s += ca[i] * r[i];
      g[a] = -s;
      for (let b = a; b < n; b++) {
        const cb = J[b];
        let t = 0;
        for (let i = 0; i < m; i++) t += ca[i] * cb[i];
        A[a][b] = t; A[b][a] = t;
      }
    }
    let improved = false;
    for (let tries = 0; tries < 10; tries++) {
      const M = A.map((row, i) => { const c = Float64Array.from(row); c[i] += lambda * (row[i] + 1e-6); return c; });
      const step = skGauss(M, g.slice());
      if (!step) { lambda *= 10; continue; }
      const old = vars.map(([o, k]) => o[k]);
      vars.forEach(([o, k], j) => { o[k] += step[j]; });
      const res2 = skResAll(I, soft), f2 = sq(res2.r);
      if (f2 < f) { r = res2.r; f = f2; lambda = Math.max(1e-12, lambda / 4); improved = true; break; }
      vars.forEach(([o, k], j) => { o[k] = old[j]; });
      lambda *= 6;
    }
    if (!improved) break;
  }
  for (const c of skData().circles) if (!(c.r > 0.05)) c.r = Math.max(0.05, Math.abs(c.r || 0));
  let hn = 0;
  for (let i = 0; i < hard; i++) hn += r[i] * r[i];
  return Math.sqrt(hn);
}
/** Symmetric eigen decomposition (cyclic Jacobi). Columns of V = vectors. */
function skEigen(A) {
  const n = A.length, a = A.map(r => Float64Array.from(r));
  const V = Array.from({length:n}, (_, i) => { const r = new Float64Array(n); r[i] = 1; return r; });
  for (let sweep = 0; sweep < 30; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += a[p][q] * a[p][q];
    if (off < 1e-22) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-30) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) {
        const akp = a[k][p], akq = a[k][q];
        a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) {
        const apk = a[p][k], aqk = a[q][k];
        a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
    }
  }
  return {vals:a.map((r, i) => r[i]), V};
}
/** Which point coordinates / radii can still move (degrees of freedom). */
function skAnalyze() {
  const I = skIndex(), vars = skVars(), n = vars.length;
  skFree = new Set(); skDof = 0;
  if (!n) return;
  if (n > 240) { for (const [o, k] of vars) skFree.add(k + ":" + o.id); skDof = null; return; }
  const r = skResAll(I, null).r, m = r.length;
  const J = m ? skJacobian(I, vars, null, r) : null;
  const N = Array.from({length:n}, () => new Float64Array(n));
  if (J) for (let a = 0; a < n; a++) for (let b = a; b < n; b++) {
    let t = 0;
    for (let i = 0; i < m; i++) t += J[a][i] * J[b][i];
    N[a][b] = t; N[b][a] = t;
  }
  const {vals, V} = skEigen(N);
  const maxv = Math.max(1e-12, ...vals.map(Math.abs));
  vals.forEach((v, i) => {
    if (Math.abs(v) > 1e-8 * maxv + 1e-10) return;
    skDof++;
    for (let j = 0; j < n; j++) if (Math.abs(V[j][i]) > 1e-4) { const [o, k] = vars[j]; skFree.add(k + ":" + o.id); }
  });
}
const skPointFree = id => skFree.has("x:" + id) || skFree.has("y:" + id);
function skCurveFull(I, key) {
  const [t, id] = key.split(":");
  if (t === "l") { const l = I.L.get(id); return !skPointFree(l.p1) && !skPointFree(l.p2); }
  const c = I.C.get(id);
  if (t === "a") return !skPointFree(c.c) && !skPointFree(c.p1) && !skPointFree(c.p2);
  return !skPointFree(c.c) && !skFree.has("r:" + id);
}
function skMsg(text, bad) { status(text, bad); }
/** Runs an edit, solves, and undoes it if the sketch cannot be solved. */
function skCommit(edit, failText) {
  pushUndo();
  const before = JSON.stringify(skData());
  const result = edit();
  if (result === false) { undoStack.pop(); S.sketch = JSON.parse(before); changed(); return false; }
  const err = skSolve(null, 150);
  if (err > 1e-3) {
    S.sketch = JSON.parse(before);
    undoStack.pop();
    skSolve(null, 50);
    skMsg(failText || "Конфликт: эскиз переопределён — зависимость не добавлена", true);
    skAnalyze(); changed();
    return false;
  }
  skClean();
  skAnalyze(); changed();
  return true;
}
/** Drops constraints that point to deleted geometry and unused points. */
function skClean() {
  const d = skData();
  const used = new Set();
  for (const l of d.lines) used.add(l.p1), used.add(l.p2);
  for (const c of d.circles) used.add(c.c);
  for (const a of d.arcs) used.add(a.c), used.add(a.p1), used.add(a.p2);
  for (const p of d.points) if (p.standalone) used.add(p.id);
  d.points = d.points.filter(p => used.has(p.id));
  const I = skIndex();
  const pointRefs = ["a", "b", "p"];
  const curveRefs = ["l", "l1", "l2", "c", "c1", "c2", "e1", "e2"];
  d.cons = d.cons.filter(k => pointRefs.every(f => !k[f] || I.P.has(k[f]))
    && curveRefs.every(f => !k[f] || I.L.has(k[f]) || I.C.has(k[f])));
}

// ── curves, intersections, snapping ──
function skCurves(I, withVirtual) {
  const out = [];
  for (const l of I.L.values()) if (withVirtual || !l.virtual) { const [a, b] = skLP(I, l.id); out.push({key:"l:" + l.id, t:"l", a, b, obj:l}); }
  for (const c of skData().circles) { const g = skCirc(I, c.id); out.push({key:"c:" + c.id, t:"c", c:g.c, r:g.r, obj:c}); }
  for (const a of skData().arcs) { const g = skCirc(I, a.id); out.push({key:"a:" + a.id, t:"a", c:g.c, r:g.r, obj:a}); }
  return out;
}
function skIntersect(I, A, B) {
  const pts = [];
  const okOn = (cv, p) => cv.t !== "a" || skAngleOnArc(I, cv.obj, Math.atan2(p.y - cv.c.y, p.x - cv.c.x));
  if (A.t === "l" && B.t === "l") {
    const r = {x:A.b.x - A.a.x, y:A.b.y - A.a.y}, s = {x:B.b.x - B.a.x, y:B.b.y - B.a.y};
    const den = r.x * s.y - r.y * s.x;
    if (Math.abs(den) < 1e-12) return pts;
    const t = ((B.a.x - A.a.x) * s.y - (B.a.y - A.a.y) * s.x) / den;
    const u = ((B.a.x - A.a.x) * r.y - (B.a.y - A.a.y) * r.x) / den;
    if (t >= -1e-9 && t <= 1 + 1e-9 && u >= -1e-9 && u <= 1 + 1e-9) pts.push({x:A.a.x + r.x * t, y:A.a.y + r.y * t});
    return pts;
  }
  if (A.t !== "l" && B.t === "l") return skIntersect(I, B, A);
  if (A.t === "l") {
    const d = {x:A.b.x - A.a.x, y:A.b.y - A.a.y}, f = {x:A.a.x - B.c.x, y:A.a.y - B.c.y};
    const a = d.x * d.x + d.y * d.y, b = 2 * (f.x * d.x + f.y * d.y), c = f.x * f.x + f.y * f.y - B.r * B.r;
    const disc = b * b - 4 * a * c;
    if (disc < 0 || a < 1e-12) return pts;
    for (const sg of [-1, 1]) {
      const t = (-b + sg * Math.sqrt(disc)) / (2 * a);
      const p = {x:A.a.x + d.x * t, y:A.a.y + d.y * t};
      if (t >= -1e-9 && t <= 1 + 1e-9 && okOn(B, p)) pts.push(p);
      if (disc === 0) break;
    }
    return pts;
  }
  const dx = B.c.x - A.c.x, dy = B.c.y - A.c.y, d = Math.hypot(dx, dy);
  if (d < 1e-9 || d > A.r + B.r || d < Math.abs(A.r - B.r)) return pts;
  const a = (A.r * A.r - B.r * B.r + d * d) / (2 * d), h = Math.sqrt(Math.max(0, A.r * A.r - a * a));
  const mx = A.c.x + dx * a / d, my = A.c.y + dy * a / d;
  for (const sg of [-1, 1]) {
    const p = {x:mx + sg * h * dy / d, y:my - sg * h * dx / d};
    if (okOn(A, p) && okOn(B, p)) pts.push(p);
    if (h === 0) break;
  }
  return pts;
}
function skNearestOn(I, cv, p) {
  if (cv.t === "l") {
    const dx = cv.b.x - cv.a.x, dy = cv.b.y - cv.a.y, L2 = dx * dx + dy * dy || 1e-12;
    const t = Math.max(0, Math.min(1, ((p.x - cv.a.x) * dx + (p.y - cv.a.y) * dy) / L2));
    const q = {x:cv.a.x + dx * t, y:cv.a.y + dy * t};
    return {x:q.x, y:q.y, d:Math.hypot(p.x - q.x, p.y - q.y)};
  }
  const ang = Math.atan2(p.y - cv.c.y, p.x - cv.c.x);
  if (cv.t === "a" && !skAngleOnArc(I, cv.obj, ang)) {
    const {a1, sweep} = skArcAngles(I, cv.obj);
    const e1 = {x:cv.c.x + cv.r * Math.cos(a1), y:cv.c.y + cv.r * Math.sin(a1)};
    const e2 = {x:cv.c.x + cv.r * Math.cos(a1 + sweep), y:cv.c.y + cv.r * Math.sin(a1 + sweep)};
    const d1 = Math.hypot(p.x - e1.x, p.y - e1.y), d2 = Math.hypot(p.x - e2.x, p.y - e2.y);
    return d1 < d2 ? {...e1, d:d1} : {...e2, d:d2};
  }
  const q = {x:cv.c.x + cv.r * Math.cos(ang), y:cv.c.y + cv.r * Math.sin(ang)};
  return {x:q.x, y:q.y, d:Math.hypot(p.x - q.x, p.y - q.y)};
}
/** Snap like Fusion: point > midpoint > intersection > on curve > grid,
 *  plus horizontal/vertical alignment guides to the base point and others. */
function skSnap(p, base, exclude) {
  const I = skIndex(), tol = 10 / view.scale;
  let best = null, bd = tol;
  for (const q of I.P.values()) {
    if (exclude && exclude.has(q.id)) continue;
    const d = Math.hypot(q.x - p.x, q.y - p.y);
    if (d < bd) { bd = d; best = {x:q.x, y:q.y, kind:"point", ref:q.id}; }
  }
  if (!best) for (const r of S.rects) for (const [cx, cy] of [[r.x, r.y], [r.x + r.w, r.y], [r.x, r.y + r.h], [r.x + r.w, r.y + r.h],
      [r.x + r.w / 2, r.y], [r.x + r.w / 2, r.y + r.h], [r.x, r.y + r.h / 2], [r.x + r.w, r.y + r.h / 2]]) {
    const d = Math.hypot(cx - p.x, cy - p.y);
    if (d < bd) { bd = d; best = {x:cx, y:cy, kind:"pos"}; }
  }
  const curves = skCurves(I, true).filter(cv => !(exclude && cv.t === "l" && (exclude.has(cv.obj.p1) || exclude.has(cv.obj.p2))));
  if (!best) for (const cv of curves) if (cv.t === "l") {
    const m = {x:(cv.a.x + cv.b.x) / 2, y:(cv.a.y + cv.b.y) / 2}, d = Math.hypot(m.x - p.x, m.y - p.y);
    if (d < bd) { bd = d; best = {x:m.x, y:m.y, kind:"mid", ref:cv.obj.id}; }
  }
  if (!best) for (let i = 0; i < curves.length; i++) for (let j = i + 1; j < curves.length; j++) {
    for (const q of skIntersect(I, curves[i], curves[j])) {
      const d = Math.hypot(q.x - p.x, q.y - p.y);
      if (d < bd) { bd = d; best = {x:q.x, y:q.y, kind:"int", ref:[curves[i].key, curves[j].key]}; }
    }
  }
  if (!best) for (const cv of curves) {
    const q = skNearestOn(I, cv, p);
    if (q.d < bd) { bd = q.d; best = {x:q.x, y:q.y, kind:"on", ref:cv.key}; }
  }
  const out = best || {x:snapV(p.x), y:snapV(p.y), kind:"grid"};
  out.guides = [];
  if (out.kind === "grid" || out.kind === "pos") {
    const cands = base ? [{x:base.x, y:base.y, base:true}] : [];
    for (const q of skData().points) if (!(exclude && exclude.has(q.id))) cands.push(q);
    let gx = null, gy = null;
    for (const q of cands) {
      if (gx === null && Math.abs(q.x - p.x) < tol) gx = q;
      if (gy === null && Math.abs(q.y - p.y) < tol) gy = q;
    }
    if (gx) { out.x = gx.x; out.guides.push({from:gx, v:true}); if (gx.base) out.hv = "vertical"; }
    if (gy) { out.y = gy.y; out.guides.push({from:gy, v:false}); if (gy.base) out.hv = out.hv ? null : "horizontal"; }
  }
  return out;
}

// ── creating geometry ──
function skAddPoint(x, y) { const p = {id:skNewId("p"), x, y}; skData().points.push(p); return p.id; }
function skCurveCon(pid, key) {
  const [t, id] = key.split(":");
  skData().cons.push(t === "l" ? {id:skNewId("k"), type:"onLine", p:pid, l:id} : {id:skNewId("k"), type:"onCircle", p:pid, c:id});
}
/** A snapped position becomes a point id: an existing sketch point is
 *  shared; other snaps add the matching constraint. */
function skResolve(pos) {
  const d = skData(), s = pos.snap || pos;
  if (s.kind === "point" && d.points.some(p => p.id === s.ref)) return s.ref;
  const id = skAddPoint(pos.x, pos.y);
  if (s.kind === "point") d.cons.push({id:skNewId("k"), type:"coincident", a:id, b:s.ref});
  else if (s.kind === "mid") d.cons.push({id:skNewId("k"), type:"midpoint", p:id, l:s.ref});
  else if (s.kind === "int") { skCurveCon(id, s.ref[0]); skCurveCon(id, s.ref[1]); }
  else if (s.kind === "on") skCurveCon(id, s.ref);
  return id;
}
const skPos = s => ({x:s.x, y:s.y, snap:s});
function skAddLine(a, b, construction) {
  const l = {id:skNewId("l"), p1:a, p2:b, construction:!!construction};
  skData().lines.push(l); return l.id;
}
function skAddDim(k) {
  const full = Object.assign({id:skNewId("k"), dim:true, driven:false}, k);
  skData().cons.push(full);
  return full;
}

// ── HUD (typed values while drawing, like Fusion) ──
let hud = null;   // {fields:[{key, input}], onEnter, onChange}
function evalNum(str) {
  str = String(str).replace(",", ".").trim();
  if (!str || !/^[\d\s.+\-*/()]+$/.test(str)) return NaN;
  try { const v = Function('"use strict";return (' + str + ")")(); return Number.isFinite(v) ? v : NaN; }
  catch (e) { return NaN; }
}
function hudShow(fields, sx, sy, onEnter, onChange) {
  const box = $("hud");
  box.innerHTML = "";
  hud = {fields:[], onEnter, onChange};
  for (const f of fields) {
    const l = document.createElement("label");
    l.textContent = f.label;
    const i = document.createElement("input");
    i.type = "text"; i.value = f.value !== undefined ? fmt(f.value) : ""; i.dataset.locked = "";
    i.addEventListener("input", () => { i.dataset.locked = i.value.trim() ? "1" : ""; if (hud && hud.onChange) hud.onChange(); draw(); });
    i.addEventListener("keydown", e => {
      e.stopPropagation();
      if (e.key === "Enter") { e.preventDefault(); const cb = hud && hud.onEnter; if (cb) cb(hudValues()); }
      else if (e.key === "Escape") { e.preventDefault(); skCancel(); }
      else if (e.key === "Tab") {
        e.preventDefault();
        const list = hud.fields.map(x => x.input), k = list.indexOf(i);
        const next = list[(k + (e.shiftKey ? list.length - 1 : 1)) % list.length];
        next.focus(); next.select();
      }
    });
    l.appendChild(i); box.appendChild(l);
    hud.fields.push({key:f.key, input:i});
  }
  box.style.display = "flex";
  hudMove(sx, sy);
}
function hudMove(sx, sy) {
  if (!hud) return;
  const box = $("hud");
  box.style.left = Math.round(sx + 18) + "px"; box.style.top = Math.round(sy + 18) + "px";
}
function hudHide() { hud = null; const b = $("hud"); b.style.display = "none"; b.innerHTML = ""; }
function hudValues() {
  const out = {};
  if (hud) for (const f of hud.fields) out[f.key] = f.input.dataset.locked ? evalNum(f.input.value) : NaN;
  return out;
}
function hudSet(key, v) {
  if (!hud) return;
  const f = hud.fields.find(x => x.key === key);
  if (f && !f.input.dataset.locked && document.activeElement !== f.input) f.input.value = fmt(v);
}

// ── tools ──
function skSetTool(t) {
  skCancel();
  skTool = t;
  for (const b of $("skTools").querySelectorAll("button[data-tool]")) b.classList.toggle("on", b.dataset.tool === t);
  $("hint").textContent = SK_HINTS[t] || SK_HINTS.select;
  if (t === "mirror" && !skSel.some(k => !k.startsWith("k:"))) skMsg("Зеркало: сначала выделите элементы (инструмент Выбор)", true);
  draw();
}
function skCancel() {
  skDraft = null; hudHide(); skDrag = null; skDraftTrim = null;
  if (skTool.startsWith("con:")) { skTool = "select"; $("hint").textContent = SK_HINTS.select; }
  draw();
}
function enterSketch(on) {
  sketchMode = on;
  skCancel();
  skSel = [];
  showEl($("tools"), !on, "");
  showEl($("secRobot"), !on);
  showEl($("skTools"), on, "flex");
  $("modeSketch").textContent = on ? "✓ Завершить эскиз" : "✎ Эскиз";
  $("modeSketch").classList.toggle("on", on);
  if (on) { skData(); skSolve(null, 60); skAnalyze(); skSetTool("select"); }
  else { $("hint").textContent = HINTS[tool]; }
  changed();
}

// ── hit testing ──
function skHitAt(sx, sy, opts) {
  opts = opts || {};
  if (!opts.noBoxes) for (const b of skBoxes) if (sx >= b.x && sx <= b.x + b.w && sy >= b.y && sy <= b.y + b.h) return b.key;
  const p = toCm(sx, sy), I = skIndex(), tol = 7 / view.scale;
  let best = null, bd = tol;
  for (const q of I.P.values()) {
    if (q.virtual && !opts.virtual) continue;
    const d = Math.hypot(q.x - p.x, q.y - p.y);
    if (d < bd) { bd = d; best = "p:" + q.id; }
  }
  if (best) return best;
  bd = tol;
  for (const cv of skCurves(I, !!opts.virtual)) {
    const q = skNearestOn(I, cv, p);
    if (q.d < bd) { bd = q.d; best = cv.key; }
  }
  return best;
}
const skKind = key => key ? key.split(":")[0] : null;
const skKid = key => key.slice(2);
const SK_VIRTUAL_KEY = /^(p:O|p:OX|p:OY|p:OXY|l:E[BRTL])$/;

// ── constraints from a selection ──
function skMakeConstraint(type, keys) {
  const I = skIndex();
  const pts = keys.filter(k => skKind(k) === "p").map(skKid);
  const lines = keys.filter(k => skKind(k) === "l").map(skKid);
  const circs = keys.filter(k => skKind(k) === "c" || skKind(k) === "a").map(skKid);
  const n = keys.length, K = o => Object.assign({id:skNewId("k"), type}, o);
  switch (type) {
    case "horizontal": case "vertical":
      if (lines.length === n && n >= 1) return lines.map(l => K({l}));
      if (pts.length === 2 && n === 2) return [K({a:pts[0], b:pts[1]})];
      return n >= 2 ? "нужна линия или две точки" : null;
    case "coincident":
      if (pts.length === 2 && n === 2) return [K({a:pts[0], b:pts[1]})];
      if (pts.length === 1 && lines.length === 1 && n === 2) return [{id:skNewId("k"), type:"onLine", p:pts[0], l:lines[0]}];
      if (pts.length === 1 && circs.length === 1 && n === 2) return [{id:skNewId("k"), type:"onCircle", p:pts[0], c:circs[0]}];
      return n >= 2 ? "нужны две точки или точка и кривая" : null;
    case "parallel": case "perpendicular": case "collinear":
      if (lines.length === 2 && n === 2) return [K({l1:lines[0], l2:lines[1]})];
      return n >= 2 ? "нужны две линии" : null;
    case "equal":
      if ((lines.length === 2 || circs.length === 2) && n === 2) return [K({e1:(lines[0] || circs[0]), e2:(lines[1] || circs[1])})];
      return n >= 2 ? "нужны две линии или две окружности/дуги" : null;
    case "tangent":
      if (lines.length === 1 && circs.length === 1 && n === 2) {
        const l = I.L.get(lines[0]), arc = I.C.get(circs[0]);
        const shared = arc.p1 ? [arc.p1, arc.p2].find(q => q === l.p1 || q === l.p2) : null;
        if (shared) return [{id:skNewId("k"), type:"tangentEnd", l:lines[0], c:circs[0], p:shared}];
        const [a, b] = skLP(I, lines[0]), c = skCirc(I, circs[0]);
        return [K({l:lines[0], c:circs[0], s:skSdist(a, b, c.c) >= 0 ? 1 : -1})];
      }
      if (circs.length === 2 && n === 2) {
        const c1 = skCirc(I, circs[0]), c2 = skCirc(I, circs[1]);
        const d = Math.hypot(c2.c.x - c1.c.x, c2.c.y - c1.c.y);
        const inner = Math.abs(d - Math.abs(c1.r - c2.r)) < Math.abs(d - (c1.r + c2.r));
        return [{id:skNewId("k"), type:"tangentCC", c1:circs[0], c2:circs[1], mode:inner ? "in" : "out", s:c1.r >= c2.r ? 1 : -1}];
      }
      return n >= 2 ? "нужны линия и окружность/дуга (или две окружности)" : null;
    case "midpoint":
      if (pts.length === 1 && lines.length === 1 && n === 2) return [K({p:pts[0], l:lines[0]})];
      return n >= 2 ? "нужны точка и линия" : null;
    case "concentric":
      if (circs.length === 2 && n === 2) return [K({c1:circs[0], c2:circs[1]})];
      return n >= 2 ? "нужны две окружности/дуги" : null;
    case "symmetric":
      if (pts.length === 2 && lines.length === 1 && n === 3) return [K({a:pts[0], b:pts[1], l:lines[0]})];
      return n >= 3 ? "нужны две точки и линия (ось)" : null;
  }
  return null;
}
function skApplyConstraint(type) {
  const keys = skSel.filter(k => !k.startsWith("k:"));
  if (type === "fix") {
    if (!keys.length) { skTool = "con:fix"; $("hint").textContent = "Кликните точку или кривую, чтобы зафиксировать/освободить"; return; }
    skToggleFix(keys); return;
  }
  const made = keys.length ? skMakeConstraint(type, keys) : null;
  if (Array.isArray(made)) {
    skCommit(() => { skData().cons.push(...made); });
    skSel = []; changed();
    return;
  }
  if (typeof made === "string") skMsg(made, true);
  skDraft = null; hudHide();
  skTool = "con:" + type; skSel = [];
  $("hint").textContent = "Зависимость «" + SK_NAMES[type] + "»: кликайте элементы (Esc — отмена)";
  draw();
}
function skToggleFix(keys) {
  const I = skIndex(), d = skData();
  const ptIds = new Set();
  pushUndo();
  for (const k of keys) {
    const t = skKind(k), id = skKid(k);
    if (t === "p") ptIds.add(id);
    else if (t === "l") { const l = I.L.get(id); if (!l.virtual) ptIds.add(l.p1), ptIds.add(l.p2); }
    else if (t === "c") { const c = I.C.get(id); ptIds.add(c.c); c.fixedR = !c.fixedR; }
    else if (t === "a") { const a = I.C.get(id); ptIds.add(a.c); ptIds.add(a.p1); ptIds.add(a.p2); }
  }
  const pts = d.points.filter(p => ptIds.has(p.id));
  const target = !pts.every(p => p.fixed);
  for (const p of pts) p.fixed = target;
  skAnalyze(); changed();
}

// ── dimensions ──
function skDimSpec(refs, place) {
  const I = skIndex();
  const kinds = refs.map(skKind), ids = refs.map(skKid);
  const twoPoints = (a, b) => {
    const A = I.P.get(a), B = I.P.get(b);
    const minx = Math.min(A.x, B.x), maxx = Math.max(A.x, B.x), miny = Math.min(A.y, B.y), maxy = Math.max(A.y, B.y);
    const inX = place.x > minx && place.x < maxx, inY = place.y > miny && place.y < maxy;
    if (inX && !inY && maxx - minx > 1e-6) return {type:"hdist", a, b, s:B.x >= A.x ? 1 : -1, off:place.y - (A.y + B.y) / 2};
    if (inY && !inX && maxy - miny > 1e-6) return {type:"vdist", a, b, s:B.y >= A.y ? 1 : -1, off:place.x - (A.x + B.x) / 2};
    const u = skUnit(A, B), mid = {x:(A.x + B.x) / 2, y:(A.y + B.y) / 2};
    return {type:"dist", a, b, off:(place.x - mid.x) * -u.y + (place.y - mid.y) * u.x};
  };
  if (refs.length === 1) {
    if (kinds[0] === "l") { const l = I.L.get(ids[0]); return twoPoints(l.p1, l.p2); }
    if (kinds[0] === "c" || kinds[0] === "a") {
      const g = skCirc(I, ids[0]);
      return {type:kinds[0] === "c" ? "diameter" : "radius", c:ids[0],
        ang:Math.atan2(place.y - g.c.y, place.x - g.c.x), off:Math.max(2, Math.hypot(place.x - g.c.x, place.y - g.c.y) - g.r)};
    }
    return null;
  }
  if (kinds[0] === "p" && kinds[1] === "p") return twoPoints(ids[0], ids[1]);
  if ((kinds[0] === "p" && kinds[1] === "l") || (kinds[0] === "l" && kinds[1] === "p")) {
    const p = kinds[0] === "p" ? ids[0] : ids[1], l = kinds[0] === "l" ? ids[0] : ids[1];
    const [a, b] = skLP(I, l);
    return {type:"pointLine", p, l, s:skSdist(a, b, I.P.get(p)) >= 0 ? 1 : -1, off:0};
  }
  if (kinds[0] === "l" && kinds[1] === "l") {
    const [a, b] = skLP(I, ids[0]), [c, d] = skLP(I, ids[1]);
    const u = skUnit(a, b), v = skUnit(c, d);
    if (Math.abs(u.x * v.y - u.y * v.x) < 0.02) {
      return {type:"pointLine", p:I.L.get(ids[1]).p1, l:ids[0], s:skSdist(a, b, c) >= 0 ? 1 : -1, off:0};
    }
    const X = skLineX(a, b, c, d) || a;
    const s1 = ((place.x - X.x) * (b.x - a.x) + (place.y - X.y) * (b.y - a.y)) >= 0 ? 1 : -1;
    const s2 = ((place.x - X.x) * (d.x - c.x) + (place.y - X.y) * (d.y - c.y)) >= 0 ? 1 : -1;
    return {type:"angle", l1:ids[0], l2:ids[1], s1, s2, off:Math.max(4, Math.hypot(place.x - X.x, place.y - X.y))};
  }
  return null;
}
function skLineX(a, b, c, d) {
  const r = {x:b.x - a.x, y:b.y - a.y}, s = {x:d.x - c.x, y:d.y - c.y}, den = r.x * s.y - r.y * s.x;
  if (Math.abs(den) < 1e-12) return null;
  const t = ((c.x - a.x) * s.y - (c.y - a.y) * s.x) / den;
  return {x:a.x + r.x * t, y:a.y + r.y * t};
}
function skPlaceDim(spec) {
  const I = skIndex();
  const dimObj = Object.assign({}, spec);
  dimObj.v = skMeasure(I, Object.assign({}, spec, {v:0}));
  pushUndo();
  const k = skAddDim(dimObj);
  skAnalyze();
  // A dimension that adds no information would over-constrain the sketch:
  // it becomes a driven (reference) dimension, like Fusion offers.
  if (!skDimAddsInfo(k)) { k.driven = true; skMsg("Эскиз уже определён — размер добавлен как справочный (в скобках)"); }
  skAnalyze(); changed();
  skEditDim(k.id);
}
/** False when the new dimension removes no degree of freedom (redundant). */
function skDimAddsInfo(k) {
  const withIt = skDof;
  k.driven = true; skAnalyze(); const without = skDof;
  k.driven = false; skAnalyze();
  return withIt === null || without === null ? true : without > withIt;
}
function skEditDim(id) {
  const k = skData().cons.find(c => c.id === id);
  if (!k) return;
  draw();
  const box = skBoxes.find(b => b.key === "k:" + id);
  const sx = box ? box.x : 100, sy = box ? box.y - 30 : 100;
  const unit = k.type === "angle" || k.type === "angleH" ? "°" : "см";
  const cur = k.driven ? skMeasure(skIndex(), k) : k.v;
  hudShow([{key:"v", label:SK_NAMES[k.type] + ", " + unit, value:cur}], sx, sy, vals => {
    const v = vals.v;
    hudHide();
    if (!Number.isFinite(v)) return;
    if (k.type !== "angleH" && v <= 0) { skMsg("Размер должен быть больше 0", true); return; }
    skCommitDim(k, v, {v:k.v, driven:k.driven});
  });
  const f = hud.fields[0].input; f.dataset.locked = "1"; setTimeout(() => { f.focus(); f.select(); }, 0);
}
function skCommitDim(k, v, was) {
  pushUndo();
  const before = JSON.stringify(skData());
  const live = skData().cons.find(c => c.id === k.id);
  if (!live) return;
  live.v = v; live.driven = false;
  const err = skSolve(null, 200);
  if (err > 1e-3) {
    S.sketch = JSON.parse(before);
    const back = skData().cons.find(c => c.id === k.id);
    if (back) { back.v = was.v; back.driven = was.driven; }
    skSolve(null, 50);
    skMsg("Не получается: размер конфликтует с другими зависимостями", true);
  }
  skAnalyze(); changed();
}

// ── trim / fillet / offset / mirror ──
function skCurveParams(I, cv, p) {
  if (cv.t === "l") { const dx = cv.b.x - cv.a.x, dy = cv.b.y - cv.a.y, L2 = dx * dx + dy * dy || 1e-12;
    return ((p.x - cv.a.x) * dx + (p.y - cv.a.y) * dy) / L2; }
  const ang = Math.atan2(p.y - cv.c.y, p.x - cv.c.x);
  if (cv.t === "c") return sk2pi(ang);
  return sk2pi(ang - skArcAngles(I, cv.obj).a1);
}
function skTrimPlan(sx, sy) {
  const key = skHitAt(sx, sy, {noBoxes:true});
  if (!key || skKind(key) === "p") return null;
  const I = skIndex(), curves = skCurves(I, true);
  const cv = curves.find(c => c.key === key);
  if (!cv || (cv.t === "l" && cv.obj.virtual)) return null;
  const p = toCm(sx, sy), tc = skCurveParams(I, cv, p);
  const cuts = [];
  for (const o of curves) {
    if (o.key === key) continue;
    for (const q of skIntersect(I, cv, o)) cuts.push({t:skCurveParams(I, cv, q), key:o.key, x:q.x, y:q.y});
  }
  const end = cv.t === "l" ? 1 : cv.t === "c" ? 2 * Math.PI : skArcAngles(I, cv.obj).sweep;
  const eps = 1e-6;
  const inner = cuts.filter(c => c.t > eps && c.t < end - eps);
  let lo = null, hi = null;
  for (const c of inner) {
    if (c.t < tc && (!lo || c.t > lo.t)) lo = c;
    if (c.t > tc && (!hi || c.t < hi.t)) hi = c;
  }
  if (cv.t === "c") {
    if (inner.length < 2) return {cv, whole:true};
    if (!lo) lo = inner.reduce((m, c) => c.t > m.t ? c : m);
    if (!hi) hi = inner.reduce((m, c) => c.t < m.t ? c : m);
  }
  return {cv, lo, hi, end, whole:!lo && !hi};
}
function skTrim(plan) {
  const {cv, lo, hi} = plan, id = cv.obj.id;
  skCommit(() => {
    const d = skData();
    if (plan.whole) { skDelete([cv.key]); return; }
    if (cv.t === "l") {
      const l = d.lines.find(x => x.id === id);
      if (lo && hi) {
        const pLo = skAddPoint(lo.x, lo.y), pHi = skAddPoint(hi.x, hi.y);
        skCurveCon(pLo, lo.key); skCurveCon(pHi, hi.key);
        const nl = skAddLine(pHi, l.p2, l.construction);
        l.p2 = pLo;
        d.cons.push({id:skNewId("k"), type:"collinear", l1:l.id, l2:nl});
      } else if (lo) { const pLo = skAddPoint(lo.x, lo.y); skCurveCon(pLo, lo.key); l.p2 = pLo; }
      else { const pHi = skAddPoint(hi.x, hi.y); skCurveCon(pHi, hi.key); l.p1 = pHi; }
    } else if (cv.t === "c") {
      const c = d.circles.find(x => x.id === id);
      const pA = skAddPoint(hi.x, hi.y), pB = skAddPoint(lo.x, lo.y);
      skCurveCon(pA, hi.key); skCurveCon(pB, lo.key);
      d.circles = d.circles.filter(x => x !== c);
      d.arcs.push({id:c.id, c:c.c, p1:pA, p2:pB, construction:c.construction});
    } else {
      const a = d.arcs.find(x => x.id === id);
      if (lo && hi) {
        const pLo = skAddPoint(lo.x, lo.y), pHi = skAddPoint(hi.x, hi.y);
        skCurveCon(pLo, lo.key); skCurveCon(pHi, hi.key);
        const na = {id:skNewId("a"), c:a.c, p1:pHi, p2:a.p2, construction:a.construction};
        a.p2 = pLo; d.arcs.push(na);
      } else if (lo) { const pLo = skAddPoint(lo.x, lo.y); skCurveCon(pLo, lo.key); a.p2 = pLo; }
      else { const pHi = skAddPoint(hi.x, hi.y); skCurveCon(pHi, hi.key); a.p1 = pHi; }
    }
  }, "Обрезка не удалась");
}
function skFilletCorner(p) {
  const d = skData(), tol = 18 / view.scale;
  let best = null, bd = tol;
  for (const pt of d.points) {
    const ls = d.lines.filter(l => l.p1 === pt.id || l.p2 === pt.id);
    if (ls.length !== 2) continue;
    const dd = Math.hypot(pt.x - p.x, pt.y - p.y);
    if (dd < bd) { bd = dd; best = {pt, ls}; }
  }
  return best;
}
function skFilletGeom(corner, r) {
  const I = skIndex(), P = I.P.get(corner.pt.id);
  if (!P) return null;
  const other = l => I.P.get(l.p1 === P.id ? l.p2 : l.p1);
  const A = other(corner.ls[0]), B = other(corner.ls[1]);
  const u = skUnit(P, A), v = skUnit(P, B);
  const cos = u.x * v.x + u.y * v.y, th = Math.acos(Math.max(-1, Math.min(1, cos)));
  if (th < 0.05 || th > Math.PI - 0.05) return null;
  const t = r / Math.tan(th / 2);
  if (t >= Math.hypot(A.x - P.x, A.y - P.y) || t >= Math.hypot(B.x - P.x, B.y - P.y)) return null;
  const bis = skUnit({x:0, y:0}, {x:u.x + v.x, y:u.y + v.y}), cd = r / Math.sin(th / 2);
  return {T1:{x:P.x + u.x * t, y:P.y + u.y * t}, T2:{x:P.x + v.x * t, y:P.y + v.y * t},
    C:{x:P.x + bis.x * cd, y:P.y + bis.y * cd}, r};
}
function skFillet(corner, r) {
  const g = skFilletGeom(corner, r);
  if (!g) { skMsg("Скругление не помещается в этот угол", true); return; }
  skCommit(() => {
    const d = skData(), P = corner.pt.id;
    const t1 = skAddPoint(g.T1.x, g.T1.y), t2 = skAddPoint(g.T2.x, g.T2.y), c = skAddPoint(g.C.x, g.C.y);
    const ls = corner.ls.map(l0 => d.lines.find(l => l.id === l0.id));
    for (const [l, t] of [[ls[0], t1], [ls[1], t2]]) { if (l.p1 === P) l.p1 = t; else l.p2 = t; }
    // counter-clockwise from p1 to p2: the order with the short sweep
    const a1 = Math.atan2(g.T1.y - g.C.y, g.T1.x - g.C.x), a2 = Math.atan2(g.T2.y - g.C.y, g.T2.x - g.C.x);
    const ccw = sk2pi(a2 - a1) < Math.PI;
    const arc = {id:skNewId("a"), c, p1:ccw ? t1 : t2, p2:ccw ? t2 : t1, construction:false};
    d.arcs.push(arc);
    const I = skIndex();
    void I;
    d.cons.push({id:skNewId("k"), type:"tangentEnd", l:ls[0].id, c:arc.id, p:t1},
      {id:skNewId("k"), type:"tangentEnd", l:ls[1].id, c:arc.id, p:t2});
    // The corner stays as a "virtual sharp" (like Fusion): a point on both
    // lines, so dimensions to the old corner keep working.
    const sharp = d.points.find(p => p.id === P);
    if (sharp) sharp.standalone = true;
    d.cons.push({id:skNewId("k"), type:"onLine", p:P, l:ls[0].id}, {id:skNewId("k"), type:"onLine", p:P, l:ls[1].id});
    skAddDim({type:"radius", c:arc.id, v:r, ang:Math.atan2(corner.pt.y - g.C.y, corner.pt.x - g.C.x), off:4});
  }, "Скругление конфликтует с зависимостями");
}
function skOffset(key, dist, side) {
  const t = skKind(key), id = skKid(key);
  skCommit(() => {
    const d = skData(), I = skIndex();
    if (t === "l") {
      const [a, b] = skLP(I, id), u = skUnit(a, b), n = {x:u.y * side, y:-u.x * side};
      const p1 = skAddPoint(a.x + n.x * dist, a.y + n.y * dist), p2 = skAddPoint(b.x + n.x * dist, b.y + n.y * dist);
      const nl = skAddLine(p1, p2, skConstruction);
      d.cons.push({id:skNewId("k"), type:"parallel", l1:id, l2:nl});
      const I2 = skIndex(), [c1, c2] = skLP(I2, id);
      skAddDim({type:"pointLine", p:p1, l:id, s:skSdist(c1, c2, I2.P.get(p1)) >= 0 ? 1 : -1, v:dist, off:0});
    } else {
      const g = skCirc(I, id), r = Math.max(0.5, g.r + side * dist);
      const c = {id:skNewId("c"), c:g.obj.c, r, construction:skConstruction};
      d.circles.push(c);
      skAddDim({type:"diameter", c:c.id, v:2 * r, ang:0.6, off:4});
    }
  }, "Смещение не удалось");
}
function skMirror(axisId) {
  const keys = skSel.filter(k => !k.startsWith("k:") && k !== "l:" + axisId && !SK_VIRTUAL_KEY.test(k));
  if (!keys.length) { skMsg("Нечего отражать: выделите элементы", true); return; }
  skCommit(() => {
    const d = skData(), I = skIndex(), [a, b] = skLP(I, axisId), u = skUnit(a, b);
    const map = new Map();
    const mp = id => {
      if (map.has(id)) return map.get(id);
      const p = I.P.get(id), dist = skSdist(a, b, p);
      if (Math.abs(dist) < 1e-6) { map.set(id, id); return id; }
      const along = (p.x - a.x) * u.x + (p.y - a.y) * u.y;
      const foot = {x:a.x + u.x * along, y:a.y + u.y * along};
      const nid = skAddPoint(2 * foot.x - p.x, 2 * foot.y - p.y);
      d.cons.push({id:skNewId("k"), type:"symmetric", a:id, b:nid, l:axisId});
      map.set(id, nid); return nid;
    };
    for (const k of keys) {
      const t = skKind(k), id = skKid(k);
      if (t === "l") { const l = I.L.get(id); skAddLine(mp(l.p1), mp(l.p2), l.construction); }
      else if (t === "c") { const c = I.C.get(id); const nc = {id:skNewId("c"), c:mp(c.c), r:c.r, construction:c.construction};
        d.circles.push(nc); d.cons.push({id:skNewId("k"), type:"equal", e1:c.id, e2:nc.id}); }
      else if (t === "a") { const ar = I.C.get(id);
        d.arcs.push({id:skNewId("a"), c:mp(ar.c), p1:mp(ar.p2), p2:mp(ar.p1), construction:ar.construction}); }
      else if (t === "p") { const q = mp(id); const qp = d.points.find(x => x.id === q); if (qp) qp.standalone = true; }
    }
  }, "Зеркало не удалось");
  skSel = [];
}
function skDelete(keys) {
  const d = skData();
  keys = keys.filter(k => !SK_VIRTUAL_KEY.test(k));
  const pts = new Set(keys.filter(k => skKind(k) === "p").map(skKid));
  const curves = new Set(keys.filter(k => ["l", "c", "a"].includes(skKind(k))).map(skKid));
  const cons = new Set(keys.filter(k => skKind(k) === "k").map(skKid));
  d.lines = d.lines.filter(l => !curves.has(l.id) && !pts.has(l.p1) && !pts.has(l.p2));
  d.circles = d.circles.filter(c => !curves.has(c.id) && !pts.has(c.c));
  d.arcs = d.arcs.filter(a => !curves.has(a.id) && !pts.has(a.c) && !pts.has(a.p1) && !pts.has(a.p2));
  d.points = d.points.filter(p => !pts.has(p.id));
  d.cons = d.cons.filter(k => !cons.has(k.id));
  for (const p of d.points) if (p.standalone && pts.has(p.id)) p.standalone = false;
  skClean();
}

// ── drawing tools: commit ──
function skCommitLine(startPos, endPos, typed) {
  let ids = null;
  skCommit(() => {
    const d = skData();
    const a = skResolve(startPos);
    const b = skResolve(endPos);
    if (a === b) return false;
    const id = skAddLine(a, b, skConstruction);
    if (Number.isFinite(typed.len)) skAddDim({type:"dist", a, b, v:typed.len, off:4});
    if (Number.isFinite(typed.ang)) d.cons.push({id:skNewId("k"), type:"angleH", l:id, v:typed.ang, dim:true, driven:false});
    else {
      const ang = Math.atan2(endPos.y - startPos.y, endPos.x - startPos.x) * 180 / Math.PI;
      const near = t => Math.abs(skWrap(rad(ang - t))) < rad(2);
      if (endPos.snap && endPos.snap.hv) d.cons.push({id:skNewId("k"), type:endPos.snap.hv, l:id});
      else if (near(0) || near(180)) d.cons.push({id:skNewId("k"), type:"horizontal", l:id});
      else if (near(90) || near(-90)) d.cons.push({id:skNewId("k"), type:"vertical", l:id});
    }
    ids = {a, b};
  }, "Линия конфликтует с зависимостями");
  return ids;
}
function skRectCorners(a, b) { return [{x:a.x, y:a.y}, {x:b.x, y:a.y}, {x:b.x, y:b.y}, {x:a.x, y:b.y}]; }
function skCommitRect(aPos, bPos, typed, centered) {
  skCommit(() => {
    const d = skData();
    const tw = Number.isFinite(typed.w), th = Number.isFinite(typed.h);
    let c1, c3, center = null;
    if (centered) {
      const w = tw ? typed.w / 2 : Math.abs(bPos.x - aPos.x);
      const h = th ? typed.h / 2 : Math.abs(bPos.y - aPos.y);
      if (w < 0.1 || h < 0.1) return false;
      const sx = bPos.x >= aPos.x ? 1 : -1, sy = bPos.y >= aPos.y ? 1 : -1;
      c1 = {x:aPos.x - sx * w, y:aPos.y - sy * h}; c3 = {x:aPos.x + sx * w, y:aPos.y + sy * h};
      center = skResolve(aPos);
    } else {
      const w = tw ? typed.w * (bPos.x >= aPos.x ? 1 : -1) : bPos.x - aPos.x;
      const h = th ? typed.h * (bPos.y >= aPos.y ? 1 : -1) : bPos.y - aPos.y;
      if (Math.abs(w) < 0.1 || Math.abs(h) < 0.1) return false;
      c1 = aPos; c3 = {x:aPos.x + w, y:aPos.y + h};
    }
    const cs = skRectCorners(c1, c3);
    const ids = cs.map((c, i) => {
      if (!centered && i === 0) return skResolve(aPos);
      if (!centered && i === 2 && !tw && !th) return skResolve(bPos);
      return skAddPoint(c.x, c.y);
    });
    const ls = [0, 1, 2, 3].map(i => skAddLine(ids[i], ids[(i + 1) % 4], skConstruction));
    d.cons.push({id:skNewId("k"), type:"horizontal", l:ls[0]}, {id:skNewId("k"), type:"vertical", l:ls[1]},
      {id:skNewId("k"), type:"horizontal", l:ls[2]}, {id:skNewId("k"), type:"vertical", l:ls[3]});
    if (centered) {
      const d1 = skAddLine(ids[0], ids[2], true), d2 = skAddLine(ids[1], ids[3], true);
      d.cons.push({id:skNewId("k"), type:"midpoint", p:center, l:d1}, {id:skNewId("k"), type:"midpoint", p:center, l:d2});
    }
    if (tw) skAddDim({type:"hdist", a:ids[0], b:ids[1], s:cs[1].x >= cs[0].x ? 1 : -1, v:typed.w, off:(cs[0].y <= cs[2].y ? -5 : 5)});
    if (th) skAddDim({type:"vdist", a:ids[1], b:ids[2], s:cs[2].y >= cs[1].y ? 1 : -1, v:typed.h, off:(cs[1].x >= cs[0].x ? 5 : -5)});
  }, "Прямоугольник конфликтует с зависимостями");
}
function skCommitCircle(cPos, edge, typed) {
  skCommit(() => {
    const r = Number.isFinite(typed.d) ? typed.d / 2 : Math.hypot(edge.x - cPos.x, edge.y - cPos.y);
    if (r < 0.1) return false;
    const c = {id:skNewId("c"), c:skResolve(cPos), r, construction:skConstruction};
    skData().circles.push(c);
    if (Number.isFinite(typed.d)) skAddDim({type:"diameter", c:c.id, v:typed.d, ang:0.7, off:4});
  }, "Окружность конфликтует с зависимостями");
}
function skCircle3(a, b, c) {
  const d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (Math.abs(d) < 1e-9) return null;
  const ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) + (c.x * c.x + c.y * c.y) * (a.y - b.y)) / d;
  const uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) + (c.x * c.x + c.y * c.y) * (b.x - a.x)) / d;
  return {x:ux, y:uy, r:Math.hypot(a.x - ux, a.y - uy)};
}
function skArcOrder(a, b, m, ctr) {
  const a1 = Math.atan2(a.y - ctr.y, a.x - ctr.x), a2 = Math.atan2(b.y - ctr.y, b.x - ctr.x), am = Math.atan2(m.y - ctr.y, m.x - ctr.x);
  return sk2pi(am - a1) <= sk2pi(a2 - a1);    // true: a -> b counter-clockwise passes m
}
function skCommitArc(aPos, bPos, mPos) {
  const g = skCircle3(aPos, bPos, mPos);
  if (!g) { skMsg("Три точки на одной прямой — дуги нет", true); return; }
  skCommit(() => {
    const a = skResolve(aPos), b = skResolve(bPos), c = skAddPoint(g.x, g.y);
    const ccw = skArcOrder(aPos, bPos, mPos, g);
    skData().arcs.push({id:skNewId("a"), c, p1:ccw ? a : b, p2:ccw ? b : a, construction:skConstruction});
  }, "Дуга конфликтует с зависимостями");
}
function skCommitPolygon(cPos, vPos, n) {
  n = Math.max(3, Math.min(24, Math.round(n || 6)));
  skCommit(() => {
    const d = skData();
    const r = Math.hypot(vPos.x - cPos.x, vPos.y - cPos.y);
    if (r < 0.2) return false;
    const c = skResolve(cPos);
    const a0 = Math.atan2(vPos.y - cPos.y, vPos.x - cPos.x);
    const circ = {id:skNewId("c"), c, r, construction:true};
    d.circles.push(circ);
    const ids = [];
    for (let i = 0; i < n; i++) {
      const a = a0 + i * 2 * Math.PI / n;
      const id = skAddPoint(cPos.x + r * Math.cos(a), cPos.y + r * Math.sin(a));
      ids.push(id); d.cons.push({id:skNewId("k"), type:"onCircle", p:id, c:circ.id});
    }
    const ls = ids.map((id, i) => skAddLine(id, ids[(i + 1) % n], skConstruction));
    for (let i = 1; i < n; i++) d.cons.push({id:skNewId("k"), type:"equal", e1:ls[0], e2:ls[i]});
  }, "Многоугольник конфликтует с зависимостями");
}

// ── mouse ──
function skMouseDown(e, p) {
  const sx = e.offsetX, sy = e.offsetY;
  if (skTool === "select" || skTool.startsWith("con:")) {
    if (skTool.startsWith("con:")) {
      const key = skHitAt(sx, sy, {virtual:true, noBoxes:true});
      if (!key) return;
      const type = skTool.slice(4);
      if (type === "fix") { skToggleFix([key]); skTool = "select"; $("hint").textContent = SK_HINTS.select; return; }
      if (!skSel.includes(key)) skSel.push(key);
      const made = skMakeConstraint(type, skSel);
      if (Array.isArray(made)) {
        skCommit(() => { skData().cons.push(...made); });
        skSel = []; skTool = "select"; $("hint").textContent = SK_HINTS.select;
      } else if (typeof made === "string") { skMsg(made, true); skSel = []; }
      changed(); return;
    }
    const key = skHitAt(sx, sy);
    if (key && skKind(key) === "k") {
      const k = skData().cons.find(c => c.id === skKid(key));
      if (e.shiftKey) { if (!skSel.includes(key)) skSel.push(key); }
      else skSel = [key];
      if (k && k.dim) { pushUndo(); skDrag = {kind:"dimLabel", k, moved:false}; }
      changed(); return;
    }
    if (key) {
      if (e.shiftKey) { skSel = skSel.includes(key) ? skSel.filter(x => x !== key) : skSel.concat([key]); changed(); return; }
      if (!skSel.includes(key)) skSel = [key];
      const I = skIndex(), t = skKind(key), id = skKid(key);
      let ptIds = [];
      if (t === "p") ptIds = [id];
      else if (t === "l") { const l = I.L.get(id); ptIds = [l.p1, l.p2]; }
      else if (t === "c") ptIds = [I.C.get(id).c];
      else if (t === "a") { const a = I.C.get(id); ptIds = [a.c, a.p1, a.p2]; }
      const pts = ptIds.map(i => I.P.get(i)).filter(q => q && !q.virtual);
      pushUndo();
      skDrag = {kind:"geom", start:p, pts:pts.map(q => ({id:q.id, x:q.x, y:q.y})), circle:t === "c" ? id : null, moved:false};
      changed(); return;
    }
    if (!e.shiftKey) skSel = [];
    skDrag = {kind:"box", a:p, b:p};
    changed(); return;
  }
  const s = skSnapNow || skSnap(p, skDraft && skDraft.base);
  const pos = skPos(s);
  const typed = hudValues();
  switch (skTool) {
    case "line":
      if (!skDraft) { skDraft = {tool:"line", start:pos, base:pos, firstId:null}; skShowLineHud(sx, sy); }
      else skLineClick(pos, typed);
      break;
    case "rect": case "crect":
      if (!skDraft) {
        skDraft = {tool:skTool, a:pos, base:pos};
        hudShow([{key:"w", label:"ширина"}, {key:"h", label:"высота"}], sx, sy, v => {
          const D = skDraft; if (!D) return; hudHide(); skDraft = null; skCommitRect(D.a, D.cur || D.a, v, D.tool === "crect"); });
      } else { const D = skDraft; skDraft = null; hudHide(); skCommitRect(D.a, pos, typed, D.tool === "crect"); }
      break;
    case "circle":
      if (!skDraft) {
        skDraft = {tool:"circle", c:pos, base:pos};
        hudShow([{key:"d", label:"диаметр"}], sx, sy, v => { const D = skDraft; if (!D) return; hudHide(); skDraft = null; skCommitCircle(D.c, D.cur || D.c, v); });
      } else { const D = skDraft; skDraft = null; hudHide(); skCommitCircle(D.c, pos, typed); }
      break;
    case "arc":
      if (!skDraft) skDraft = {tool:"arc", pts:[pos], base:pos};
      else if (skDraft.pts.length === 1) skDraft.pts.push(pos);
      else { const D = skDraft; skDraft = null; skCommitArc(D.pts[0], D.pts[1], pos); }
      break;
    case "polygon":
      if (!skDraft) {
        skDraft = {tool:"polygon", c:pos, base:pos};
        hudShow([{key:"n", label:"сторон", value:6}], sx, sy, v => {
          const D = skDraft; if (!D || !D.cur) return; hudHide(); skDraft = null; skCommitPolygon(D.c, D.cur, Number.isFinite(v.n) ? v.n : 6); });
      } else {
        const n = hud ? evalNum(hud.fields[0].input.value) : 6, D = skDraft;
        skDraft = null; hudHide(); skCommitPolygon(D.c, pos, Number.isFinite(n) ? n : 6);
      }
      break;
    case "point":
      if (s.kind === "point" && skData().points.some(q => q.id === s.ref)) break;
      skCommit(() => { const id = skResolve(pos); const q = skData().points.find(x => x.id === id); if (q) q.standalone = true; });
      break;
    case "dim": {
      const key = skHitAt(sx, sy, {virtual:true, noBoxes:true});
      if (!skDraft) {
        if (!key) return;
        skDraft = {tool:"dim", refs:[key], cur:p};
      } else if (key && !skDraft.refs.includes(key) && skDraft.refs.length < 2
          && !["c", "a"].includes(skKind(skDraft.refs[0])) && !["c", "a"].includes(skKind(key))) {
        skDraft.refs.push(key);
      } else {
        const spec = skDimSpec(skDraft.refs, p);
        skDraft = null;
        if (spec) skPlaceDim(spec); else skMsg("Для этих элементов размера нет", true);
      }
      draw();
      break;
    }
    case "trim": { const plan = skTrimPlan(sx, sy); if (plan) skTrim(plan); skDraftTrim = null; break; }
    case "fillet": {
      const corner = skFilletCorner(p);
      if (!corner) { skMsg("Кликните ближе к углу, где сходятся две линии", true); return; }
      skDraft = {tool:"fillet", corner, r:5};
      hudShow([{key:"r", label:"радиус", value:5}], sx, sy, v => {
        const D = skDraft; if (!D) return; const r = Number.isFinite(v.r) ? v.r : D.r; hudHide(); skDraft = null; skFillet(D.corner, r); },
        () => { const v = hudValues(); if (Number.isFinite(v.r) && skDraft) skDraft.r = v.r; });
      const f = hud.fields[0].input; f.dataset.locked = "1"; setTimeout(() => { f.focus(); f.select(); }, 0);
      break;
    }
    case "offset": {
      const key = skHitAt(sx, sy, {noBoxes:true});
      if (!key || !["l", "c", "a"].includes(skKind(key))) { skMsg("Кликните линию или окружность", true); return; }
      const I = skIndex(); let side;
      if (skKind(key) === "l") { const [a, b] = skLP(I, skKid(key)); side = skSdist(a, b, p) >= 0 ? 1 : -1; }
      else { const g = skCirc(I, skKid(key)); side = Math.hypot(p.x - g.c.x, p.y - g.c.y) >= g.r ? 1 : -1; }
      skDraft = {tool:"offset", key, side, dist:10};
      hudShow([{key:"d", label:"расстояние", value:10}], sx, sy, v => {
        const D = skDraft; if (!D) return; const dd = Number.isFinite(v.d) ? v.d : D.dist; hudHide(); skDraft = null; skOffset(D.key, dd, D.side); },
        () => { const v = hudValues(); if (Number.isFinite(v.d) && skDraft) skDraft.dist = v.d; });
      const f = hud.fields[0].input; f.dataset.locked = "1"; setTimeout(() => { f.focus(); f.select(); }, 0);
      break;
    }
    case "mirror": {
      const key = skHitAt(sx, sy, {virtual:true, noBoxes:true});
      if (!key || skKind(key) !== "l") { skMsg("Кликните линию — ось симметрии", true); return; }
      skMirror(skKid(key)); skSetTool("select");
      break;
    }
  }
  draw();
}
function skShowLineHud(sx, sy) {
  hudShow([{key:"len", label:"длина"}, {key:"ang", label:"угол°"}], sx, sy,
    v => { if (skDraft && skDraft.tool === "line") skLineClick(skDraft.cur || skDraft.start, v); });
}
function skLineEnd(startPos, cur, typed) {
  if (Number.isFinite(typed.len) || Number.isFinite(typed.ang)) {
    const ang = Number.isFinite(typed.ang) ? rad(typed.ang) : Math.atan2(cur.y - startPos.y, cur.x - startPos.x);
    const len = Number.isFinite(typed.len) ? typed.len : Math.hypot(cur.x - startPos.x, cur.y - startPos.y);
    const x = startPos.x + len * Math.cos(ang), y = startPos.y + len * Math.sin(ang);
    return {x, y, snap:{kind:"grid", x, y}};
  }
  return cur;
}
function skLineClick(pos, typed) {
  const D = skDraft;
  const end = skLineEnd(D.start, pos, typed);
  const closing = end.snap && end.snap.kind === "point" && D.firstId && end.snap.ref === D.firstId;
  const ids = skCommitLine(D.start, end, typed);
  if (!ids || skDraft !== D) return;
  if (!D.firstId) D.firstId = ids.a;
  if (closing) { skDraft = null; hudHide(); draw(); return; }
  const q = skData().points.find(x => x.id === ids.b);
  if (!q) { skDraft = null; hudHide(); return; }
  D.start = {x:q.x, y:q.y, snap:{kind:"point", ref:ids.b, x:q.x, y:q.y}};
  D.base = D.start;
  const s = toScreen(q);
  hudHide(); skShowLineHud(s.x, s.y);
  draw();
}
function skMouseMove(e, p) {
  const sx = e.offsetX, sy = e.offsetY;
  if (skDrag) {
    if (skDrag.kind === "box") { skDrag.b = p; draw(); return; }
    if (skDrag.kind === "dimLabel") {
      const k = skDrag.k, I = skIndex();
      if (k.type === "dist") { const a = I.P.get(k.a), b = I.P.get(k.b), u = skUnit(a, b), m = {x:(a.x + b.x) / 2, y:(a.y + b.y) / 2};
        k.off = (p.x - m.x) * -u.y + (p.y - m.y) * u.x; }
      else if (k.type === "hdist") { const a = I.P.get(k.a), b = I.P.get(k.b); k.off = p.y - (a.y + b.y) / 2; }
      else if (k.type === "vdist") { const a = I.P.get(k.a), b = I.P.get(k.b); k.off = p.x - (a.x + b.x) / 2; }
      else if (k.type === "radius" || k.type === "diameter") { const g = skCirc(I, k.c); k.ang = Math.atan2(p.y - g.c.y, p.x - g.c.x); k.off = Math.max(1, Math.hypot(p.x - g.c.x, p.y - g.c.y) - g.r); }
      else if (k.type === "pointLine") { const [a, b] = skLP(I, k.l), u = skUnit(a, b), q = I.P.get(k.p); k.off = (p.x - q.x) * u.x + (p.y - q.y) * u.y; }
      else if (k.type === "angle") { const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2), X = skLineX(a, b, c, d); if (X) k.off = Math.max(3, Math.hypot(p.x - X.x, p.y - X.y)); }
      skDrag.moved = true; draw(); return;
    }
    if (skDrag.kind === "geom") {
      const dx = p.x - skDrag.start.x, dy = p.y - skDrag.start.y;
      let soft = skDrag.pts.map(q => ({id:q.id, x:q.x + dx, y:q.y + dy, w:0.05}));
      if (skDrag.circle) {
        // dragging a circle changes its radius (the center stays put)
        const c = skData().circles.find(x => x.id === skDrag.circle), ctr = skIndex().P.get(c.c);
        if (!c.fixedR) c.r = Math.max(0.2, Math.hypot(p.x - ctr.x, p.y - ctr.y));
        soft = [];
      }
      skSolve(soft, 12);
      skSolve(null, 8);
      skDrag.moved = true;
      changed(false, true);
      return;
    }
  }
  const base = skDraft && (skDraft.start || skDraft.a || skDraft.c || (skDraft.pts && skDraft.pts[skDraft.pts.length - 1]));
  if (["line", "rect", "crect", "circle", "arc", "polygon", "point"].includes(skTool)) {
    skSnapNow = skSnap(p, base);
    if (skDraft) {
      skDraft.cur = skPos(skSnapNow);
      hudMove(sx, sy);
      const D = skDraft, c = D.cur;
      if (D.tool === "line") {
        hudSet("len", Math.hypot(c.x - D.start.x, c.y - D.start.y));
        hudSet("ang", Math.atan2(c.y - D.start.y, c.x - D.start.x) * 180 / Math.PI);
      } else if (D.tool === "rect") { hudSet("w", Math.abs(c.x - D.a.x)); hudSet("h", Math.abs(c.y - D.a.y)); }
      else if (D.tool === "crect") { hudSet("w", 2 * Math.abs(c.x - D.a.x)); hudSet("h", 2 * Math.abs(c.y - D.a.y)); }
      else if (D.tool === "circle") hudSet("d", 2 * Math.hypot(c.x - D.c.x, c.y - D.c.y));
    }
  } else {
    skSnapNow = null;
    if (skTool === "dim" && skDraft) skDraft.cur = p;
  }
  const virt = skTool === "dim" || skTool === "mirror" || skTool.startsWith("con:");
  skHover = skTool === "trim" ? null : skHitAt(sx, sy, {virtual:virt, noBoxes:virt});
  skDraftTrim = skTool === "trim" ? skTrimPlan(sx, sy) : null;
  draw();
}
function skMouseUp() {
  if (!skDrag) return;
  const dragK = skDrag;
  skDrag = null;
  if (dragK.kind === "box") {
    const r = rectFrom(dragK.a, dragK.b);
    if (r.w > 0.5 || r.h > 0.5) {
      const I = skIndex(), inside = q => q.x >= r.x && q.x <= r.x + r.w && q.y >= r.y && q.y <= r.y + r.h;
      const add = [];
      for (const l of skData().lines) { const [a, b] = skLP(I, l.id); if (inside(a) && inside(b)) add.push("l:" + l.id); }
      for (const c of skData().circles) { const g = skCirc(I, c.id); if (inside({x:g.c.x - g.r, y:g.c.y - g.r}) && inside({x:g.c.x + g.r, y:g.c.y + g.r})) add.push("c:" + c.id); }
      for (const a of skData().arcs) { const pts = [a.p1, a.p2].map(id => I.P.get(id)); if (pts.every(inside)) add.push("a:" + a.id); }
      for (const q of skData().points) if (inside(q)) add.push("p:" + q.id);
      skSel = Array.from(new Set(skSel.concat(add)));
    }
    changed(); return;
  }
  if (!dragK.moved) { undoStack.pop(); changed(); return; }
  if (dragK.kind === "geom") skSolve(null, 60);
  skAnalyze(); changed();
}
function skDblClick(e) {
  const key = skHitAt(e.offsetX, e.offsetY);
  if (key && skKind(key) === "k") {
    const k = skData().cons.find(c => c.id === skKid(key));
    if (k && k.dim) { skSel = [key]; skEditDim(k.id); }
    return;
  }
  if (skTool === "line" && skDraft) { skDraft = null; hudHide(); draw(); }
}
function skKeyDown(e) {
  const k = e.key;
  if (hud && /^[0-9.\-]$/.test(k) && document.activeElement.tagName !== "INPUT") {
    const f = hud.fields[0].input;
    f.value = k; f.dataset.locked = "1"; f.focus();
    f.setSelectionRange(f.value.length, f.value.length);
    if (hud.onChange) hud.onChange();
    draw();
    e.preventDefault(); return true;
  }
  if (k === "Escape") {
    if (skDraft || hud || skTool.startsWith("con:")) skCancel();
    else if (skTool !== "select") skSetTool("select");
    else { skSel = []; changed(false); }
    e.preventDefault(); return true;
  }
  if (k === "Enter" && hud && hud.onEnter) { hud.onEnter(hudValues()); e.preventDefault(); return true; }
  if ((k === "Delete" || k === "Backspace") && skSel.length) {
    const keys = skSel.slice();
    skCommit(() => skDelete(keys)); skSel = []; changed(); e.preventDefault(); return true;
  }
  if ((e.ctrlKey || e.metaKey) && k.toLowerCase() === "z") { e.preventDefault(); skCancel(); undo(); skAnalyze(); changed(); return true; }
  if (e.ctrlKey || e.metaKey || e.altKey) return false;
  const map = {l:"line", r:"rect", c:"circle", a:"arc", p:"point", d:"dim", t:"trim", f:"fillet", o:"offset", m:"mirror", s:"select"};
  const lk = k.toLowerCase();
  if (map[lk]) { skSetTool(map[lk]); e.preventDefault(); return true; }
  if (lk === "x") { skToggleConstruction(); e.preventDefault(); return true; }
  return false;
}
function skToggleConstruction() {
  const keys = skSel.filter(x => ["l", "c", "a"].includes(skKind(x)) && !SK_VIRTUAL_KEY.test(x));
  if (!keys.length) {
    skConstruction = !skConstruction;
    $("skConstr").classList.toggle("on", skConstruction);
    skMsg(skConstruction ? "Новые линии — вспомогательные (не стены)" : "Новые линии — обычные (стены для робота)");
    return;
  }
  pushUndo();
  const I = skIndex();
  const objs = keys.map(x => skKind(x) === "l" ? I.L.get(skKid(x)) : I.C.get(skKid(x)));
  const target = !objs.every(o => o.construction);
  for (const o of objs) o.construction = target;
  changed();
}

// ── walls for the robot simulation ──
function skWallSegments() {
  const d = S.sketch;
  if (!d) return [];
  const I = skIndex(), segs = [];
  for (const l of d.lines || []) if (!l.construction) { const [a, b] = skLP(I, l.id); segs.push([a, b]); }
  const arcPts = (c, r, a1, sweep) => {
    const n = Math.max(8, Math.ceil(sweep / (Math.PI / 24))), pts = [];
    for (let i = 0; i <= n; i++) { const a = a1 + sweep * i / n; pts.push({x:c.x + r * Math.cos(a), y:c.y + r * Math.sin(a)}); }
    return pts;
  };
  const addPoly = pts => { for (let i = 0; i + 1 < pts.length; i++) segs.push([pts[i], pts[i + 1]]); };
  for (const c of d.circles || []) if (!c.construction) { const g = skCirc(I, c.id); addPoly(arcPts(g.c, g.r, 0, 2 * Math.PI)); }
  for (const a of d.arcs || []) if (!a.construction) { const g = skCirc(I, a.id), an = skArcAngles(I, a); addPoly(arcPts(g.c, g.r, an.a1, an.sweep)); }
  return segs;
}
function skRaySeg(p, d, a, b) {
  const ex = b.x - a.x, ey = b.y - a.y, den = d.x * ey - d.y * ex;
  if (Math.abs(den) < 1e-12) return Infinity;
  const t = ((a.x - p.x) * ey - (a.y - p.y) * ex) / den;
  const u = ((a.x - p.x) * d.y - (a.y - p.y) * d.x) / den;
  return t >= 0 && u >= -1e-9 && u <= 1 + 1e-9 ? t : Infinity;
}
function skSegHitsPoly(a, b, poly) {
  const inside = p => {
    let sign = 0;
    for (let i = 0; i < poly.length; i++) {
      const q = poly[i], r = poly[(i + 1) % poly.length];
      const c = (r.x - q.x) * (p.y - q.y) - (r.y - q.y) * (p.x - q.x);
      if (Math.abs(c) < 1e-9) continue;
      if (!sign) sign = Math.sign(c); else if (Math.sign(c) !== sign) return false;
    }
    return true;
  };
  if (inside(a) || inside(b)) return true;
  const cross = (o, p, q) => (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
  for (let i = 0; i < poly.length; i++) {
    const c = poly[i], d = poly[(i + 1) % poly.length];
    if (cross(a, b, c) * cross(a, b, d) < 0 && cross(c, d, a) * cross(c, d, b) < 0) return true;
  }
  return false;
}
function skSegCircle(a, b, c, r) {
  const dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy || 1e-12;
  const t = Math.max(0, Math.min(1, ((c.x - a.x) * dx + (c.y - a.y) * dy) / L2));
  return Math.hypot(a.x + dx * t - c.x, a.y + dy * t - c.y) < r;
}

// ── drawing ──
function skStrokeCurve(I, cv) {
  ctx.beginPath();
  if (cv.t === "l") { const a = toScreen(cv.a), b = toScreen(cv.b); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); }
  else if (cv.t === "c") { const c = toScreen(cv.c); ctx.arc(c.x, c.y, cv.r * view.scale, 0, 2 * Math.PI); }
  else { const c = toScreen(cv.c), an = skArcAngles(I, cv.obj); ctx.arc(c.x, c.y, cv.r * view.scale, -an.a1, -(an.a1 + an.sweep), true); }
  ctx.stroke();
}
function skRound(x, y, w, h, r) {
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(x, y, w, h, r); else ctx.rect(x, y, w, h);
}
function skTextBox(text, p, key, color) {
  const s = toScreen(p);
  ctx.font = "600 12px system-ui"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  const w = ctx.measureText(text).width + 10, h = 18;
  ctx.fillStyle = "rgba(19,11,36,.92)"; ctx.strokeStyle = color; ctx.lineWidth = 1;
  skRound(s.x - w / 2, s.y - h / 2, w, h, 5); ctx.fill(); ctx.stroke();
  ctx.fillStyle = color; ctx.fillText(text, s.x, s.y + 0.5);
  if (key) skBoxes.push({key, x:s.x - w / 2, y:s.y - h / 2, w, h});
}
function skSeg(a, b, color, w, dash) {
  const A = toScreen(a), B = toScreen(b);
  ctx.strokeStyle = color; ctx.lineWidth = w || 1; ctx.setLineDash(dash || []);
  ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke(); ctx.setLineDash([]);
}
function skArrow(at, dir, color) {
  const s = toScreen(at), d = {x:dir.x, y:-dir.y}, n = {x:-d.y, y:d.x}, L = 7, W = 3;
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.moveTo(s.x, s.y);
  ctx.lineTo(s.x - d.x * L + n.x * W, s.y - d.y * L + n.y * W);
  ctx.lineTo(s.x - d.x * L - n.x * W, s.y - d.y * L - n.y * W); ctx.closePath(); ctx.fill();
}
function skDrawDim(I, k, sel) {
  const color = sel ? "#fde68a" : k.driven ? "#958ab3" : "#e9d5ff";
  const val = k.driven ? skMeasure(I, k) : k.v;
  const unit = k.type === "angle" || k.type === "angleH" ? "°" : "";
  let text = (k.type === "radius" ? "R" : k.type === "diameter" ? "⌀" : "") + fmt(val) + unit;
  if (k.driven) text = "(" + text + ")";
  const key = "k:" + k.id, P = id => I.P.get(id);
  const line2 = (A, B) => { skSeg(A, B, color, 1); const u = skUnit(A, B); skArrow(B, u, color); skArrow(A, {x:-u.x, y:-u.y}, color); };
  if (k.type === "dist" || k.type === "hdist" || k.type === "vdist") {
    const a = P(k.a), b = P(k.b);
    let A2, B2;
    if (k.type === "dist") { const u = skUnit(a, b), n = {x:-u.y, y:u.x}, o = k.off || 4;
      A2 = {x:a.x + n.x * o, y:a.y + n.y * o}; B2 = {x:b.x + n.x * o, y:b.y + n.y * o}; }
    else if (k.type === "hdist") { const yd = (a.y + b.y) / 2 + (k.off || -4); A2 = {x:a.x, y:yd}; B2 = {x:b.x, y:yd}; }
    else { const xd = (a.x + b.x) / 2 + (k.off || 4); A2 = {x:xd, y:a.y}; B2 = {x:xd, y:b.y}; }
    skSeg(a, A2, color + "88", 1); skSeg(b, B2, color + "88", 1);
    line2(A2, B2);
    skTextBox(text, {x:(A2.x + B2.x) / 2, y:(A2.y + B2.y) / 2}, key, color);
  } else if (k.type === "pointLine") {
    const [a, b] = skLP(I, k.l), q = P(k.p), u = skUnit(a, b);
    const along = (q.x - a.x) * u.x + (q.y - a.y) * u.y, foot = {x:a.x + u.x * along, y:a.y + u.y * along};
    const o = k.off || 0, q2 = {x:q.x + u.x * o, y:q.y + u.y * o}, f2 = {x:foot.x + u.x * o, y:foot.y + u.y * o};
    if (o) { skSeg(q, q2, color + "88", 1); skSeg(foot, f2, color + "88", 1); }
    if (Math.hypot(q2.x - f2.x, q2.y - f2.y) > 1e-6) line2(f2, q2);
    skTextBox(text, {x:(q2.x + f2.x) / 2, y:(q2.y + f2.y) / 2}, key, color);
  } else if (k.type === "radius" || k.type === "diameter") {
    const g = skCirc(I, k.c), ang = k.ang !== undefined ? k.ang : 0.7, dir = {x:Math.cos(ang), y:Math.sin(ang)};
    const edge = {x:g.c.x + dir.x * g.r, y:g.c.y + dir.y * g.r};
    const start = k.type === "diameter" ? {x:g.c.x - dir.x * g.r, y:g.c.y - dir.y * g.r} : g.c;
    const lab = {x:g.c.x + dir.x * (g.r + (k.off || 4)), y:g.c.y + dir.y * (g.r + (k.off || 4))};
    skSeg(start, lab, color, 1); skArrow(edge, dir, color);
    if (k.type === "diameter") skArrow(start, {x:-dir.x, y:-dir.y}, color);
    skTextBox(text, lab, key, color);
  } else if (k.type === "angle") {
    const [a, b] = skLP(I, k.l1), [c, d] = skLP(I, k.l2), X = skLineX(a, b, c, d);
    if (!X) return;
    const u = {x:(b.x - a.x) * k.s1, y:(b.y - a.y) * k.s1}, v = {x:(d.x - c.x) * k.s2, y:(d.y - c.y) * k.s2};
    let a1 = Math.atan2(u.y, u.x), a2 = Math.atan2(v.y, v.x);
    if (sk2pi(a2 - a1) > Math.PI) [a1, a2] = [a2, a1];
    const R = k.off || 8, s = toScreen(X), sw = sk2pi(a2 - a1);
    ctx.strokeStyle = color; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(s.x, s.y, R * view.scale, -a1, -(a1 + sw), true); ctx.stroke();
    const am = a1 + sw / 2;
    skTextBox(text, {x:X.x + Math.cos(am) * (R + 3), y:X.y + Math.sin(am) * (R + 3)}, key, color);
  } else if (k.type === "angleH") {
    const [a, b] = skLP(I, k.l), s = toScreen(a), ang = Math.atan2(b.y - a.y, b.x - a.x), R = 7;
    skSeg(a, {x:a.x + R * 1.4, y:a.y}, color + "88", 1, [3, 3]);
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(s.x, s.y, R * view.scale, 0, -ang, ang > 0); ctx.stroke();
    skTextBox(text, {x:a.x + Math.cos(ang / 2) * (R + 4), y:a.y + Math.sin(ang / 2) * (R + 4)}, key, color);
  }
}
function skGlyphAnchor(I, k) {
  const lineMid = id => { const [a, b] = skLP(I, id); return {m:{x:(a.x + b.x) / 2, y:(a.y + b.y) / 2}, u:skUnit(a, b)}; };
  if (k.type === "tangentEnd") { const [a, b] = skLP(I, k.l); const p = I.P.get(k.p); return {key:"p:" + k.p, m:{x:p.x, y:p.y}, u:skUnit(a, b), point:true}; }
  if (k.l && !k.p && !k.a && (k.type === "horizontal" || k.type === "vertical" || k.type === "tangent")) return {key:"l:" + k.l, ...lineMid(k.l)};
  if (k.l1 && k.l2) return {key:"l:" + k.l1, ...lineMid(k.l1), pair:{key:"l:" + k.l2, ...lineMid(k.l2)}};
  if (k.type === "equal" && skIsLine(I, k.e1)) return {key:"l:" + k.e1, ...lineMid(k.e1), pair:{key:"l:" + k.e2, ...lineMid(k.e2)}};
  if (k.type === "equal" || k.type === "concentric" || k.type === "tangentCC") {
    const i1 = k.e1 || k.c1, i2 = k.e2 || k.c2, g1 = skCirc(I, i1), g2 = skCirc(I, i2);
    return {key:"c:" + i1, m:{x:g1.c.x + g1.r * 0.7, y:g1.c.y + g1.r * 0.7}, u:{x:1, y:0},
      pair:{key:"c:" + i2, m:{x:g2.c.x + g2.r * 0.7, y:g2.c.y + g2.r * 0.7}, u:{x:1, y:0}}};
  }
  const p = I.P.get(k.p || k.a);
  return p ? {key:"p:" + p.id, m:{x:p.x, y:p.y}, u:{x:1, y:0}, point:true} : null;
}
function drawSketch() {
  const d = skData();
  const has = d.lines.length || d.circles.length || d.arcs.length || d.points.length;
  skBoxes = [];
  if (!sketchMode && !has) return;
  const I = skIndex(), curves = skCurves(I, false), sel = new Set(skSel);
  if (!sketchMode) {
    for (const cv of curves) {
      if (cv.obj.construction) continue;
      ctx.strokeStyle = "#c4b5fd"; ctx.lineWidth = 2.5; ctx.setLineDash([]);
      skStrokeCurve(I, cv);
    }
    return;
  }
  const o = toScreen({x:0, y:0});
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#fb7185"; ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(o.x + 26, o.y); ctx.stroke();
  ctx.strokeStyle = "#6ee7b7"; ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(o.x, o.y - 26); ctx.stroke();
  if (skTool === "trim" && skDraftTrim) {
    const pl = skDraftTrim, cv = pl.cv;
    ctx.strokeStyle = "#fb7185"; ctx.lineWidth = 4; ctx.setLineDash([5, 4]);
    if (pl.whole) skStrokeCurve(I, cv);
    else if (cv.t === "l") {
      const at = t => ({x:cv.a.x + (cv.b.x - cv.a.x) * t, y:cv.a.y + (cv.b.y - cv.a.y) * t});
      const A = toScreen(at(pl.lo ? pl.lo.t : 0)), B = toScreen(at(pl.hi ? pl.hi.t : 1));
      ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
    } else {
      const base = cv.t === "c" ? 0 : skArcAngles(I, cv.obj).a1, c = toScreen(cv.c);
      const t0 = pl.lo ? pl.lo.t : 0, t1 = pl.hi ? pl.hi.t : pl.end;
      const sweep = sk2pi(t1 - t0) || pl.end;
      ctx.beginPath(); ctx.arc(c.x, c.y, cv.r * view.scale, -(base + t0), -(base + t0 + sweep), true); ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  for (const cv of curves) {
    const isSel = sel.has(cv.key), isHover = skHover === cv.key;
    ctx.setLineDash(cv.obj.construction ? [7, 5] : []);
    ctx.strokeStyle = isSel ? "#fde68a" : cv.obj.construction ? "#fb923c" : skCurveFull(I, cv.key) ? "#f5f3ff" : "#a78bfa";
    ctx.lineWidth = isSel || isHover ? 3 : 2;
    skStrokeCurve(I, cv);
  }
  ctx.setLineDash([]);
  for (const vk of [skHover, ...skSel]) {
    if (!vk || !SK_VIRTUAL_KEY.test(vk)) continue;
    if (vk.startsWith("l:")) { const cv = skCurves(I, true).find(c => c.key === vk); if (cv) { ctx.strokeStyle = "#fde68a"; ctx.lineWidth = 3; skStrokeCurve(I, cv); } }
    else { const s = toScreen(I.P.get(skKid(vk))); ctx.fillStyle = "#fde68a"; ctx.fillRect(s.x - 4, s.y - 4, 8, 8); }
  }
  for (const p of d.points) {
    const s = toScreen(p), isSel = sel.has("p:" + p.id) || skHover === "p:" + p.id;
    ctx.fillStyle = isSel ? "#fde68a" : p.fixed ? "#6ee7b7" : skPointFree(p.id) ? "#a78bfa" : "#f5f3ff";
    const r = isSel ? 4.5 : 3;
    ctx.fillRect(s.x - r, s.y - r, 2 * r, 2 * r);
    if (p.fixed) { ctx.strokeStyle = "#6ee7b7"; ctx.lineWidth = 1; ctx.strokeRect(s.x - 6, s.y - 6, 12, 12); }
  }
  const stack = {};
  for (const k of d.cons) {
    if (k.dim || !SK_GLYPH[k.type]) continue;
    const an = skGlyphAnchor(I, k);
    if (!an) continue;
    const isSel = sel.has("k:" + k.id);
    const place = a => {
      const n = stack[a.key] = (stack[a.key] || 0) + 1;
      const s = toScreen(a.m);
      const nx = a.point ? 12 : -a.u.y * 14, ny = a.point ? -12 : -a.u.x * 14;
      const gx = s.x + nx + (n - 1) * 15 * (a.point ? 1 : a.u.x), gy = s.y + ny - (n - 1) * 15 * (a.point ? 0 : a.u.y);
      ctx.font = "11px system-ui"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillStyle = isSel ? "#fde68a" : "rgba(30,18,52,.95)";
      ctx.strokeStyle = isSel ? "#fde68a" : "#7c6ab0"; ctx.lineWidth = 1;
      skRound(gx - 7, gy - 7, 14, 14, 3); ctx.fill(); ctx.stroke();
      ctx.fillStyle = isSel ? "#120a22" : "#e9d5ff"; ctx.fillText(SK_GLYPH[k.type], gx, gy + 0.5);
      skBoxes.push({key:"k:" + k.id, x:gx - 7, y:gy - 7, w:14, h:14});
    };
    place(an);
    if (an.pair) place(an.pair);
  }
  for (const k of d.cons) if (k.dim) skDrawDim(I, k, sel.has("k:" + k.id));
  if (skTool === "dim" && skDraft) {
    for (const r of skDraft.refs) {
      const cv = skCurves(I, true).find(c => c.key === r);
      if (cv) { ctx.strokeStyle = "#fde68a"; ctx.lineWidth = 3; skStrokeCurve(I, cv); }
      else if (skKind(r) === "p") { const s = toScreen(I.P.get(skKid(r))); ctx.fillStyle = "#fde68a"; ctx.fillRect(s.x - 4, s.y - 4, 8, 8); }
    }
    const spec = skDraft.cur && skDimSpec(skDraft.refs, skDraft.cur);
    if (spec) { const n = skBoxes.length; skDrawDim(I, Object.assign({id:"preview", dim:true, driven:true}, spec), true); skBoxes.length = n; }
  }
  const D = skDraft, pv = "#fde68a";
  if (D && D.cur) {
    const t = hudValues();
    if (D.tool === "line") skSeg(D.start, skLineEnd(D.start, D.cur, t), pv, 2);
    else if (D.tool === "rect" || D.tool === "crect") {
      let a = D.a, b = D.cur;
      if (D.tool === "crect") {
        const w = Number.isFinite(t.w) ? t.w / 2 : Math.abs(b.x - a.x), h = Number.isFinite(t.h) ? t.h / 2 : Math.abs(b.y - a.y);
        const sx = b.x >= a.x ? 1 : -1, sy = b.y >= a.y ? 1 : -1;
        b = {x:D.a.x + sx * w, y:D.a.y + sy * h}; a = {x:D.a.x - sx * w, y:D.a.y - sy * h};
      } else {
        if (Number.isFinite(t.w)) b = {x:a.x + t.w * (b.x >= a.x ? 1 : -1), y:b.y};
        if (Number.isFinite(t.h)) b = {x:b.x, y:a.y + t.h * (b.y >= a.y ? 1 : -1)};
      }
      const cs = skRectCorners(a, b);
      for (let i = 0; i < 4; i++) skSeg(cs[i], cs[(i + 1) % 4], pv, 2);
    } else if (D.tool === "circle") {
      const r = Number.isFinite(t.d) ? t.d / 2 : Math.hypot(D.cur.x - D.c.x, D.cur.y - D.c.y), c = toScreen(D.c);
      ctx.strokeStyle = pv; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(c.x, c.y, r * view.scale, 0, 2 * Math.PI); ctx.stroke();
    } else if (D.tool === "arc") {
      if (D.pts.length === 1) skSeg(D.pts[0], D.cur, pv, 1, [4, 4]);
      else {
        const g = skCircle3(D.pts[0], D.pts[1], D.cur);
        if (g) {
          const ccw = skArcOrder(D.pts[0], D.pts[1], D.cur, g), A = ccw ? D.pts[0] : D.pts[1], B = ccw ? D.pts[1] : D.pts[0];
          const a1 = Math.atan2(A.y - g.y, A.x - g.x), sw = sk2pi(Math.atan2(B.y - g.y, B.x - g.x) - a1), c = toScreen(g);
          ctx.strokeStyle = pv; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(c.x, c.y, g.r * view.scale, -a1, -(a1 + sw), true); ctx.stroke();
        }
      }
    } else if (D.tool === "polygon") {
      const n = hud ? Math.max(3, Math.min(24, Math.round(evalNum(hud.fields[0].input.value) || 6))) : 6;
      const r = Math.hypot(D.cur.x - D.c.x, D.cur.y - D.c.y), a0 = Math.atan2(D.cur.y - D.c.y, D.cur.x - D.c.x);
      const pts = Array.from({length:n}, (_, i) => ({x:D.c.x + r * Math.cos(a0 + i * 2 * Math.PI / n), y:D.c.y + r * Math.sin(a0 + i * 2 * Math.PI / n)}));
      for (let i = 0; i < n; i++) skSeg(pts[i], pts[(i + 1) % n], pv, 2);
    }
  }
  if (D && D.tool === "fillet") {
    const g = skFilletGeom(D.corner, D.r);
    if (g) {
      const c = toScreen(g.C), a1 = Math.atan2(g.T1.y - g.C.y, g.T1.x - g.C.x), a2 = Math.atan2(g.T2.y - g.C.y, g.T2.x - g.C.x);
      const ccw = sk2pi(a2 - a1) < Math.PI, s0 = ccw ? a1 : a2, sw = ccw ? sk2pi(a2 - a1) : sk2pi(a1 - a2);
      ctx.strokeStyle = pv; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(c.x, c.y, g.r * view.scale, -s0, -(s0 + sw), true); ctx.stroke();
    }
  }
  if (D && D.tool === "offset") {
    const cv = skCurves(I, false).find(c => c.key === D.key);
    if (cv && cv.t === "l") {
      const u = skUnit(cv.a, cv.b), n = {x:u.y * D.side, y:-u.x * D.side};
      skSeg({x:cv.a.x + n.x * D.dist, y:cv.a.y + n.y * D.dist}, {x:cv.b.x + n.x * D.dist, y:cv.b.y + n.y * D.dist}, pv, 2, [5, 4]);
    } else if (cv) {
      const c = toScreen(cv.c); ctx.strokeStyle = pv; ctx.setLineDash([5, 4]); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(c.x, c.y, Math.max(0.5, cv.r + D.side * D.dist) * view.scale, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
    }
  }
  const sn = skSnapNow;
  if (sn && ["line", "rect", "crect", "circle", "arc", "polygon", "point"].includes(skTool)) {
    for (const g of sn.guides || []) skSeg(g.from, {x:sn.x, y:sn.y}, "#fde68a99", 1, [2, 4]);
    const s = toScreen(sn);
    ctx.strokeStyle = "#fde68a"; ctx.lineWidth = 1.5;
    ctx.beginPath();
    if (sn.kind === "point" || sn.kind === "pos") ctx.rect(s.x - 5, s.y - 5, 10, 10);
    else if (sn.kind === "mid") { ctx.moveTo(s.x, s.y - 6); ctx.lineTo(s.x + 6, s.y + 5); ctx.lineTo(s.x - 6, s.y + 5); ctx.closePath(); }
    else if (sn.kind === "int") { ctx.moveTo(s.x - 5, s.y - 5); ctx.lineTo(s.x + 5, s.y + 5); ctx.moveTo(s.x + 5, s.y - 5); ctx.lineTo(s.x - 5, s.y + 5); }
    else if (sn.kind === "on") ctx.arc(s.x, s.y, 5, 0, 2 * Math.PI);
    else { ctx.moveTo(s.x - 4, s.y); ctx.lineTo(s.x + 4, s.y); ctx.moveTo(s.x, s.y - 4); ctx.lineTo(s.x, s.y + 4); }
    ctx.stroke();
    if (sn.hv) label(sn.hv === "horizontal" ? "━" : "┃", {x:sn.x, y:sn.y + 12 / view.scale}, "#fde68a");
  }
  if (skDrag && skDrag.kind === "box") {
    const a = toScreen(skDrag.a), b = toScreen(skDrag.b);
    ctx.fillStyle = "rgba(167,139,250,.10)"; ctx.strokeStyle = "#a78bfa"; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
    ctx.fillRect(Math.min(a.x, b.x), Math.min(a.y, b.y), Math.abs(b.x - a.x), Math.abs(b.y - a.y));
    ctx.strokeRect(Math.min(a.x, b.x), Math.min(a.y, b.y), Math.abs(b.x - a.x), Math.abs(b.y - a.y));
    ctx.setLineDash([]);
  }
}

// ── side panel in sketch mode ──
function renderSketchPanel() {
  const panel = $("selPanel");
  panel.innerHTML = "<h2>Эскиз</h2>";
  const d = skData(), I = skIndex();
  const st = skDof === null ? "большой эскиз — степени свободы не считаются"
    : skDof === 0 ? "✓ полностью определён" : "степеней свободы: " + skDof + " (фиолетовое ещё можно двигать)";
  panel.insertAdjacentHTML("beforeend", '<div class="muted" style="color:' + (skDof === 0 ? "var(--ok)" : "var(--muted)") + '">' + st + "</div>");
  panel.insertAdjacentHTML("beforeend", '<div class="muted">Линии, окружности и дуги (кроме оранжевых вспомогательных) — стены для робота. Края поля и начало координат можно брать в размеры.</div>');
  const keys = skSel;
  if (!keys.length) {
    panel.insertAdjacentHTML("beforeend", '<div class="muted" style="margin-top:6px">Выделите элементы — здесь будут их зависимости и размеры.</div>');
    return;
  }
  const names = {p:"точка", l:"линия", c:"окружность", a:"дуга", k:"зависимость"};
  const count = {};
  for (const k of keys) count[skKind(k)] = (count[skKind(k)] || 0) + 1;
  panel.insertAdjacentHTML("beforeend", '<div class="row">' + Object.entries(count).map(([t, n]) => names[t] + (n > 1 ? " ×" + n : "")).join(", ") + "</div>");
  const ids = new Set(keys.map(skKid));
  const related = d.cons.filter(k => ids.has(k.id) || ["a", "b", "p", "l", "l1", "l2", "c", "c1", "c2", "e1", "e2"].some(f => k[f] && ids.has(k[f])));
  for (const k of related) {
    const r = document.createElement("div"); r.className = "row";
    const name = document.createElement("label");
    name.textContent = (SK_GLYPH[k.type] ? SK_GLYPH[k.type] + " " : "") + SK_NAMES[k.type] + (k.dim && k.driven ? " (справочный)" : "");
    r.appendChild(name);
    if (k.dim) {
      const inp = document.createElement("input"); inp.type = "text"; inp.style.width = "70px";
      inp.value = fmt(k.driven ? skMeasure(I, k) : k.v);
      inp.addEventListener("change", () => { const v = evalNum(inp.value); if (Number.isFinite(v)) skCommitDim(k, v, {v:k.v, driven:k.driven}); });
      r.appendChild(inp);
      r.appendChild(button(k.driven ? "сделать ведущим" : "справочный", () => {
        if (k.driven) skCommitDim(k, skMeasure(skIndex(), k), {v:k.v, driven:true});
        else { pushUndo(); k.driven = true; skAnalyze(); changed(); }
      }));
    }
    r.appendChild(button("✕", () => { skCommit(() => { skData().cons = skData().cons.filter(c => c.id !== k.id); }); }, "bad"));
    panel.appendChild(r);
  }
  const curvesSel = keys.filter(x => ["l", "c", "a"].includes(skKind(x)) && !SK_VIRTUAL_KEY.test(x));
  panel.append(row(
    ...(curvesSel.length ? [button("Вспомогательная (X)", skToggleConstruction)] : []),
    button("Фиксировать", () => skToggleFix(keys.filter(x => skKind(x) !== "k"))),
    button("Удалить", () => { const ks = skSel.slice(); skCommit(() => skDelete(ks)); skSel = []; changed(); }, "bad")));
}

// toolbar for the sketch mode
(function buildSketchBar() {
  const bar = $("skTools");
  for (const [k, name, key] of SK_TOOLS) {
    const b = button(name, () => skSetTool(k)); b.dataset.tool = k;
    if (key) b.title = name + " (" + key + ")";
    bar.appendChild(b);
  }
  bar.appendChild(Object.assign(document.createElement("span"), {className:"sep"}));
  for (const [k, sym, name] of SK_CONS) {
    const b = button(sym, () => skApplyConstraint(k), "con"); b.title = name; bar.appendChild(b);
  }
  bar.appendChild(Object.assign(document.createElement("span"), {className:"sep"}));
  const cb = button("Вспомогат. (X)", skToggleConstruction); cb.id = "skConstr"; cb.title = "Вспомогательная геометрия: не стена для робота";
  bar.appendChild(cb);
  $("modeSketch").addEventListener("click", () => enterSketch(!sketchMode));
})();

// ── background: falling comets at the sides ─────────────────────────────
(function sky() {
  const c = document.getElementById("sky"), g = c.getContext("2d");
  const still = window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches;
  let W = 0, H = 0, dpr = 1, stars = [], comets = [], last = 0;
  const DIR = rad(205);                     // falling down and a little left
  const dx = Math.sin(DIR), dy = -Math.cos(DIR);
  function sideX() {                        // only in the side bands
    const band = Math.max(90, W * 0.12);
    return Math.random() < 0.5 ? Math.random() * band : W - band + Math.random() * band + 60;
  }
  function comet(anywhere) {
    return {x:sideX(), y:anywhere ? Math.random() * H : -40 - Math.random() * H * 0.5,
      v:140 + Math.random() * 280, len:90 + Math.random() * 200, w:1.1 + Math.random() * 1.6,
      hue:255 + Math.random() * 65, a:0.65 + Math.random() * 0.35};
  }
  function resize() {
    dpr = window.devicePixelRatio || 1; W = innerWidth; H = innerHeight;
    c.width = W * dpr; c.height = H * dpr;
    stars = Array.from({length:Math.round(W * H / 9000)}, () => ({x:Math.random() * W,
      y:Math.random() * H, r:Math.random() * 1.1 + 0.2, p:Math.random() * 6.28}));
    comets = Array.from({length:Math.max(8, Math.round(H / 55))}, () => comet(true));
  }
  function frame(ts) {
    const dt = last ? Math.min(0.05, (ts - last) / 1000) : 0; last = ts;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const bg = g.createLinearGradient(0, 0, 0, H);
    bg.addColorStop(0, "#0d0818"); bg.addColorStop(1, "#07040d");
    g.fillStyle = bg; g.fillRect(0, 0, W, H);
    for (const [x, col] of [[0, "rgba(124,58,237,.16)"], [W, "rgba(192,38,211,.12)"]]) {
      const glow = g.createRadialGradient(x, H * 0.35, 0, x, H * 0.35, Math.max(W, H) * 0.45);
      glow.addColorStop(0, col); glow.addColorStop(1, "rgba(0,0,0,0)");
      g.fillStyle = glow; g.fillRect(0, 0, W, H);
    }
    for (const s of stars) {
      const tw = still ? 0.6 : 0.45 + 0.4 * Math.sin(ts / 900 + s.p);
      g.fillStyle = "rgba(221,214,254," + tw.toFixed(2) + ")";
      g.fillRect(s.x, s.y, s.r, s.r);
    }
    g.lineCap = "round";
    for (const k of comets) {
      if (!still) { k.x += dx * k.v * dt; k.y += dy * k.v * dt; }
      const tx = k.x - dx * k.len, ty = k.y - dy * k.len;
      const grad = g.createLinearGradient(k.x, k.y, tx, ty);
      grad.addColorStop(0, "hsla(" + k.hue + ",95%,88%," + k.a + ")");
      grad.addColorStop(0.25, "hsla(" + k.hue + ",90%,70%," + (k.a * 0.45) + ")");
      grad.addColorStop(1, "hsla(" + k.hue + ",90%,60%,0)");
      g.strokeStyle = grad;
      g.globalAlpha = 0.35; g.lineWidth = k.w * 4;      // soft glow
      g.beginPath(); g.moveTo(k.x, k.y); g.lineTo(tx, ty); g.stroke();
      g.globalAlpha = 1; g.lineWidth = k.w;
      g.beginPath(); g.moveTo(k.x, k.y); g.lineTo(tx, ty); g.stroke();
      g.fillStyle = "hsla(" + k.hue + ",100%,92%," + k.a + ")";
      g.beginPath(); g.arc(k.x, k.y, k.w * 1.5, 0, 6.283); g.fill();
      if (k.y - k.len > H + 20 || k.x + k.len < -20) Object.assign(k, comet(false));
    }
    if (!still) requestAnimationFrame(frame);
  }
  addEventListener("resize", () => { resize(); if (still) frame(0); });
  resize(); requestAnimationFrame(frame);
})();

// ── 3D view: the field, the route and the robot model with its mechanisms ─
// three.js world: x = map x (cm), y = up, z = -map y. Robot model frame
// (from the 3MF converter): x right, y forward, z up, origin = turning center.
let is3d = false;
let v3 = null;              // renderer, scene, camera, controls, groups
let robotModel = null;      // {meta, pos, idx}
let mechManual = null;      // {lift, arm, claw} from the sliders, or null
let pick3d = false;
let picked3d = null;        // body name
let cam3d = "orbit";
const MECH_START = {lift:30, arm:80, claw:18};
const H3 = {wall:25, obstacle:15, podium:10, sudden:15, zone:0.3};
// local copies in vendor/ first (no internet needed), the CDN only as a fallback
const THREE_LIBS = [["/vendor/three.min.js", "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"],
  ["/vendor/OrbitControls.js", "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"]];

function loadScript(src) {
  return new Promise((ok, fail) => {
    const el = document.createElement("script");
    el.src = src; el.onload = ok;
    el.onerror = () => fail(new Error("не загрузилась библиотека 3D (нужен интернет): " + src));
    document.head.appendChild(el);
  });
}
async function loadFirst(srcs) {
  let last;
  for (const src of srcs) { try { await loadScript(src); return; } catch (e) { last = e; } }
  throw last;
}
async function ensureThree() {
  if (!window.THREE) await loadFirst(THREE_LIBS[0]);
  if (!THREE.OrbitControls) await loadFirst(THREE_LIBS[1]);
}
const toW = (x, y, h) => new THREE.Vector3(x, h || 0, -y);          // map cm -> world
const rToL = (x, y, z) => new THREE.Vector3(x, z, -y);              // robot frame -> local

async function enter3d(on) {
  if (on) {
    try { await ensureThree(); } catch (e) { status(e.message, true); return; }
    is3d = true;
    if (!v3) init3d();
    if (!robotModel) loadRobotModel(false);
  } else is3d = false;
  // crossfade 2D <-> 3D
  const v3el = $("view3d");
  if (is3d) { v3el.style.opacity = 0; v3el.style.display = "block"; requestAnimationFrame(() => { v3el.style.opacity = 1; }); }
  else { v3el.style.opacity = 0; setTimeout(() => { if (!is3d) v3el.style.display = "none"; }, 360); }
  $("cv").style.opacity = is3d ? 0 : 1;
  $("cv").style.pointerEvents = is3d ? "none" : "";
  $("mode3d").classList.toggle("on", is3d);
  $("mode3d").textContent = is3d ? "2D" : "3D";
  showEl($("sec3d"), is3d);
  if (is3d) { rebuild3d(); resize3d(); requestAnimationFrame(frame3d); }
}
function init3d() {
  const box = $("view3d");
  const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  box.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 1, 6000);
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xede9fe, 0x1a1030, 0.95));
  const sun = new THREE.DirectionalLight(0xffffff, 0.75);
  sun.position.set(200, 400, 150); scene.add(sun);
  const field = new THREE.Group(), robot = new THREE.Group();
  scene.add(field); scene.add(robot);
  v3 = {renderer, scene, camera, controls, field, robot, parts:null, meshes:[], objects:[]};
  resetCamera("orbit");
  new ResizeObserver(resize3d).observe(box);
  renderer.domElement.addEventListener("click", e => { if (pick3d) pickBody(e); });
}
function resize3d() {
  if (!v3) return;
  const r = $("view3d").getBoundingClientRect();
  if (r.width < 10 || r.height < 10) return;
  v3.renderer.setSize(r.width, r.height, false);
  v3.renderer.domElement.style.width = r.width + "px"; v3.renderer.domElement.style.height = r.height + "px";
  v3.camera.aspect = r.width / r.height; v3.camera.updateProjectionMatrix();
}
function resetCamera(mode) {
  cam3d = mode;
  for (const b of document.querySelectorAll("[data-cam]")) b.classList.toggle("on", b.dataset.cam === mode);
  if (!v3) return;
  const W = S.field.w, H = S.field.h, M = Math.max(W, H);
  v3.controls.enabled = mode !== "follow";
  if (mode === "top") { v3.camera.position.set(W / 2, M * 1.25, -H / 2 + 0.01); v3.controls.target.set(W / 2, 0, -H / 2); }
  else if (mode === "orbit") { v3.camera.position.set(W * 0.5, M * 0.75, M * 0.55); v3.controls.target.set(W / 2, 0, -H / 2); }
}

// ── scene content ──
function boxMesh(w, h, d, color, opacity) {
  return new THREE.Mesh(new THREE.BoxGeometry(w, h, d),
    new THREE.MeshStandardMaterial({color, roughness:0.8, metalness:0.05, transparent:(opacity || 1) < 1, opacity:opacity || 1}));
}
function rebuild3d() {
  if (!v3 || !is3d) return;
  const F = v3.field;
  while (F.children.length) { const c = F.children.pop(); c.traverse(o => { if (o.geometry) o.geometry.dispose(); }); }
  const W = S.field.w, H = S.field.h;
  const floor = boxMesh(W, 1, H, 0x150c28, 1); floor.position.set(W / 2, -0.5, -H / 2); F.add(floor);
  const grid = [];
  for (let x = 0; x <= W + 1e-6; x += 10) grid.push(x, 0.05, 0, x, 0.05, -H);
  for (let y = 0; y <= H + 1e-6; y += 10) grid.push(0, 0.05, -y, W, 0.05, -y);
  const gg = new THREE.BufferGeometry(); gg.setAttribute("position", new THREE.Float32BufferAttribute(grid, 3));
  F.add(new THREE.LineSegments(gg, new THREE.LineBasicMaterial({color:0x7c6ab0, transparent:true, opacity:0.25})));
  for (const [x, y, w, h] of [[0, -1, W, 1], [0, H, W, 1], [-1, 0, 1, H], [W, 0, 1, H]]) {
    const b = boxMesh(w, 6, h, 0x8b84a8, 1); b.position.set(x + w / 2, 3, -(y + h / 2)); F.add(b);
  }
  if (bgImage) {
    const tex = new THREE.Texture(bgImage); tex.needsUpdate = true;
    const pl = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshBasicMaterial({map:tex, transparent:true, opacity:S.bg.opacity}));
    pl.rotation.x = -Math.PI / 2; pl.position.set(W / 2, 0.1, -H / 2); F.add(pl);
  }
  for (const r of S.rects) {
    const h = H3[r.type] || 10;
    const b = boxMesh(r.w, h, r.h, new THREE.Color(TYPES[r.type].fill), r.type === "zone" ? 0.45 : 1);
    b.position.set(r.x + r.w / 2, h / 2, -(r.y + r.h / 2)); F.add(b);
  }
  for (const [a, c] of skWallSegments()) {
    const len = Math.hypot(c.x - a.x, c.y - a.y);
    if (len < 0.01) continue;
    const b = boxMesh(len, 20, 1.2, 0xc4b5fd, 1);
    b.position.set((a.x + c.x) / 2, 10, -(a.y + c.y) / 2);
    b.rotation.y = Math.atan2(c.y - a.y, c.x - a.x);
    F.add(b);
  }
  v3.objects = S.objects.map(o => {
    const pod = S.rects.find(r => r.type === "podium" && o.x >= r.x && o.x <= r.x + r.w && o.y >= r.y && o.y <= r.y + r.h);
    const base = pod ? H3.podium : 0;
    const m = boxMesh(5, 5, 5, new THREE.Color(o.color || "#f472b6"), 1);
    m.position.set(o.x, base + 2.5, -o.y); F.add(m);
    return {o, mesh:m, base};
  });
  if (sim) {
    const pts = [];
    for (const mv of sim.moves) pts.push(mv.from.x, 0.4, -mv.from.y, mv.to.x, 0.4, -mv.to.y);
    const lg = new THREE.BufferGeometry(); lg.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    F.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({color:0xa78bfa})));
    for (const dt of sim.detours || []) {
      const g = new THREE.BufferGeometry().setFromPoints(dt.pts.map(q => toW(q.x, q.y, 0.6)));
      const line = new THREE.Line(g, new THREE.LineDashedMaterial({color:0xfb923c, dashSize:4, gapSize:3}));
      line.computeLineDistances(); F.add(line);
    }
    S.route.forEach(wp => { const m = boxMesh(3, 0.6, 3, 0xa78bfa, 1); m.position.set(wp.x, 0.3, -wp.y); F.add(m); });
  }
  if (!v3.parts) buildRobot();
}
function buildRobot() {
  const R = v3.robot;
  while (R.children.length) R.remove(R.children[0]);
  v3.meshes = [];
  const lift = new THREE.Group(), armPivot = new THREE.Group(), arm = new THREE.Group();
  const clawL = new THREE.Group(), clawR = new THREE.Group();
  R.add(lift); lift.add(armPivot); armPivot.add(arm); arm.add(clawL); arm.add(clawR);
  const parts = {lift, armPivot, arm, clawL, clawR, wheels:[], held:null};
  if (robotModel) {
    const {meta, pos, idx} = robotModel, J = meta.joints;
    const pv = rToL(J.armPivot[0], J.armPivot[1], J.armPivot[2]);
    armPivot.position.copy(pv); arm.position.copy(pv.clone().multiplyScalar(-1));
    for (const b of meta.bodies) {
      const p = new Float32Array(b.vCount * 3);
      for (let k = 0; k < b.vCount; k++) {
        const o = (b.vOff + k) * 3;
        p[k * 3] = pos[o]; p[k * 3 + 1] = pos[o + 2]; p[k * 3 + 2] = -pos[o + 1];
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(p, 3));
      g.setIndex(new THREE.BufferAttribute(idx.subarray(b.iOff, b.iOff + b.iCount), 1));
      g.computeVertexNormals();
      const mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color:new THREE.Color(b.color), roughness:0.7, metalness:0.15}));
      mesh.userData.body = b.name;
      v3.meshes.push(mesh);
      if (b.group === "wheel") {
        const c = rToL((b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2);
        const piv = new THREE.Group(); piv.position.copy(c); mesh.position.copy(c.clone().multiplyScalar(-1));
        piv.add(mesh); R.add(piv);
        parts.wheels.push({piv, axis:new THREE.Vector3(c.x, 0, c.z).normalize()});   // kiwi: axle points to the center
      } else ({lift, arm, clawL, clawR}[b.group] || R).add(mesh);
    }
    const ref = meta.bodies.find(b => b.group === "clawL") || meta.bodies.find(b => b.group === "arm");
    if (ref) {
      const held = boxMesh(5, 5, 5, 0xf472b6, 1);
      held.position.copy(rToL(0, ref.max[1] - 2.5, ref.min[2] + 2.5)); held.visible = false;
      arm.add(held); parts.held = held;
    }
  } else {
    const body = boxMesh(S.robot.w, 12, S.robot.l, 0x6d28d9, 1); body.position.set(0, 9, 0); R.add(body);
    const col = boxMesh(10, 20, 4, 0x8b84a8, 1); col.position.set(0, 25, -S.robot.l / 2 + 4); lift.add(col);
    const claw = boxMesh(8, 4, 8, 0xf0abfc, 1); claw.position.set(0, 20, -S.robot.l / 2 - 3); arm.add(claw);
    armPivot.position.set(0, 25, -S.robot.l / 2 + 2); arm.position.set(0, -25, S.robot.l / 2 - 2);
  }
  v3.parts = parts;
}

// ── mechanisms over the timeline ──
const clampN = (v, a, b) => Math.max(a, Math.min(b, v));
function mechAfter(st, m) {
  if (st.code.startsWith("visionPickup")) return {lift:30, arm:10, claw:170};
  if (st.code.startsWith("putDownObject")) return {lift:300, arm:80, claw:18};
  return m;
}
function mechDuring(st, m, f) {
  const lerp = (a, b, t) => a + (b - a) * clampN(t, 0, 1);
  const seg = (t0, t1) => (f - t0) / (t1 - t0);
  if (st.code.startsWith("visionPickup")) {   // search, lower, grab, raise (like DriveMotor)
    if (f < 0.1) return {lift:lerp(m.lift, 30, seg(0, .1)), arm:lerp(m.arm, 80, seg(0, .1)), claw:lerp(m.claw, 18, seg(0, .1))};
    if (f < 0.35) return {lift:30, arm:80 + 65 * Math.sin(Math.PI * seg(.1, .35)), claw:18};
    if (f < 0.55) return {lift:lerp(30, 170, seg(.35, .55)), arm:80, claw:18};
    if (f < 0.7) return {lift:170, arm:80, claw:lerp(18, 170, seg(.55, .7))};
    if (f < 0.85) return {lift:lerp(170, 30, seg(.7, .85)), arm:80, claw:170};
    return {lift:30, arm:lerp(80, 10, seg(.85, 1)), claw:170};
  }
  if (st.code.startsWith("putDownObject")) {   // lift 300 -> arm 80 -> claw 18
    if (f < .35) return {lift:lerp(m.lift, 300, seg(0, .35)), arm:m.arm, claw:m.claw};
    if (f < .65) return {lift:300, arm:lerp(m.arm, 80, seg(.35, .65)), claw:m.claw};
    return {lift:300, arm:80, claw:lerp(m.claw, 18, seg(.65, 1))};
  }
  return m;
}
function timelineT() {
  if (!sim || !sim.steps.length) return 0;
  if (tlPlaying || (tlSel === null && tlT > 0)) return tlT;
  if (tlSel !== null) return tlSel + 1;
  return 0;
}
/** Robot pose, servo angles, carried object and wheel travel at time t. */
function stateAt(t) {
  const steps = sim ? sim.steps : [];
  let m = {...MECH_START}, travel = 0, holding = false;
  const taken = new Set(), placed = [];
  let pose = {x:S.start.x, y:S.start.y, b:S.start.b};
  const n = Math.min(steps.length, Math.ceil(t));
  for (let k = 0; k < n; k++) {
    const st = steps[k], f = Math.min(1, t - k);
    const a = st.before, b = st.after || st.before;
    pose = f >= 1 ? {...b} : lerpPose(a, b, f);
    travel += Math.hypot(pose.x - a.x, pose.y - a.y) + Math.abs(angDiff(pose.b, a.b)) * Math.PI / 180 * 15.8;
    m = f >= 1 ? mechAfter(st, m) : mechDuring(st, m, f);
    if (st.code.startsWith("visionPickup") && f >= 0.7 && v3 && v3.objects) {
      let best = null, bd = Infinity;
      for (const ob of v3.objects) {
        if (taken.has(ob)) continue;
        const d = Math.hypot(ob.o.x - a.x, ob.o.y - a.y);
        if (d < bd) { bd = d; best = ob; }
      }
      if (best && bd < 120) { taken.add(best); holding = best; }
    }
    if (st.code.startsWith("putDownObject") && f >= 0.95 && holding) {
      const fw = dirOf(a.b), front = (robotModel ? robotModel.meta.measured.front : S.robot.l / 2) + 4;
      placed.push({ob:holding, x:a.x + fw.x * front, y:a.y + fw.y * front});
      holding = false;
    }
  }
  if (mechManual) m = {...mechManual};
  return {pose, m, travel, taken, placed, holding};
}
function applyMech(st) {
  const P = v3.parts;
  if (!P) return;
  const J = robotModel ? robotModel.meta.joints
    : {liftModelDeg:30, liftCmPerDeg:0.05, armModelDeg:80, armSign:1, clawTravelCm:2, clawModelDeg:18};
  P.lift.position.y = -(st.m.lift - J.liftModelDeg) * J.liftCmPerDeg;
  P.armPivot.rotation.x = J.armSign * rad(J.armModelDeg - st.m.arm);
  const open = deg => (170 - deg) / (170 - 18);
  const shift = (open(J.clawModelDeg) - open(st.m.claw)) * J.clawTravelCm;
  P.clawL.position.x = shift; P.clawR.position.x = -shift;
  const wr = robotModel ? robotModel.meta.measured.wheelRadiusCm || 5 : 5;
  for (const w of P.wheels) w.piv.setRotationFromAxisAngle(w.axis, st.travel / wr);
  if (P.held) { P.held.visible = !!st.holding; if (st.holding) P.held.material.color.set(st.holding.o.color || "#f472b6"); }
  for (const ob of v3.objects || []) {
    const pl = st.placed.find(q => q.ob === ob);
    ob.mesh.visible = !st.taken.has(ob) || !!pl;
    if (pl) ob.mesh.position.set(pl.x, 2.5, -pl.y);
    else ob.mesh.position.set(ob.o.x, ob.base + 2.5, -ob.o.y);
  }
}
function frame3d() {
  if (!is3d || !v3) return;
  const st = stateAt(timelineT());
  v3.robot.position.set(st.pose.x, 0, -st.pose.y);
  v3.robot.rotation.y = -rad(st.pose.b);
  applyMech(st);
  if (cam3d === "follow") {
    const f = dirOf(st.pose.b);
    const target = new THREE.Vector3(st.pose.x, 15, -st.pose.y);
    const want = new THREE.Vector3(st.pose.x - f.x * 75, 55, -(st.pose.y - f.y * 75));
    v3.camera.position.lerp(want, 0.08);
    v3.controls.target.lerp(target, 0.15);
    v3.camera.lookAt(v3.controls.target);
  } else v3.controls.update();
  if (!mechManual) syncMechSliders(st.m);
  v3.renderer.render(v3.scene, v3.camera);
  requestAnimationFrame(frame3d);
}
function syncMechSliders(m) {
  for (const k of ["lift", "arm", "claw"]) {
    const el = $("mech_" + k);
    if (el && document.activeElement !== el) el.value = Math.round(m[k]);
    const lab = $("mechv_" + k); if (lab) lab.textContent = Math.round(m[k]) + "°";
  }
}

// ── model loading, joints editor ──
async function loadRobotModel(report) {
  try {
    const meta = await (await fetch("/robot_model.json", {cache:"no-store"})).json();
    if (meta.error) throw new Error(meta.error);
    const buf = await (await fetch("/robot_model.bin", {cache:"no-store"})).arrayBuffer();
    robotModel = {meta, pos:new Float32Array(buf, 0, meta.posCount), idx:new Uint32Array(buf, meta.posCount * 4, meta.idxCount)};
    if (v3) { v3.parts = null; rebuild3d(); }
    render3dPanel();
    if (report) status("Модель робота загружена: деталей " + meta.bodies.length);
  } catch (e) { if (report) status(e.message, true); render3dPanel(); }
}
function pickBody(e) {
  const r = v3.renderer.domElement.getBoundingClientRect();
  const ray = new THREE.Raycaster();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), v3.camera);
  const hit = ray.intersectObjects(v3.meshes, false)[0];
  for (const m of v3.meshes) m.material.emissive.set(0x000000);
  picked3d = hit ? hit.object.userData.body : null;
  if (hit) hit.object.material.emissive.set(0x5b21b6);
  render3dPanel();
}
const GROUP_NAMES = {base:"корпус (неподвижно)", lift:"лифт", arm:"рука (наклон клешни)", clawL:"клешня — левый палец",
  clawR:"клешня — правый палец", wheel:"колесо"};
function render3dPanel() {
  const box = $("model3dInfo");
  if (!box) return;
  if (!robotModel) { box.innerHTML = '<div class="muted">Модели нет — укажите путь к .3mf и нажмите «Загрузить». Пока показан упрощённый робот.</div>'; $("jointBox").style.display = "none"; return; }
  const M = robotModel.meta, cnt = {};
  for (const b of M.bodies) cnt[b.group] = (cnt[b.group] || 0) + 1;
  const ms = M.measured;
  box.innerHTML = '<div class="muted">Деталей: ' + M.bodies.length + " · " + Object.entries(cnt).map(([g, n]) => (GROUP_NAMES[g] || g).split(" ")[0] + " " + n).join(", ")
    + "<br>Из модели: " + ms.w + "×" + ms.l + " см, соники " + ms.sonicSide + " вбок / " + ms.sonicAlong + " вдоль, Sharp "
    + ms.sharpFront + " вперёд / " + ms.sharpSide + " вбок, колёса R" + ms.wheelRadiusCm + " на окружности " + ms.wheelCircleCm + " см</div>";
  $("jointBox").style.display = "";
  const J = M.joints;
  const set = (id, v) => { const el = $(id); if (document.activeElement !== el) el.value = v; };
  set("jLiftModel", J.liftModelDeg); set("jLiftCm", J.liftCmPerDeg); set("jArmModel", J.armModelDeg);
  set("jArmSign", J.armSign); set("jClaw", J.clawTravelCm); set("jPx", J.armPivot[0]); set("jPy", J.armPivot[1]); set("jPz", J.armPivot[2]);
  const sel = $("pickedGroup");
  $("pickedName").textContent = picked3d ? picked3d : (pick3d ? "кликните деталь на модели" : "—");
  sel.disabled = !picked3d;
  if (picked3d) { const b = M.bodies.find(x => x.name === picked3d); if (b) sel.value = b.group; }
}
function jointsChanged() {
  if (!robotModel) return;
  const J = robotModel.meta.joints, num = id => Number($(id).value);
  J.liftModelDeg = num("jLiftModel"); J.liftCmPerDeg = num("jLiftCm"); J.armModelDeg = num("jArmModel");
  J.armSign = num("jArmSign") >= 0 ? 1 : -1; J.clawTravelCm = num("jClaw");
  J.armPivot = [num("jPx"), num("jPy"), num("jPz")];
  v3.parts = null; rebuild3d();
}

$("mode3d").addEventListener("click", () => enter3d(!is3d));
for (const b of document.querySelectorAll("[data-cam]")) b.addEventListener("click", () => resetCamera(b.dataset.cam));
for (const k of ["lift", "arm", "claw"]) $("mech_" + k).addEventListener("input", () => {
  $("mechManual").checked = true;
  mechManual = {lift:Number($("mech_lift").value), arm:Number($("mech_arm").value), claw:Number($("mech_claw").value)};
  $("mechv_" + k).textContent = $("mech_" + k).value + "°";
});
$("mechManual").addEventListener("change", () => {
  mechManual = $("mechManual").checked
    ? {lift:Number($("mech_lift").value), arm:Number($("mech_arm").value), claw:Number($("mech_claw").value)} : null;
});
$("modelPath").value = localStorage.getItem("plannerModelPath") || "";
$("modelLoad").addEventListener("click", async () => {
  const path = $("modelPath").value.trim();
  try { localStorage.setItem("plannerModelPath", path); } catch (e) {}
  status("Конвертирую модель… (до минуты)");
  try { await api("/importModel", {path}); await loadRobotModel(true); }
  catch (e) { status(e.message, true); }
});
$("modelApply").addEventListener("click", () => {
  if (!robotModel) { status("Сначала загрузите модель", true); return; }
  const m = robotModel.meta.measured;
  pushUndo();
  Object.assign(S.robot, {w:m.w, l:m.l});
  for (const k of ["sonicSide", "sonicAlong", "sharpFront", "sharpSide"]) if (m[k] !== undefined) S.robot[k] = m[k];
  changed();
  status("Размеры и датчики взяты из модели");
});
for (const id of ["jLiftCm", "jLiftModel", "jArmModel", "jArmSign", "jPx", "jPy", "jPz", "jClaw"]) $(id).addEventListener("change", jointsChanged);
$("pickMode").addEventListener("change", () => { pick3d = $("pickMode").checked; if (!pick3d) picked3d = null; render3dPanel(); });
$("pickedGroup").addEventListener("change", () => {
  if (!robotModel || !picked3d) return;
  const b = robotModel.meta.bodies.find(x => x.name === picked3d);
  if (b) { b.group = $("pickedGroup").value; v3.parts = null; rebuild3d(); render3dPanel(); }
});
$("jointsSave").addEventListener("click", async () => {
  if (!robotModel) return;
  const groups = {};
  for (const b of robotModel.meta.bodies) groups[b.name] = b.group;
  try { await api("/saveJoints", {joints:robotModel.meta.joints, groups}); status("Механизмы сохранены"); }
  catch (e) { status(e.message, true); }
});

// ── timeline ────────────────────────────────────────────────────────────
let tlSel = null;          // selected step index
let tlT = 0;               // playback position: step index + fraction
let tlPlaying = false;
let tlLastFrame = 0;

function stepDuration(st) {
  if (st && st.code.startsWith("visionPickup")) return 6;
  if (st && st.code.startsWith("putDownObject")) return 4.5;
  if (!st || !st.before || !st.after) return 0.6;
  const d = Math.hypot(st.after.x - st.before.x, st.after.y - st.before.y);
  const a = Math.abs(angDiff(st.after.b, st.before.b));
  return Math.max(0.6, d / 40 + a / 90);
}
function lerpPose(a, b, t) {
  return {x:a.x + (b.x - a.x) * t, y:a.y + (b.y - a.y) * t, b:a.b + angDiff(b.b, a.b) * t};
}
function timelinePose() {
  if (!sim || !sim.steps.length) return null;
  if (tlPlaying || (tlSel === null && tlT > 0)) {
    const i = Math.min(sim.steps.length - 1, Math.floor(tlT));
    const st = sim.steps[i];
    const f = Math.min(1, tlT - i);
    return {index:i, pose:lerpPose(st.before, st.after, f)};
  }
  if (tlSel === null || !sim.steps[tlSel]) return null;
  const target = sim.steps[tlSel].after;
  if (tlAnim && tlAnim.sel === tlSel) {
    const t = clamp01((performance.now() - tlAnim.t0) / 280);
    if (t < 1) return {index:tlSel, pose:lerpPose(tlAnim.from, target, easeOut(t))};
    tlAnim = null;
  }
  return {index:tlSel, pose:target};
}
function stepView(st) {
  const src = st.src || {};
  const wp = S.route[src.wp];
  const c = st.code;
  const num = m => (c.match(/\(([-\d.]+)\)/) || [])[1];
  if (src.kind === "rotate" || c.startsWith("rotateTheRobot")) return {cls:"rotate", big:"↻ " + num() + "°"};
  const sm = c.match(/smooth(To|Via)\(([^)]*)\)/);
  if (sm) { const a = sm[2].split(",").map(x => x.trim());
    return {cls:"move", big:(sm[1] === "Via" ? "↝ " : "↗ ") + a[0] + "," + a[1] + " ↻" + a[2] + "°" + (c.startsWith("barrier") ? " · объезд" : "")}; }
  if (c.startsWith("setPose")) return {cls:"other", big:"⌖ " + c.slice(7, 8) + " = " + num()};
  if (c.startsWith("alignFrontSharp")) return {cls:"sonic", big:"⇕ Sharp " + num()};
  if (c.startsWith("alignLeftSonic")) return {cls:"sonic", big:"⇔ лев. " + num()};
  if (c.startsWith("alignRightSonic")) return {cls:"sonic", big:"⇔ прав. " + num()};
  if (src.kind === "move") {
    if (wp && wp.back) return {cls:"back", big:"↓ назад " + num() + " см"};
    if (wp && wp.sharp) return {cls:"move", big:"↑ до Sharp " + num()};
    return {cls:"move", big:"↑ " + num() + " см" + (wp && wp.barrier ? " · объезд" : "")};
  }
  if (c.startsWith("visionPickup")) return {cls:"pickup", big:"✋ взять"};
  if (c.startsWith("putDownObject")) return {cls:"place", big:"⬇ положить"};
  if (c.startsWith("goLeftSonic")) return {cls:"sonic", big:"← до стены " + num()};
  if (c.startsWith("goRightSonic")) return {cls:"sonic", big:"→ до стены " + num()};
  if (c.startsWith("goLeftWithRightSonic")) return {cls:"sonic", big:"← пока П≥" + num()};
  if (c.startsWith("goRigtWithLeftSonic")) return {cls:"sonic", big:"→ пока Л≥" + num()};
  if (c.startsWith("goBackDistance")) return {cls:"back", big:"↓ назад " + num()};
  if (c.startsWith("goForwardSharp")) return {cls:"move", big:"↑ Sharp " + num()};
  if (c.startsWith("advanceAfterDelay")) return {cls:"other", big:"⏱ " + num() + " с"};
  const fn = c.match(/^(\w+)\(/);
  return {cls:"other", big:fn ? "ƒ " + fn[1] : "код"};
}
function tlSelect(i, scroll) {
  const shown = timelinePose();
  if (shown && i !== null && i !== tlSel) { tlAnim = {from:shown.pose, sel:i, t0:performance.now()}; kick(320); }
  tlPlaying = false; $("tlPlay").textContent = "▶ Пуск";
  tlSel = i;
  if (i !== null && sim.steps[i]) {
    tlT = i + 1;
    const src = sim.steps[i].src;
    if (src && S.route[src.wp]) selected = {kind:"route", id:S.route[src.wp].id};
  }
  renderSelPanel(); renderTimeline(scroll !== false); draw();
}
let lastCardSel = null;
function renderTimeline(scroll) {
  setTimeout(() => { lastCardSel = tlSel; }, 0);
  const box = $("timeline");
  const keep = box.scrollLeft;
  box.innerHTML = "";
  const steps = sim ? sim.steps : [];
  $("tlScrub").max = String(steps.length);
  $("tlScrub").value = String(Math.min(tlT, steps.length));
  $("tlInfo").textContent = "шагов: " + steps.length;
  if (!steps.length) {
    box.insertAdjacentHTML("beforeend", '<div class="muted" style="align-self:center">Таймлайн пуст: инструментом «Маршрут» кликните точки на карте.</div>');
    return;
  }
  const first = Math.max(0, Math.round(Number(S.firstCase) || 0));
  const levels = {};
  for (const w of sim.warnings) {
    if (w.level === "ok" || !w.wp) continue;
    levels[w.wp] = levels[w.wp] === "bad" || w.level === "bad" ? "bad" : "warn";
  }
  let selCard = null, lastWp = null;
  steps.forEach((st, i) => {
    if (lastWp !== null && st.wp !== lastWp) box.appendChild(Object.assign(document.createElement("div"), {className:"wpGap"}));
    lastWp = st.wp;
    const v = stepView(st);
    const card = document.createElement("div");
    card.className = "card " + v.cls + (i === tlSel ? " sel" + (tlSel !== lastCardSel ? " just" : "") : "");
    card.innerHTML = '<div class="top"><span>case ' + (first + i) + '</span><span>т.' + st.wp + '</span></div>'
      + '<div class="big"></div><div class="code"></div>';
    card.querySelector(".big").textContent = v.big;
    card.querySelector(".code").textContent = st.code;
    const lv = st.src && st.src.kind === "move" ? levels[st.wp] : null;
    if (lv) card.insertAdjacentHTML("beforeend", '<span class="badge ' + (lv === "warn" ? "warn" : "") + '">!</span>');
    card.addEventListener("click", e => { if (e.target.closest(".ed")) return; if (i !== tlSel) tlSelect(i); });
    if (i === tlSel) { card.appendChild(stepEditor(st)); selCard = card; }
    box.appendChild(card);
  });
  box.scrollLeft = keep;
  if (selCard && scroll) selCard.scrollIntoView({block:"nearest", inline:"center"});
}
/** Inline editor for one timeline step: edits the map the step came from. */
function stepEditor(st) {
  const ed = document.createElement("div"); ed.className = "ed";
  const src = st.src || {};
  const wp = S.route[src.wp];
  if (!wp) return ed;
  const keepSel = () => { const i = tlSel; changed(); tlSel = Math.min(i, sim.steps.length - 1); renderTimeline(true); draw(); };
  if (src.kind === "move") {
    const dx = wp.x - st.before.x, dy = wp.y - st.before.y;
    const len = Math.hypot(dx, dy);
    const li = document.createElement("input"); li.type = "number"; li.step = "1"; li.value = fmt(len);
    li.title = "длина отрезка, см (точка сдвигается вдоль него)";
    li.addEventListener("change", () => {
      const L = Number(li.value); if (!Number.isFinite(L) || L <= 0 || len < 1e-6) return;
      pushUndo(); wp.x = r1(st.before.x + dx / len * L); wp.y = r1(st.before.y + dy / len * L); keepSel();
    });
    ed.append("см", li);
    for (const [key, text] of [["back","назад"],["barrier","barrier"],["sharp","Sharp"]]) {
      const l = document.createElement("label");
      const c = document.createElement("input"); c.type = "checkbox"; c.checked = !!wp[key];
      c.addEventListener("change", () => { pushUndo(); wp[key] = c.checked; keepSel(); });
      l.append(c, " " + text); ed.appendChild(l);
    }
    ed.appendChild(button("+ действие", () => { pushUndo(); wp.actions.unshift({type:"pickup", value:null}); keepSel(); }));
  } else if (src.kind === "rotate") {
    ed.insertAdjacentHTML("beforeend", '<span class="muted">поворот сам — к следующей точке</span>');
  } else if (src.kind === "correct") {
    ed.insertAdjacentHTML("beforeend", '<span class="muted">коррекция по датчикам — сама, по карте</span>');
    const l = document.createElement("label"), c = document.createElement("input");
    c.type = "checkbox"; c.checked = !!wp.align;
    c.addEventListener("change", () => { pushUndo(); wp.align = c.checked; keepSel(); });
    l.append(c, " в этой точке"); ed.appendChild(l);
  } else if (src.kind === "action") {
    const a = wp.actions[src.ai];
    const sel = document.createElement("select");
    for (const [k, t] of Object.entries(ACTIONS)) sel.add(new Option(t.name, k, false, k === a.type));
    sel.style.maxWidth = "150px";
    sel.addEventListener("change", () => { pushUndo(); a.type = sel.value; const d = ACTIONS[a.type];
      a.value = d.value && d.value !== "method" ? d.def : null; if (d.value === "method") initMethodAction(a); keepSel(); });
    ed.appendChild(sel);
    const d = ACTIONS[a.type];
    if (d.value === "method") { for (const x of methodEditor(a, keepSel)) ed.appendChild(x); }
    else if (d.value === "text") {
      const ti = document.createElement("input"); ti.type = "text"; ti.value = a.value || ""; ti.style.width = "140px";
      ti.addEventListener("change", () => { pushUndo(); a.value = ti.value; keepSel(); });
      ed.appendChild(ti);
    } else if (d.value) {
      const ni = document.createElement("input"); ni.type = "number"; ni.step = "0.5"; ni.value = a.value;
      ni.addEventListener("change", () => { const x = Number(ni.value); if (!Number.isFinite(x)) return; pushUndo(); a.value = x; keepSel(); });
      ed.appendChild(ni);
    }
    const moveAct = delta => {
      const j = src.ai + delta; if (j < 0 || j >= wp.actions.length) return;
      pushUndo(); [wp.actions[src.ai], wp.actions[j]] = [wp.actions[j], wp.actions[src.ai]];
      const i = tlSel + delta; changed(); tlSel = i; renderTimeline(true); draw();
    };
    ed.append(button("◀", () => moveAct(-1)), button("▶", () => moveAct(1)),
      button("+", () => { pushUndo(); wp.actions.splice(src.ai + 1, 0, {type:"pickup", value:null}); const i = tlSel + 1; changed(); tlSel = i; renderTimeline(true); draw(); }),
      button("✕", () => { pushUndo(); wp.actions.splice(src.ai, 1); keepSel(); }, "bad"));
  }
  return ed;
}
function tlFrame(ts) {
  if (!tlPlaying) return;
  const dt = tlLastFrame ? Math.min(0.1, (ts - tlLastFrame) / 1000) : 0;
  tlLastFrame = ts;
  const i = Math.min(sim.steps.length - 1, Math.floor(tlT));
  tlT += dt * Number($("tlSpeed").value) / stepDuration(sim.steps[i]);
  if (tlT >= sim.steps.length) { tlT = sim.steps.length; tlPlaying = false; $("tlPlay").textContent = "▶ Пуск"; }
  const cur = Math.min(sim.steps.length - 1, Math.floor(tlT));
  $("tlScrub").value = String(tlT);
  if (cur !== tlCurCard) { tlCurCard = cur; highlightPlaying(cur); }
  draw();
  if (tlPlaying) requestAnimationFrame(tlFrame);
}
let tlCurCard = -1;
function highlightPlaying(i) {
  const cards = $("timeline").querySelectorAll(".card");
  cards.forEach((c, k) => c.style.outline = k === i ? "2px solid #fde68a" : "");
  if (cards[i]) cards[i].scrollIntoView({block:"nearest", inline:"center"});
}
$("tlPlay").addEventListener("click", () => {
  if (!sim.steps.length) return;
  if (tlPlaying) { tlPlaying = false; $("tlPlay").textContent = "▶ Пуск"; return; }
  if (tlSel !== null) { tlT = tlSel; tlSel = null; renderTimeline(false); }
  if (tlT >= sim.steps.length) tlT = 0;
  tlPlaying = true; tlLastFrame = 0; tlCurCard = -1; $("tlPlay").textContent = "⏸ Пауза";
  requestAnimationFrame(tlFrame);
});
$("tlScrub").addEventListener("input", () => {
  tlPlaying = false; $("tlPlay").textContent = "▶ Пуск";
  if (tlSel !== null) { tlSel = null; renderTimeline(false); }
  tlT = Number($("tlScrub").value);
  const cur = Math.min(sim.steps.length - 1, Math.floor(tlT));
  highlightPlaying(cur); draw();
});
const tlStep = d => {
  if (!sim.steps.length) return;
  const base = tlSel !== null ? tlSel : Math.max(-1, Math.ceil(tlT) - 1);
  tlSelect(Math.max(0, Math.min(sim.steps.length - 1, base + d)));
};
$("tlPrev").addEventListener("click", () => tlStep(-1));
$("tlNext").addEventListener("click", () => tlStep(1));
window.addEventListener("keydown", e => {
  if (["INPUT", "TEXTAREA", "SELECT"].includes(e.target.tagName)) return;
  if (e.key === "ArrowRight") { e.preventDefault(); tlStep(1); }
  else if (e.key === "ArrowLeft") { e.preventDefault(); tlStep(-1); }
});
$("timeline").addEventListener("wheel", e => {
  // vertical mouse wheel scrolls the timeline sideways
  if (Math.abs(e.deltaY) > Math.abs(e.deltaX)) { $("timeline").scrollLeft += e.deltaY; e.preventDefault(); }
}, {passive:false});

// ── server ──────────────────────────────────────────────────────────────
function status(text, bad) {
  const s = $("status"); s.textContent = text; s.style.color = bad ? "var(--bad)" : "var(--muted)";
  s.classList.remove("flash"); void s.offsetWidth; s.classList.add("flash");
}
/** Shows/hides a panel with a short fade-in. */
function showEl(el, on, disp) {
  el.style.display = on ? (disp || "") : "none";
  if (on) { el.classList.remove("fadein"); void el.offsetWidth; el.classList.add("fadein"); }
}
async function api(path, body) {
  const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"},
    body:JSON.stringify(Object.assign({token:TOKEN}, body || {}))});
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
async function refreshList() {
  try {
    const data = await api("/list");
    const sel = $("mapList"); sel.length = 1;
    for (const n of data.names) sel.add(new Option(n, n));
  } catch (e) { status(e.message, true); }
}
$("save").addEventListener("click", async () => {
  try {
    const name = $("mapName").value.trim();
    const data = await api("/save", {name, map:S});
    status("Сохранено: " + data.path); refreshList();
  } catch (e) { status(e.message, true); }
});
$("saveCode").addEventListener("click", async () => {
  try {
    const data = await api("/saveCode", {name:$("mapName").value.trim(), code:$("code").value});
    status("Код сохранён: " + data.path);
  } catch (e) { status(e.message, true); }
});
$("mapList").addEventListener("change", async () => {
  const name = $("mapList").value; if (!name) return;
  try {
    const data = await api("/load", {name});
    pushUndo(); S = Object.assign(defaultMap(), data.map); $("mapName").value = name;
    selected = null; loadBg(); changed(); viewTouched = false; fitView(true); status("Открыто: " + name);
  } catch (e) { status(e.message, true); }
  $("mapList").value = "";
});
$("newMap").addEventListener("click", () => {
  if (!confirm("Начать новую карту? Несохранённое пропадёт (можно отменить Ctrl+Z).")) return;
  pushUndo(); S = defaultMap(); selected = null; bgImage = null; changed(); viewTouched = false; fitView(true);
});

// ── start ───────────────────────────────────────────────────────────────
const toolsBox = $("tools");
for (const [k, name] of TOOLS) {
  const b = button(name, () => { tool = k; measure = null; for (const x of toolsBox.children) x.classList.toggle("on", x === b);
    $("hint").textContent = HINTS[k]; draw(); });
  if (k === tool) b.classList.add("on");
  toolsBox.appendChild(b);
}
$("hint").textContent = HINTS[tool];
try { const saved = localStorage.getItem("fieldPlannerAutosave"); S = saved ? Object.assign(defaultMap(), JSON.parse(saved)) : defaultMap(); }
catch (e) { S = defaultMap(); }
for (const list of [S.rects, S.objects, S.route]) for (const el of list) if (!el.id) el.id = uid();
loadBg();
// Refit while the user has not zoomed/panned (the first layout may come
// after the start, then the map was drawn tiny in a corner).
new ResizeObserver(() => { if (viewTouched) draw(); else fitView(); }).observe($("stage"));
changed();
fitView();
refreshList();
api("/info").then(d => { projectDirDefault = d.projectDir; renderMethods(); }).catch(() => {});
</script>
</body>
</html>
"""


# ── finding route methods in the robot project ──────────────────────────

SIMPLE_TYPES = {"double", "int", "boolean", "String", "float", "long"}
LIFECYCLE = {"execute", "initialize", "end", "isFinished", "runsWhenDisabled"}
METHOD_RE = re.compile(
    r"(?P<mods>(?:\b(?:public|private|protected|static|final|synchronized)\s+)+)"
    r"(?P<ret>[\w<>\[\]]+)\s+(?P<name>\w+)\s*\((?P<params>[^)]*)\)\s*(?:throws[^{;]*)?\{")


def _strip_code(text):
    """Comments and string contents blanked out (same length), so braces
    and names inside them do not count."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i)); i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j])); i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(c + " " * (min(j, n) - i - 1) + (c if j < n else "")); i = j + 1
        else:
            out.append(c); i += 1
    return "".join(out)


def _body_end(code, open_at):
    depth = 0
    for k in range(open_at, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    return len(code) - 1


def _javadoc(text, start):
    """First sentence of the /** */ comment right before a method."""
    head = text[:start].rstrip()
    while head.endswith(")") or re.search(r"@\w+\s*$", head):   # annotations
        m = re.search(r"@\w+(\([^)]*\))?\s*$", head)
        if not m:
            break
        head = head[:m.start()].rstrip()
    if not head.endswith("*/"):
        return ""
    k = head.rfind("/**")
    if k < 0:
        return ""
    doc = re.sub(r"^\s*\*\s?", "", head[k + 3:-2], flags=re.M)
    doc = " ".join(doc.split())
    m = re.match(r"(.+?[.!?])(\s|$)", doc)
    return (m.group(1) if m else doc)[:160]


def scan_project(path):
    root = Path(path).expanduser()
    if not root.is_dir():
        raise ValueError("Папка не найдена: %s" % root)
    java_root = root / "src" / "main" / "java"
    if not java_root.is_dir():
        java_root = root
    files = []
    for f in sorted(java_root.rglob("*.java"))[:600]:
        try:
            text = f.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "stateAutomatic" in text and re.search(r"extends\s+\w*Command\w*", text):
            files.append((f, text))
    if not files:
        raise ValueError("В папке нет класса маршрута (CommandBase со stateAutomatic)")
    steps, junk, modifiers, names = [], [], [], []
    for f, text in files:
        code = _strip_code(text)
        cls = re.search(r"class\s+(\w+)", code)
        cls = cls.group(1) if cls else ""
        sw = re.search(r"switch\s*\(\s*stateAutomatic\s*\)\s*\{", code)
        switch_text = code[sw.end():_body_end(code, sw.end() - 1)] if sw else ""
        found = {}
        for m in METHOD_RE.finditer(code):
            body = code[m.end() - 1:_body_end(code, m.end() - 1) + 1]
            params = []
            for raw in [p.strip() for p in m.group("params").split(",") if p.strip()]:
                parts = raw.replace("final ", "").split()
                params.append({"type": parts[0], "name": parts[-1]} if len(parts) >= 2 else {"type": raw, "name": raw})
            found[m.group("name")] = {
                "name": m.group("name"), "mods": m.group("mods"), "ret": m.group("ret"),
                "params": params, "body": body, "doc": _javadoc(text, m.start()),
                "override": bool(re.search(r"@Override\s*$", code[:m.start()].rstrip() + " ")),
            }
        # A route step must move stateAutomatic on (itself or through a
        # method it calls); otherwise the route would hang on it.
        adv = {n for n, d in found.items() if re.search(r"stateAutomatic\s*(\+\+|\+=|=[^=])", d["body"])}
        changed_ = True
        while changed_:
            changed_ = False
            for n, d in found.items():
                if n not in adv and any(re.search(r"\b%s\s*\(" % a, d["body"]) for a in adv if a != n):
                    adv.add(n); changed_ = True
        for n, d in found.items():
            used = bool(re.search(r"\b%s\s*\(" % n, switch_text))
            reason = None
            if "public" not in d["mods"]:
                reason = "не public (служебный)"
            elif n in LIFECYCLE or d["override"]:
                reason = "часть команды WPILib"
            elif d["ret"] == cls and "return this" in d["body"]:
                modifiers.append(n + "()"); continue
            elif d["ret"] != "void":
                reason = "возвращает значение (геттер)"
            elif re.match(r"(get|set|is|reset)[A-Z_]", n) and n not in adv:
                reason = "геттер/сеттер/сброс"
            elif any(p["type"] not in SIMPLE_TYPES for p in d["params"]):
                reason = "параметры не числа/строки"
            elif n not in adv:
                reason = "не переходит к следующему шагу — маршрут на нём встанет"
            if reason:
                junk.append({"name": n, "reason": reason})
            elif n not in names:
                names.append(n)
                steps.append({"name": n, "params": [{"type": p["type"], "name": p["name"]} for p in d["params"]],
                              "doc": d["doc"], "used": used, "className": cls})
    steps.sort(key=lambda m: (not m["used"], m["name"].lower()))
    return {"methods": steps, "junk": junk, "modifiers": modifiers,
            "files": [os.path.relpath(f, root) for f, _ in files]}


# ── robot 3D model (3MF from Fusion) ─────────────────────────────────────

MODEL_JSON = "robot_model.json"
MODEL_BIN = "robot_model.bin"


def _decimate(verts, tris, cell):
    """Vertex clustering: vertices in one cell merge; flat triangles go."""
    import numpy as np
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    t = np.asarray(tris, dtype=np.int64).reshape(-1, 3)
    if not len(v) or not len(t):
        return v.astype(np.float32), t.astype(np.uint32)
    keys = np.floor(v / cell).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    nv = np.zeros((counts.size, 3))
    np.add.at(nv, inv, v)
    nv /= counts[:, None]
    nt = inv[t]
    ok = (nt[:, 0] != nt[:, 1]) & (nt[:, 1] != nt[:, 2]) & (nt[:, 0] != nt[:, 2])
    nt = nt[ok]
    if len(nt):
        nt = np.unique(nt, axis=0)
    return nv.astype(np.float32), nt.astype(np.uint32)


def convert_3mf(path):
    """3MF -> field_maps/robot_model.json (+ .bin): simplified meshes in the
    robot frame (cm; x to the right, y forward, z up from the floor, origin =
    turning center = middle of the three wheels), measured sensor positions
    and default mechanism groups (lift / arm / claw / wheels)."""
    import zipfile
    import numpy as np
    import xml.etree.ElementTree as ET
    src = Path(path).expanduser()
    if not src.is_file():
        raise ValueError("Файл не найден: %s" % src)
    with zipfile.ZipFile(src) as z:
        name = next((n for n in z.namelist() if n.lower().endswith(".model")), None)
        if not name:
            raise ValueError("В 3MF нет модели")
        stream = z.open(name)
        colors, bodies, cur, group = {}, [], None, None
        for ev, el in ET.iterparse(stream, events=("start", "end")):
            tag = el.tag.split("}")[-1]
            if ev == "start":
                if tag == "colorgroup":
                    group = []; colors[el.get("id")] = group
                elif tag == "object":
                    cur = {"name": el.get("name") or ("id" + el.get("id")), "pid": el.get("pid"),
                           "pindex": int(el.get("pindex") or 0), "v": [], "t": []}
                continue
            if tag == "color" and group is not None:
                group.append(el.get("color"))
            elif tag == "vertex" and cur is not None:
                cur["v"].extend((float(el.get("x")), float(el.get("y")), float(el.get("z"))))
                el.clear()
            elif tag == "triangle" and cur is not None:
                cur["t"].extend((int(el.get("v1")), int(el.get("v2")), int(el.get("v3"))))
                el.clear()
            elif tag == "object":
                if cur["v"]:
                    bodies.append(cur)
                cur = None
                el.clear()
    if not bodies:
        raise ValueError("В модели нет тел")
    for b in bodies:
        v = np.asarray(b["v"]).reshape(-1, 3)
        b["min"], b["max"] = v.min(0), v.max(0)
        cg = colors.get(b["pid"]) or []
        col = cg[b["pindex"]] if b["pindex"] < len(cg) else "#8B84A8FF"
        b["color"] = (col or "#8B84A8FF")[:7]
    zmin = min(b["min"][2] for b in bodies)
    # wheels: bodies that touch the floor and are wheel-sized
    wheels = [b for b in bodies if b["min"][2] <= zmin + 2 and b["max"][2] - b["min"][2] > 50]
    if len(wheels) < 3:
        raise ValueError("Не нашёл три колеса (тела, касающиеся пола)")
    wheels = sorted(wheels, key=lambda b: -len(b["v"]))[:3]
    wc = [((b["min"] + b["max"]) / 2) for b in wheels]
    center = sum(wc) / 3
    sonics = [b for b in bodies if "ultrasonic" in b["name"].lower() or "sonic" in b["name"].lower()]
    # rear wheel: the one the side ultrasonics sit on (else the first)
    if sonics:
        sc = sum(((s["min"] + s["max"]) / 2) for s in sonics) / len(sonics)
        rear = min(range(3), key=lambda k: np.hypot(*(wc[k][:2] - sc[:2])))
    else:
        rear = 0
    f = center[:2] - wc[rear][:2]
    f = f / (np.hypot(*f) or 1)
    right = np.array([f[1], -f[0]])

    def to_robot(v):
        d = v[:, :2] - center[:2]
        return np.stack([d @ right, d @ f, v[:, 2] - zmin], axis=1) / 10.0

    def box_r(b):
        corners = np.array([[x, y, z] for x in (b["min"][0], b["max"][0])
                            for y in (b["min"][1], b["max"][1]) for z in (b["min"][2], b["max"][2])])
        r = to_robot(corners)
        return r.min(0), r.max(0)

    out_bodies, pos_parts, idx_parts, voff, ioff = [], [], [], 0, 0
    wheel_ids = {id(b) for b in wheels}
    for b in bodies:
        v = np.asarray(b["v"]).reshape(-1, 3)
        diag = float(np.linalg.norm(b["max"] - b["min"]))
        nv, nt = _decimate(v, b["t"], max(0.6, diag / 70.0))
        if not len(nt):
            continue
        rv = to_robot(nv.astype(np.float64)).astype(np.float32)
        lo, hi = box_r(b)
        out_bodies.append({"name": b["name"], "color": b["color"], "vOff": voff, "vCount": int(len(rv)),
                           "iOff": ioff, "iCount": int(nt.size), "min": [round(float(x), 2) for x in lo],
                           "max": [round(float(x), 2) for x in hi], "wheel": id(b) in wheel_ids})
        pos_parts.append(rv.reshape(-1)); idx_parts.append(nt.reshape(-1))
        voff += len(rv); ioff += nt.size
    sharps = [ob for ob in out_bodies if "sharp" in ob["name"].lower() or (
        not ob["wheel"] and sorted(round(ob["max"][k] - ob["min"][k], 1) for k in range(3))[2] > 4.0
        and 3.5 <= sorted(ob["max"][k] - ob["min"][k] for k in range(3))[2] <= 5.0
        and 1.0 <= sorted(ob["max"][k] - ob["min"][k] for k in range(3))[0] <= 1.6
        and ob["max"][1] > 0 and ob["max"][2] < 15)]
    # default mechanism groups from names and position
    named_claw = [ob for ob in out_bodies if re.match(r"(clawbody|rackr|copler|coupler|tpu|go ?pro|obj_1_go)", ob["name"].lower())]
    head_lo = np.min([ob["min"] for ob in named_claw], axis=0) - 1.5 if named_claw else None
    head_hi = np.max([ob["max"] for ob in named_claw], axis=0) + 1.5 if named_claw else None
    for ob in out_bodies:
        lo, hi = np.array(ob["min"]), np.array(ob["max"])
        nm = ob["name"].lower()
        if ob["wheel"]:
            g = "wheel"
        elif any(ob is sh for sh in sharps):
            g = "base"
        elif nm.startswith("tpul"):
            g = "clawL"
        elif nm.startswith("tpur"):
            g = "clawR"
        elif head_lo is not None and (lo >= head_lo).all() and (hi <= head_hi).all():
            g = "arm"
        elif (lo[1] + hi[1]) / 2 > 6 and lo[2] > 9.5 and not ("sonic" in nm):
            g = "lift"
        else:
            g = "base"
        ob["group"] = g
    for ob in out_bodies:   # fingers: left = smaller x
        if ob["group"] in ("clawL", "clawR"):
            ob["group"] = "clawL" if (ob["min"][0] + ob["max"][0]) / 2 < 0 else "clawR"
    pivot_src = next((ob for ob in out_bodies if ob["name"].lower().startswith(("copler", "coupler"))), None)
    pivot = ([(pivot_src["min"][k] + pivot_src["max"][k]) / 2 for k in range(3)] if pivot_src
             else [0, 15, 20])
    # measured sensor positions
    measured = {}
    allmin = np.min([ob["min"] for ob in out_bodies], axis=0)
    allmax = np.max([ob["max"] for ob in out_bodies], axis=0)
    measured["w"] = round(float(2 * max(-allmin[0], allmax[0])), 1)
    measured["l"] = round(float(allmax[1] - allmin[1]), 1)
    measured["front"] = round(float(allmax[1]), 1)
    measured["back"] = round(float(-allmin[1]), 1)
    measured["height"] = round(float(allmax[2]), 1)
    measured["wheelRadiusCm"] = round(float(np.mean([w["max"][2] - w["min"][2] for w in wheels]) / 20), 2)
    measured["wheelCircleCm"] = round(float(np.mean([np.hypot(*(c[:2] - center[:2])) for c in wc]) / 10), 2)
    son = [ob for ob in out_bodies if "sonic" in ob["name"].lower()]
    if son:
        measured["sonicSide"] = round(float(np.mean([max(abs(o["min"][0]), abs(o["max"][0])) for o in son])), 1)
        measured["sonicAlong"] = round(float(np.mean([(o["min"][1] + o["max"][1]) / 2 for o in son])), 1)
    if len(sharps) >= 2:
        measured["sharpFront"] = round(float(np.mean([o["max"][1] for o in sharps])), 1)
        measured["sharpSide"] = round(float(np.mean([abs((o["min"][0] + o["max"][0]) / 2) for o in sharps])), 1)
    meta = {"version": 1, "source": str(src), "bodies": out_bodies, "measured": measured,
            "joints": {"liftModelDeg": 30, "liftCmPerDeg": 0.05, "armPivot": [round(x, 2) for x in pivot],
                       "armModelDeg": 165, "armSign": 1, "clawTravelCm": 2.0, "clawModelDeg": 18}}
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    pos = np.concatenate(pos_parts).astype("<f4")
    idx = np.concatenate(idx_parts).astype("<u4")
    (MAPS_DIR / MODEL_BIN).write_bytes(pos.tobytes() + idx.tobytes())
    meta["posCount"] = int(pos.size)
    meta["idxCount"] = int(idx.size)
    write_text(MAPS_DIR / MODEL_JSON, json.dumps(meta, ensure_ascii=False))
    return meta


def safe_name(name):
    if not isinstance(name, str) or not NAME_PATTERN.match(name.strip()):
        raise ValueError("Имя: буквы, цифры, пробел, _ или -, до 60 символов")
    return name.strip()


def write_text(path, text):
    """Writes through a temporary file so a crash never leaves half a map."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


class PlannerServer(ThreadingHTTPServer):
    def __init__(self):
        super().__init__(("127.0.0.1", 0), PlannerHandler)
        self.token = secrets.token_urlsafe(24)
        self.lock = threading.Lock()


class PlannerHandler(BaseHTTPRequestHandler):
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
            self.send_data(200, PAGE.replace("__TOKEN__", self.server.token),
                           "text/html")
        elif self.path.startswith("/vendor/") and self.path[8:] in VENDOR_FILES:
            path = VENDOR_DIR / self.path[8:]
            if not path.is_file():
                self.send_data(404, {"error": "нет файла " + str(path)})
                return
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/javascript; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.split("?")[0] in ("/" + MODEL_JSON, "/" + MODEL_BIN):
            path = MAPS_DIR / self.path.split("?")[0][1:]
            if not path.is_file():
                self.send_data(404, {"error": "Модели робота ещё нет — загрузите .3mf"})
                return
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8"
                             if path.suffix == ".json" else "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_data(404, {"error": "Не найдено"})

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= MAX_REQUEST_BYTES:
                raise ValueError("Неверный размер запроса (фото больше ~18 МБ?)")
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError("Неверный запрос")
            if request.get("token") != self.server.token:
                raise ValueError("Неверный код сессии — обновите страницу")
            handler = {
                "/list": self.list_maps, "/load": self.load_map,
                "/save": self.save_map, "/saveCode": self.save_code,
                "/info": self.info, "/scan": self.scan,
                "/importModel": self.import_model, "/saveJoints": self.save_joints,
            }.get(self.path)
            if handler is None:
                self.send_data(404, {"error": "Не найдено"})
            else:
                with self.server.lock:
                    handler(request)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.send_data(400, {"error": str(error)})

    def import_model(self, request):
        path = request.get("path")
        if not isinstance(path, str) or not path.strip():
            raise ValueError("Укажите путь к .3mf")
        meta = convert_3mf(path.strip())
        self.send_data(200, {"ok": True, "bodies": len(meta["bodies"]), "measured": meta["measured"]})

    def save_joints(self, request):
        path = MAPS_DIR / MODEL_JSON
        meta = json.loads(path.read_text(encoding="utf-8"))
        joints, groups = request.get("joints"), request.get("groups")
        if not isinstance(joints, dict) or not isinstance(groups, dict):
            raise ValueError("Неверные данные механизмов")
        allowed = {"base", "lift", "arm", "clawL", "clawR", "wheel"}
        meta["joints"].update({k: v for k, v in joints.items() if isinstance(v, (int, float, list))})
        for b in meta["bodies"]:
            g = groups.get(b["name"])
            if g in allowed:
                b["group"] = g
        write_text(path, json.dumps(meta, ensure_ascii=False))
        self.send_data(200, {"ok": True})

    def info(self, _request):
        self.send_data(200, {"projectDir": str(PROJECT_DIR)})

    def scan(self, request):
        path = request.get("path") or str(PROJECT_DIR)
        if not isinstance(path, str):
            raise ValueError("Неверная папка")
        self.send_data(200, scan_project(path))

    def list_maps(self, _request):
        names = sorted(path.stem for path in MAPS_DIR.glob("*.json")) \
            if MAPS_DIR.is_dir() else []
        self.send_data(200, {"names": names})

    def load_map(self, request):
        path = MAPS_DIR / (safe_name(request.get("name")) + ".json")
        with path.open("r", encoding="utf-8") as source:
            data = json.load(source)
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("Это не карта поля")
        self.send_data(200, {"map": data})

    def save_map(self, request):
        name = safe_name(request.get("name"))
        data = request.get("map")
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("Неверная карта")
        path = MAPS_DIR / (name + ".json")
        write_text(path, json.dumps(data, ensure_ascii=False, indent=1))
        self.send_data(200, {"path": str(path)})

    def save_code(self, request):
        name = safe_name(request.get("name"))
        code = request.get("code")
        if not isinstance(code, str):
            raise ValueError("Нет кода")
        path = MAPS_DIR / (name + "_route.java.txt")
        write_text(path, code + "\n")
        self.send_data(200, {"path": str(path)})


def main():
    server = PlannerServer()
    url = "http://127.0.0.1:%d/" % server.server_port
    print("Field planner: %s  (Ctrl+C to stop)" % url)
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
