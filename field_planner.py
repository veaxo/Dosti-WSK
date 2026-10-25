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
import re
import secrets
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
MAPS_DIR = PROJECT_DIR / "field_maps"
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
:root { --bg:#14202a; --panel:#233441; --panel2:#1b2a35; --text:#f4f7f9;
        --muted:#a9b7c2; --ok:#38b882; --warn:#e2b34a; --bad:#e2675a;
        --accent:#4aa3e2; }
* { box-sizing:border-box; }
body { margin:0; font:14px system-ui,sans-serif; background:var(--bg);
       color:var(--text); height:100vh; display:flex; flex-direction:column; }
header { display:flex; flex-wrap:wrap; gap:6px; align-items:center;
         padding:8px 10px; background:var(--panel); }
header h1 { font-size:16px; margin:0 10px 0 0; }
button, select, input { font:inherit; color:var(--text); background:#30485a;
         border:1px solid #47657a; border-radius:5px; padding:4px 8px; }
button { cursor:pointer; }
button:hover { background:#3b5a70; }
button.on { background:var(--accent); border-color:var(--accent); color:#06121b; }
button.bad { background:#6b2d27; border-color:#8f3b33; }
input[type=number] { width:76px; }
input[type=text] { width:150px; }
main { flex:1; display:flex; min-height:0; }
#stage { flex:1; position:relative; min-width:0; }
canvas { position:absolute; inset:0; width:100%; height:100%; display:block; }
#hint { position:absolute; left:8px; bottom:8px; background:#0d161dcc;
        padding:4px 8px; border-radius:5px; color:var(--muted); pointer-events:none; }
#cursor { position:absolute; right:8px; bottom:8px; background:#0d161dcc;
          padding:4px 8px; border-radius:5px; pointer-events:none; }
aside { width:390px; overflow:auto; background:var(--panel2); padding:10px;
        border-left:1px solid #33495a; }
section { background:var(--panel); border-radius:7px; padding:9px; margin-bottom:10px; }
section h2 { font-size:14px; margin:0 0 7px; }
.row { display:flex; flex-wrap:wrap; gap:6px; align-items:center; margin:4px 0; }
.row label { color:var(--muted); }
.muted { color:var(--muted); font-size:12px; }
.act { display:flex; gap:4px; align-items:center; margin:3px 0; }
.act select { flex:1; min-width:0; }
textarea { width:100%; height:260px; font:12px ui-monospace,monospace;
           background:#0d161d; color:#dce6ee; border:1px solid #33495a;
           border-radius:5px; padding:6px; }
#warnings div { padding:3px 6px; border-radius:4px; margin:2px 0; cursor:pointer; }
.w-bad { background:#5a2621; } .w-warn { background:#5a4a1e; } .w-ok { background:#1e4a37; }
#status { margin-left:auto; color:var(--muted); }
#tlBar { display:flex; gap:8px; align-items:stretch; padding:6px 8px;
         background:var(--panel2); border-bottom:1px solid #33495a; }
#tlControls { display:flex; flex-direction:column; gap:4px; justify-content:center;
              min-width:150px; }
#tlControls .row { margin:0; }
#tlScrub { width:150px; }
#timeline { flex:1; display:flex; gap:6px; overflow-x:auto; overflow-y:hidden;
            padding-bottom:4px; scroll-behavior:smooth; }
.card { flex:0 0 auto; width:116px; background:var(--panel); border:2px solid transparent;
        border-left:5px solid #47657a; border-radius:7px; padding:5px 7px; cursor:pointer;
        display:flex; flex-direction:column; gap:2px; position:relative; }
.card:hover { background:#2b4152; }
.card.sel { width:250px; border-color:#fff; cursor:default; }
.card .top { display:flex; justify-content:space-between; color:var(--muted); font-size:11px; }
.card .big { font-size:15px; font-weight:600; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.card .code { font:11px ui-monospace,monospace; color:#a9c1d3; white-space:nowrap;
              overflow:hidden; text-overflow:ellipsis; }
.card .ed { display:flex; flex-wrap:wrap; gap:4px; align-items:center; margin-top:3px; font-size:12px; }
.card .ed input[type=number] { width:62px; padding:2px 4px; }
.card .ed button { padding:2px 6px; }
.card.move { border-left-color:#4aa3e2; } .card.back { border-left-color:#e2b34a; }
.card.rotate { border-left-color:#9b6fd0; } .card.pickup { border-left-color:#38b882; }
.card.place { border-left-color:#3fae7a; } .card.sonic { border-left-color:#e2675a; }
.card.other { border-left-color:#8a96a3; }
.card .badge { position:absolute; top:-6px; right:-6px; background:var(--bad); color:#fff;
               border-radius:9px; font-size:11px; padding:0 5px; }
.card .badge.warn { background:var(--warn); color:#06121b; }
.wpGap { flex:0 0 auto; width:2px; background:#47657a; margin:6px 2px; }
</style>
</head>
<body>
<header>
  <h1>Карта поля</h1>
  <span class="row" id="tools"></span>
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
  <div id="stage"><canvas id="cv"></canvas><div id="hint"></div><div id="cursor"></div></div>
  <aside>
    <section>
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
    <section>
      <h2>Робот</h2>
      <div class="row"><label>ширина</label><input type="number" id="rW" step="0.5">
        <label>длина</label><input type="number" id="rL" step="0.5"></div>
      <div class="row"><label>соник от центра вбок</label><input type="number" id="rSonicSide" step="0.5"></div>
      <div class="row"><label>соник от центра вперёд (− = сзади)</label><input type="number" id="rSonicAlong" step="0.5"></div>
      <div class="row"><label>Sharp от центра вперёд</label><input type="number" id="rSharp" step="0.5"></div>
      <div class="row"><label>старт: x</label><input type="number" id="sX" step="1">
        <label>y</label><input type="number" id="sY" step="1">
        <label>смотрит</label><input type="number" id="sB" step="1" title="0 = вверх, 90 = вправо, по часовой"></div>
      <div class="muted">Направление: 0° = вверх по карте, 90° = вправо (по часовой, как гироскоп).
        На старте гироскоп = 0, все повороты считаются от этого направления.</div>
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
  wall:     {name:"Стена",      fill:"#8a96a3", solid:true},
  obstacle: {name:"Препятствие",fill:"#c9773c", solid:true},
  podium:   {name:"Подиум",     fill:"#9b6fd0", solid:true},
  zone:     {name:"Зона",       fill:"#3fae7a", solid:false},
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
  wait:      {name:"подождать, с", value:true, def:1},
  custom:    {name:"свой код Java", value:"text", def:"stateAutomatic++;"},
};
const TOOLS = [["select","Выбор"],["wall","Стена"],["obstacle","Препятствие"],
  ["podium","Подиум"],["zone","Зона"],["object","Объект"],["start","Старт"],
  ["route","Маршрут"],["measure","Линейка"]];
const HINTS = {
  select:"Клик — выбрать, тянуть — двигать (угол прямоугольника — размер). Пустое место — сдвиг карты, колесо — масштаб. Delete — удалить.",
  wall:"Тяните прямоугольник. Точные размеры — в панели справа.",
  obstacle:"Препятствие можно объехать: включите barrier у отрезка, который через него идёт.",
  podium:"Подиум — твёрдый, робот в него не въезжает.",
  zone:"Зона (например, куда класть) — робот через неё ездит.",
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
    robot:{w:35, l:35, sonicSide:17, sonicAlong:-12, sharpFront:17},
    start:{x:30, y:30, b:0}, rects:[], objects:[], route:[], firstCase:0,
    bg:{data:null, opacity:0.5}};
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
function rayCast(p, b) {
  const d = dirOf(b);
  let best = Infinity;
  // field border (from inside)
  if (d.x > 1e-9) best = Math.min(best, (S.field.w - p.x) / d.x);
  if (d.x < -1e-9) best = Math.min(best, (0 - p.x) / d.x);
  if (d.y > 1e-9) best = Math.min(best, (S.field.h - p.y) / d.y);
  if (d.y < -1e-9) best = Math.min(best, (0 - p.y) / d.y);
  for (const r of solids()) {
    let t0 = -Infinity, t1 = Infinity;
    const ax = [[p.x, d.x, r.x, r.x + r.w], [p.y, d.y, r.y, r.y + r.h]];
    let miss = false;
    for (const [o, v, lo, hi] of ax) {
      if (Math.abs(v) < 1e-9) { if (o < lo || o > hi) { miss = true; break; } continue; }
      let a = (lo - o) / v, c = (hi - o) / v;
      if (a > c) [a, c] = [c, a];
      t0 = Math.max(t0, a); t1 = Math.min(t1, c);
    }
    if (miss || t0 > t1 || t1 < 0) continue;
    best = Math.min(best, Math.max(0, t0));
  }
  return Math.max(0, best);
}
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

// ── simulation and code ─────────────────────────────────────────────────
function simulate() {
  const R = S.robot;
  const out = {steps:[], warnings:[], moves:[], stops:[], rotations:[]};
  let pose = {x:S.start.x, y:S.start.y, b:norm360(S.start.b)};
  let headingKnown = true;
  const hl = R.l / 2, hw = R.w / 2;
  const sensorPoint = p => { const f = dirOf(p.b); return {x:p.x + f.x * R.sonicAlong, y:p.y + f.y * R.sonicAlong}; };
  const sonicReading = (p, side) => rayCast(sensorPoint(p), p.b + side * 90) - R.sonicSide;
  const sharpReading = p => rayCast(p, p.b) - R.sharpFront;
  let curSrc = null;      // what in the map made this step (for the timeline)
  const step = (code, wp, note) => {
    const last = out.steps[out.steps.length - 1];
    if (last) last.after = {...pose};
    out.steps.push({code, wp, note, src:curSrc, before:{...pose}});
  };
  const warn = (level, wp, text) => out.warnings.push({level, wp, text});

  function checkSwept(from, to, b, wp, along, barrier) {
    const dist = Math.hypot(to.x - from.x, to.y - from.y);
    const mid = {x:(from.x + to.x) / 2, y:(from.y + to.y) / 2};
    const poly = along ? orientedRect(mid, b, hl + dist / 2, hw)
                       : orientedRect(mid, b, hl, hw + dist / 2);
    out.moves.push({from:{...from}, to:{...to}, b, poly, wp});
    if (polyOutside(poly)) warn("bad", wp, "робот выезжает за поле");
    for (const r of solids()) {
      if (!polyHitsRect(poly, r)) continue;
      if (r.type === "obstacle" && barrier) {
        warn("warn", wp, "через препятствие" + (r.label ? " «" + r.label + "»" : "") + " — сработает объезд barrier()");
      } else {
        warn("bad", wp, "столкновение: " + TYPES[r.type].name.toLowerCase()
          + (r.label ? " «" + r.label + "»" : "")
          + (r.type === "obstacle" ? " (включите barrier или обойдите)" : ""));
      }
    }
  }
  function rotateTo(b, wp, force) {
    b = norm360(b);
    if (!force && headingKnown && Math.abs(angDiff(b, pose.b)) < 0.5) return;
    const rel = norm360(b - S.start.b);
    step("rotateTheRobot(" + fmt(rel) + ");", wp);
    const rr = Math.hypot(hl, hw);
    out.rotations.push({c:{x:pose.x, y:pose.y}, r:rr, wp});
    for (const r of solids()) {
      if (circleHitsRect(pose, rr, r)) { warn("warn", wp, "при повороте может задеть " + TYPES[r.type].name.toLowerCase()); break; }
    }
    pose.b = b; headingKnown = true;
  }
  function lateral(dir, distance, wp) {           // dir -1 left, +1 right
    const d = dirOf(pose.b + dir * 90);
    const to = {x:pose.x + d.x * distance, y:pose.y + d.y * distance};
    checkSwept(pose, to, pose.b, wp, false, false);
    pose.x = to.x; pose.y = to.y;
  }
  function along(distance, wp, barrier) {
    const d = dirOf(pose.b);
    const to = {x:pose.x + d.x * distance, y:pose.y + d.y * distance};
    checkSwept(pose, to, pose.b, wp, true, barrier);
    pose.x = to.x; pose.y = to.y;
  }

  S.route.forEach((wp, i) => {
    const n = i + 1;
    const target = {x:wp.x, y:wp.y};
    const dist = Math.hypot(target.x - pose.x, target.y - pose.y);
    if (dist >= 0.5) {
      const segB = bearingTo(pose, target);
      const robotB = wp.back ? segB + 180 : segB;
      curSrc = {kind:"rotate", wp:i};
      rotateTo(robotB, n, !headingKnown);
      curSrc = {kind:"move", wp:i};
      if (wp.back) {
        step("goBackDistance(" + fmt(dist) + ");", n, "точка " + n);
        along(-dist, n, false);
      } else if (wp.sharp) {
        const endPose = {x:target.x, y:target.y, b:pose.b};
        const reading = sharpReading(endPose);
        step("goForwardSharp(" + fmt(reading) + ");", n, "точка " + n + ", по Sharp");
        if (reading < 10) warn("bad", n, "Sharp: " + fmt(reading) + " см — ближе 10 см он не меряет (DriveMotor поднимет до 10)");
        else if (reading > 80) warn("warn", n, "Sharp: " + fmt(reading) + " см — дальше ~80 см ненадёжно, лучше энкодер");
        along(dist, n, false);
      } else {
        step((wp.barrier ? "barrier()." : "") + "goForwardDistance(" + fmt(dist) + ");", n, "точка " + n);
        along(dist, n, wp.barrier);
      }
    } else if (!headingKnown && wp.actions.some(a => a.type !== "rotate")) {
      warn("warn", n, "после захвата направление робота неизвестно — добавьте поворот");
    }
    wp.actions.forEach((a, ai) => {
      curSrc = {kind:"action", wp:i, ai};
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
        case "back":
          step("goBackDistance(" + fmt(v) + ");", n); along(-v, n, false); break;
        case "sharp": {
          const reading = sharpReading(pose);
          step("goForwardSharp(" + fmt(v) + ");", n);
          if (v < 10) warn("warn", n, "Sharp ближе 10 см не меряет, остановится на 10");
          const move = reading - Math.max(10, v);
          if (reading > 80) warn("warn", n, "Sharp сейчас видит " + fmt(reading) + " см — может не увидеть стену");
          if (move > 0) along(move, n, false);
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
          const watched = -side;                            // sonic on the other side
          const reading = sonicReading(pose, watched);
          step((side < 0 ? "goLeftWithRightSonic(" : "goRigtWithLeftSonic(") + fmt(v) + ");", n);
          if (reading < v) lateral(side, v - reading, n);
          else warn("warn", n, "соник уже показывает " + fmt(reading) + " см ≥ " + fmt(v) + " — шаг сразу закончится");
          break;
        }
      }
    });
    out.stops.push({pose:{...pose}, wp:n,
      sonicL:sonicReading(pose, -1), sonicR:sonicReading(pose, 1), sharp:sharpReading(pose)});
  });
  if (out.steps.length) out.steps[out.steps.length - 1].after = {...pose};
  out.final = pose;
  return out;
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

function fitView() {
  const r = cv.getBoundingClientRect();
  if (r.width < 50 || r.height < 50) return;   // not laid out yet
  const pad = 40;
  view.scale = Math.max(0.2, Math.min((r.width - 2 * pad) / S.field.w, (r.height - 2 * pad) / S.field.h));
  view.ox = (r.width - S.field.w * view.scale) / 2;
  view.oy = r.height - (r.height - S.field.h * view.scale) / 2;
  draw();
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
  ctx.lineWidth = 3; ctx.strokeStyle = "#0d161d"; ctx.strokeText(text, s.x, s.y);
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
  const F = S.field;
  const o = toScreen({x:0, y:F.h});
  ctx.fillStyle = "#1d2e3a"; ctx.fillRect(o.x, o.y, F.w * view.scale, F.h * view.scale);
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
    ctx.strokeStyle = x % 50 ? "#ffffff14" : "#ffffff30"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    if (x % 50 === 0) label(String(x), {x, y:-6}, "#a9b7c2");
  }
  for (let y = 0; y <= F.h + 1e-6; y += 10) {
    if (!minor && y % 50) continue;
    const a = toScreen({x:0, y}), b = toScreen({x:F.w, y});
    ctx.strokeStyle = y % 50 ? "#ffffff14" : "#ffffff30"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    if (y % 50 === 0) label(String(y), {x:-6, y}, "#a9b7c2", "right");
  }
  ctx.strokeStyle = "#dfe7ee"; ctx.lineWidth = 2;
  ctx.strokeRect(o.x, o.y, F.w * view.scale, F.h * view.scale);

  for (const rc of S.rects) {
    const t = TYPES[rc.type];
    const pts = [{x:rc.x,y:rc.y},{x:rc.x+rc.w,y:rc.y},{x:rc.x+rc.w,y:rc.y+rc.h},{x:rc.x,y:rc.y+rc.h}];
    const sel = selected && selected.kind === "rect" && selected.id === rc.id;
    poly(pts, t.fill + (rc.type === "zone" ? "55" : "cc"), sel ? "#fff" : t.fill, sel ? 2 : 1);
    label((rc.label ? rc.label + " " : "") + fmt(rc.w) + "×" + fmt(rc.h), {x:rc.x + rc.w / 2, y:rc.y + rc.h / 2});
  }
  for (const ob of S.objects) {
    const s = toScreen(ob);
    const sel = selected && selected.kind === "object" && selected.id === ob.id;
    ctx.beginPath(); ctx.arc(s.x, s.y, Math.max(5, 2.5 * view.scale), 0, 2 * Math.PI);
    ctx.fillStyle = ob.color || "#e2675a"; ctx.fill();
    ctx.strokeStyle = sel ? "#fff" : "#0d161d"; ctx.lineWidth = 2; ctx.stroke();
    label(ob.name || "объект", {x:ob.x, y:ob.y + 6 + 8 / view.scale});
  }

  // simulated path
  if (sim) {
    for (const m of sim.moves) poly(m.poly, "#4aa3e214", null);
    ctx.strokeStyle = "#4aa3e2"; ctx.lineWidth = 2;
    for (const m of sim.moves) {
      const a = toScreen(m.from), b = toScreen(m.to);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    for (const st of sim.stops) drawRobot(st.pose, "#4aa3e2aa", null, [4, 4]);
  }
  // waypoints
  S.route.forEach((wp, i) => {
    const s = toScreen(wp);
    const sel = selected && selected.kind === "route" && selected.id === wp.id;
    ctx.beginPath(); ctx.arc(s.x, s.y, sel ? 10 : 8, 0, 2 * Math.PI);
    ctx.fillStyle = wp.back ? "#e2b34a" : "#4aa3e2"; ctx.fill();
    ctx.strokeStyle = sel ? "#fff" : "#0d161d"; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = "#06121b"; ctx.font = "bold 11px system-ui"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(String(i + 1), s.x, s.y);
    if (wp.actions.length) label(wp.actions.map(a => actionShort(a)).join(", "), {x:wp.x, y:wp.y - 14 / view.scale}, "#e2b34a");
  });
  if (sim) {
    const bad = new Set(sim.warnings.filter(w => w.level === "bad").map(w => w.wp));
    for (const m of sim.moves) if (bad.has(m.wp)) poly(m.poly, "#e2675a33", "#e2675a", 1);
  }
  // timeline: selected step / playback
  const tlPose = timelinePose();
  if (tlPose) {
    const stp = sim.steps[tlPose.index];
    if (stp && stp.before && stp.after) {
      const a = toScreen(stp.before), b = toScreen(stp.after);
      ctx.strokeStyle = "#ffd75e"; ctx.lineWidth = 4;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    drawRobot(tlPose.pose, "#ffd75e", "#ffd75e44");
  }
  // start robot
  const st = {x:S.start.x, y:S.start.y, b:S.start.b};
  const selStart = selected && selected.kind === "start";
  drawRobot(st, selStart ? "#ffffff" : "#38b882", "#38b88244");
  label("СТАРТ", {x:st.x, y:st.y});

  if (measure && measure.a) {
    const b = measure.b || cursorCm;
    if (b) {
      const a = toScreen(measure.a), c = toScreen(b);
      ctx.strokeStyle = "#ffd75e"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(c.x, c.y); ctx.stroke(); ctx.setLineDash([]);
      label(fmt(Math.hypot(b.x - measure.a.x, b.y - measure.a.y)) + " см",
        {x:(measure.a.x + b.x) / 2, y:(measure.a.y + b.y) / 2}, "#ffd75e");
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
  if (tool === "measure") {
    if (!measure || measure.b) measure = {a:sp, b:null}; else measure.b = sp;
    draw(); return;
  }
  if (TYPES[tool]) { drag = {kind:"newRect", type:tool, a:sp, cur:sp}; return; }
  if (tool === "object") {
    pushUndo();
    const ob = {id:uid(), x:sp.x, y:sp.y, name:"объект " + (S.objects.length + 1), color:"#e2675a"};
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
window.addEventListener("mouseup", () => {
  if (drag && drag.kind === "newRect" && drag.cur) {
    const r = rectFrom(drag.a, drag.cur);
    if (r.w >= 1 && r.h >= 1) {
      pushUndo();
      const rc = {id:uid(), type:drag.type, label:"", ...r};
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
  view.scale = Math.max(0.2, Math.min(40, view.scale * k));
  view.ox = e.offsetX - p.x * view.scale; view.oy = e.offsetY + p.y * view.scale;
  draw();
}, {passive:false});
window.addEventListener("keydown", e => {
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
    const color = document.createElement("input"); color.type = "color"; color.value = el.color || "#e2675a";
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
          checkbox("стоп по Sharp", el.sharp, v => el.sharp = v)));
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
        a.value = def.value ? def.def : null; changed();
      });
      d.appendChild(sel);
      const def = ACTIONS[a.type];
      if (def.value === "text") d.appendChild(textInput(a.value, v => a.value = v));
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
}
function changed(record, light) {
  sim = simulate();
  if (tlSel !== null && tlSel >= sim.steps.length) tlSel = sim.steps.length ? sim.steps.length - 1 : null;
  if (tlT > sim.steps.length) tlT = sim.steps.length;
  if (!light) { renderSelPanel(); syncInputs(); }
  if (typeof renderTimeline === "function" && !tlPlaying) renderTimeline(false);
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
bindNumber("fieldW", v => S.field.w = Math.max(20, v));
bindNumber("fieldH", v => S.field.h = Math.max(20, v));
bindNumber("rW", v => S.robot.w = Math.max(5, v));
bindNumber("rL", v => S.robot.l = Math.max(5, v));
bindNumber("rSonicSide", v => S.robot.sonicSide = v);
bindNumber("rSonicAlong", v => S.robot.sonicAlong = v);
bindNumber("rSharp", v => S.robot.sharpFront = v);
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
$("fit").addEventListener("click", () => { viewTouched = false; fitView(); });
$("copy").addEventListener("click", async () => {
  try { await navigator.clipboard.writeText($("code").value); status("Код скопирован"); }
  catch (e) { $("code").select(); document.execCommand("copy"); status("Код скопирован"); }
});

// ── timeline ────────────────────────────────────────────────────────────
let tlSel = null;          // selected step index
let tlT = 0;               // playback position: step index + fraction
let tlPlaying = false;
let tlLastFrame = 0;

function stepDuration(st) {
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
  return {index:tlSel, pose:sim.steps[tlSel].after};
}
function stepView(st) {
  const src = st.src || {};
  const wp = S.route[src.wp];
  const c = st.code;
  const num = m => (c.match(/\(([-\d.]+)\)/) || [])[1];
  if (src.kind === "rotate" || c.startsWith("rotateTheRobot")) return {cls:"rotate", big:"↻ " + num() + "°"};
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
  return {cls:"other", big:"код"};
}
function tlSelect(i, scroll) {
  tlPlaying = false; $("tlPlay").textContent = "▶ Пуск";
  tlSel = i;
  if (i !== null && sim.steps[i]) {
    tlT = i + 1;
    const src = sim.steps[i].src;
    if (src && S.route[src.wp]) selected = {kind:"route", id:S.route[src.wp].id};
  }
  renderSelPanel(); renderTimeline(scroll !== false); draw();
}
function renderTimeline(scroll) {
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
    card.className = "card " + v.cls + (i === tlSel ? " sel" : "");
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
  } else if (src.kind === "action") {
    const a = wp.actions[src.ai];
    const sel = document.createElement("select");
    for (const [k, t] of Object.entries(ACTIONS)) sel.add(new Option(t.name, k, false, k === a.type));
    sel.style.maxWidth = "150px";
    sel.addEventListener("change", () => { pushUndo(); a.type = sel.value; const d = ACTIONS[a.type]; a.value = d.value ? d.def : null; keepSel(); });
    ed.appendChild(sel);
    const d = ACTIONS[a.type];
    if (d.value === "text") {
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
  cards.forEach((c, k) => c.style.outline = k === i ? "2px solid #ffd75e" : "");
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
function status(text, bad) { const s = $("status"); s.textContent = text; s.style.color = bad ? "var(--bad)" : "var(--muted)"; }
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
    selected = null; loadBg(); changed(); fitView(); status("Открыто: " + name);
  } catch (e) { status(e.message, true); }
  $("mapList").value = "";
});
$("newMap").addEventListener("click", () => {
  if (!confirm("Начать новую карту? Несохранённое пропадёт (можно отменить Ctrl+Z).")) return;
  pushUndo(); S = defaultMap(); selected = null; bgImage = null; changed(); fitView();
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
</script>
</body>
</html>
"""


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
            }.get(self.path)
            if handler is None:
                self.send_data(404, {"error": "Не найдено"})
            else:
                with self.server.lock:
                    handler(request)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.send_data(400, {"error": str(error)})

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
