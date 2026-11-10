// MapperX inside VS Code (works in old versions too, e.g. WPILib's 1.41 that
// has no Simple Browser): starts field_planner.py and shows it in a tab.
const vscode = require("vscode");
const cp = require("child_process");
const fs = require("fs");
const http = require("http");
const path = require("path");

let server = null;      // child process we started
let url = null;         // http://127.0.0.1:PORT/
let panel = null;
let output = null;

function log(text) {
  if (!output) output = vscode.window.createOutputChannel("MapperX");
  output.append(text);
}

// GET / of a running MapperX: its code stamp (header) and session token.
function probe(address) {
  return new Promise(done => {
    const req = http.get(address, res => {
      let body = "";
      res.on("data", c => { body += c; });
      res.on("end", () => {
        const m = body.match(/const TOKEN = "([^"]+)"/);
        done(res.statusCode === 200 ? { stamp: res.headers["x-mapperx-stamp"] || "", token: m ? m[1] : null } : null);
      });
    });
    req.on("error", () => done(null));
    req.setTimeout(1500, () => { req.destroy(); done(null); });
  });
}

function post(address, pathName, data) {
  return new Promise(done => {
    const body = JSON.stringify(data);
    const u = new URL(address);
    const req = http.request({ hostname: u.hostname, port: u.port, path: pathName, method: "POST",
      headers: { "Content-Type": "application/json", "Content-Length": Buffer.byteLength(body) } }, res => { res.resume(); done(true); });
    req.on("error", () => done(false));
    req.end(body);
  });
}

function fileStamp(file) {
  try { const st = fs.statSync(file); return Math.floor(st.mtimeMs) + "-" + st.size; }
  catch (e) { return ""; }
}

// A MapperX started earlier (old code) is stopped so the new file runs.
// Returns true when the running one is current and can be used.
async function stopOutdated(address, planner, force) {
  const info = await probe(address);
  if (!info) return false;
  if (!force && planner && info.stamp && info.stamp === fileStamp(planner)) return true;
  log("MapperX на " + address + " устарел или перезапуск — останавливаю\n");
  if (info.token) await post(address, "/shutdown", { token: info.token });
  for (let i = 0; i < 20 && await ping(address); i++) await new Promise(r => setTimeout(r, 150));
  return false;
}

function ping(address) {
  return new Promise(done => {
    const req = http.get(address, res => { res.resume(); done(res.statusCode === 200); });
    req.on("error", () => done(false));
    req.setTimeout(800, () => { req.destroy(); done(false); });
  });
}

async function findPlanner() {
  for (const folder of vscode.workspace.workspaceFolders || []) {
    const direct = path.join(folder.uri.fsPath, "field_planner.py");
    if (fs.existsSync(direct)) return direct;
  }
  const found = await vscode.workspace.findFiles("**/field_planner.py", "**/{node_modules,build,.git}/**", 1);
  return found.length ? found[0].fsPath : null;
}

async function ensureServer(force) {
  const planner = await findPlanner();
  for (const address of [url, "http://127.0.0.1:8765/"]) {
    if (address && await stopOutdated(address, planner, force)) { url = address; return url; }
  }
  if (server) { server.kill(); server = null; }
  if (!planner) throw new Error("Не нашёл field_planner.py — откройте в VS Code папку проекта (Dosti_WSK).");
  // macOS: old VS Code (x86_64) runs under Rosetta on Apple Silicon, and the
  // /usr/bin/python3 shim then fails in xcrun. The real Python by its full
  // path starts in the Mac's own architecture (arm64 on Apple Silicon, x86_64
  // on Intel), so it goes first; the others are fallbacks (Homebrew on Apple
  // Silicon / Intel, PATH). Windows: python / py.
  const pythons = process.platform === "win32" ? ["python", "py"]
    : ["/Library/Developer/CommandLineTools/usr/bin/python3", "/opt/homebrew/bin/python3", "/usr/local/bin/python3", "python3", "/usr/bin/python3"]
        .filter(p => !p.startsWith("/") || fs.existsSync(p));
  return new Promise((resolve, reject) => {
    let tried = 0;
    const start = () => {
      if (tried >= pythons.length) { reject(new Error("Не удалось запустить Python 3 — установите его.")); return; }
      const exe = pythons[tried++];
      log("$ " + exe + " " + planner + " --no-browser\n");
      let child;
      try {
        child = cp.spawn(exe, [planner, "--no-browser"], { cwd: path.dirname(planner), env: Object.assign({}, process.env, { MAPPERX_NO_BROWSER: "1", PYTHONUNBUFFERED: "1" }) });
      } catch (e) { start(); return; }
      let settled = false;
      const timer = setTimeout(() => { if (!settled) { settled = true; reject(new Error("MapperX не ответил за 20 с — см. вывод «MapperX».")); } }, 20000);
      child.on("error", () => { if (!settled) { settled = true; clearTimeout(timer); start(); } });
      child.stdout.on("data", chunk => {
        const text = chunk.toString(); log(text);
        const m = text.match(/MapperX: (http:\/\/127\.0\.0\.1:\d+\/)/);
        if (m && !settled) { settled = true; clearTimeout(timer); url = m[1]; server = child; resolve(url); }
      });
      child.stderr.on("data", chunk => log(chunk.toString()));
      child.on("exit", code => {
        log("MapperX остановлен (код " + code + ")\n");
        if (server === child) { server = null; url = null; }
        if (!settled) { clearTimeout(timer); settled = true; log("пробую другой Python…\n"); start(); }
      });
    };
    start();
  });
}

function pageHtml(address) {
  const origin = address.replace(/\/$/, "");
  return `<!DOCTYPE html><html><head><meta charset="utf-8">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; frame-src ${origin} http://127.0.0.1:*; style-src 'unsafe-inline';">
<style>html,body{margin:0;padding:0;height:100%;overflow:hidden;background:#0b0714}iframe{border:0;width:100%;height:100%;display:block}</style>
</head><body><iframe src="${address}" allow="clipboard-read; clipboard-write"></iframe></body></html>`;
}

async function open() {
  let address;
  try {
    address = await vscode.window.withProgress({ location: vscode.ProgressLocation.Notification, title: "MapperX запускается…" }, () => ensureServer(open.force));
  } catch (e) {
    vscode.window.showErrorMessage(e.message);
    if (output) output.show(true);
    return;
  }
  if (panel) { panel.webview.html = pageHtml(address); panel.reveal(); return; }
  panel = vscode.window.createWebviewPanel("mapperx", "MapperX", vscode.ViewColumn.Active, {
    enableScripts: true, retainContextWhenHidden: true,
  });
  panel.webview.html = pageHtml(address);
  panel.onDidDispose(() => { panel = null; });
}

async function stop() {
  if (server) { server.kill(); server = null; }
  for (const address of [url, "http://127.0.0.1:8765/"]) {
    const info = address && await probe(address);
    if (info && info.token) await post(address, "/shutdown", { token: info.token });
  }
  url = null;
  if (panel) panel.dispose();
}

function activate(context) {
  const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
  item.text = "$(globe) MapperX";
  item.tooltip = "Открыть MapperX во вкладке VS Code";
  item.command = "mapperx.open";
  item.show();
  context.subscriptions.push(item,
    vscode.commands.registerCommand("mapperx.open", open),
    vscode.commands.registerCommand("mapperx.browser", async () => {
      try { vscode.env.openExternal(vscode.Uri.parse(await ensureServer())); } catch (e) { vscode.window.showErrorMessage(e.message); }
    }),
    vscode.commands.registerCommand("mapperx.restart", async () => { if (panel) panel.dispose(); open.force = true; try { await open(); } finally { open.force = false; } }),
    vscode.commands.registerCommand("mapperx.stop", stop));
}

function deactivate() {
  if (server) server.kill();
}

module.exports = { activate, deactivate };
