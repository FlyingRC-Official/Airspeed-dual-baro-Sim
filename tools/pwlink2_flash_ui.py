#!/usr/bin/env python3
import argparse
import binascii
import json
import os
import shutil
import tempfile
import struct
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_HOME = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")).expanduser()
PIO = os.environ.get("PIO") or shutil.which("pio") or "pio"
OPENOCD = (
    os.environ.get("OPENOCD")
    or shutil.which("openocd")
    or str(PLATFORMIO_HOME / "packages" / "tool-openocd" / "bin" / "openocd")
)
OPENOCD_SCRIPTS = os.environ.get(
    "OPENOCD_SCRIPTS",
    str(PLATFORMIO_HOME / "packages" / "tool-openocd" / "openocd" / "scripts"),
)

CAL_BASE = 0x0800F000
CAL_SIZE = 0x1000
RECORD_LEN = 48
MAGIC = 0x46524153

state = {
    "count": 0,
    "current": 1,
    "running": False,
    "last_log": "",
    "results": [],
}
state_lock = Lock()


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>PWLINK2 批量烧录</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f6f3;
      --panel: #ffffff;
      --ink: #1d2329;
      --muted: #65717d;
      --line: #d8ded6;
      --accent: #1f7a5a;
      --accent-ink: #ffffff;
      --warn: #a65f00;
      --bad: #b42318;
      --good-bg: #e9f6ef;
      --bad-bg: #fff0ed;
      --shadow: 0 14px 40px rgba(37, 47, 38, 0.11);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--ink);
    }
    main {
      max-width: 1120px;
      margin: 0 auto;
      padding: 28px 22px 36px;
    }
    header {
      display: flex;
      justify-content: space-between;
      gap: 20px;
      align-items: flex-end;
      margin-bottom: 20px;
    }
    h1 {
      margin: 0;
      font-size: 30px;
      line-height: 1.12;
      font-weight: 720;
      letter-spacing: 0;
    }
    .sub {
      margin-top: 8px;
      color: var(--muted);
      font-size: 14px;
    }
    .status {
      display: flex;
      align-items: center;
      gap: 9px;
      padding: 8px 11px;
      border: 1px solid var(--line);
      background: var(--panel);
      border-radius: 8px;
      font-size: 13px;
      color: var(--muted);
      white-space: nowrap;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--accent);
    }
    .dot.running {
      animation: pulse 1s ease-in-out infinite;
      background: var(--warn);
    }
    @keyframes pulse {
      0%, 100% { transform: scale(0.85); opacity: 0.65; }
      50% { transform: scale(1.15); opacity: 1; }
    }
    .grid {
      display: grid;
      grid-template-columns: minmax(280px, 0.9fr) minmax(360px, 1.3fr);
      gap: 16px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
    }
    .control {
      padding: 20px;
    }
    .current {
      display: grid;
      grid-template-columns: 86px 1fr;
      gap: 16px;
      align-items: center;
      margin-bottom: 18px;
    }
    .big {
      display: grid;
      place-items: center;
      height: 86px;
      border-radius: 8px;
      background: #eef3ec;
      border: 1px solid var(--line);
      font-size: 38px;
      font-weight: 760;
      color: var(--accent);
    }
    .label {
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 4px;
    }
    .title {
      font-size: 18px;
      font-weight: 680;
    }
    .hint {
      color: var(--muted);
      font-size: 14px;
      line-height: 1.45;
      margin-top: 7px;
    }
    button {
      width: 100%;
      height: 48px;
      border: 0;
      border-radius: 8px;
      font-size: 16px;
      font-weight: 680;
      cursor: pointer;
    }
    button.primary {
      background: var(--accent);
      color: var(--accent-ink);
    }
    button.secondary {
      margin-top: 10px;
      background: #edf0eb;
      color: #34413a;
      border: 1px solid var(--line);
    }
    .step-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    button.compact {
      height: 42px;
    }
    button:disabled {
      cursor: not-allowed;
      opacity: 0.55;
    }
    .summary {
      margin-top: 18px;
      display: grid;
      gap: 9px;
    }
    .metric {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      border-top: 1px solid var(--line);
      padding-top: 9px;
      color: var(--muted);
      font-size: 13px;
    }
    .metric strong {
      color: var(--ink);
      font-weight: 680;
    }
    .results {
      padding: 0;
      overflow: hidden;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      text-align: left;
      padding: 12px 14px;
      border-bottom: 1px solid var(--line);
      white-space: nowrap;
    }
    th {
      background: #f8faf7;
      color: var(--muted);
      font-weight: 660;
    }
    tr.ok td:first-child { color: var(--accent); font-weight: 720; }
    tr.fail td:first-child { color: var(--bad); font-weight: 720; }
    .log {
      grid-column: 1 / -1;
      overflow: hidden;
    }
    .log-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 12px 14px;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 13px;
    }
    pre {
      margin: 0;
      max-height: 280px;
      overflow: auto;
      padding: 14px;
      background: #161b1f;
      color: #dfe7de;
      font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    @media (max-width: 820px) {
      header { align-items: stretch; flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
      th, td { padding: 10px 9px; }
      .results { overflow-x: auto; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>PWLINK2 批量烧录</h1>
        <div class="sub">烧录 STM32G031 固件，等待校准，读取 Flash 校验记录。</div>
      </div>
      <div class="status"><span id="dot" class="dot"></span><span id="statusText">空闲</span></div>
    </header>

    <div class="grid">
      <section class="control">
        <div class="current">
          <div class="big" id="currentNo">1</div>
          <div>
            <div class="label">当前板号</div>
            <div class="title" id="currentTitle">第 1 片</div>
            <div class="hint" id="hint">压好烧录针并保持静止，然后点击开始。</div>
          </div>
        </div>
        <button class="primary" id="flashBtn">烧录当前板</button>
        <div class="step-row">
          <button class="secondary compact" id="prevBtn">上一片</button>
          <button class="secondary compact" id="nextBtn">下一片</button>
        </div>
        <button class="secondary" id="retryBtn">重试并重新校准当前板</button>
        <button class="secondary" id="resetBtn">重新开始计数</button>
        <div class="summary">
          <div class="metric"><span>完成数量</span><strong id="doneCount">0</strong></div>
          <div class="metric"><span>最近 offset</span><strong id="lastOffset">-</strong></div>
          <div class="metric"><span>最近 stddev</span><strong id="lastStddev">-</strong></div>
        </div>
      </section>

      <section class="results">
        <table>
          <thead>
            <tr>
              <th>板号</th>
              <th>结果</th>
              <th>offset Pa</th>
              <th>stddev Pa</th>
              <th>样本</th>
              <th>温度 C</th>
            </tr>
          </thead>
          <tbody id="rows"></tbody>
        </table>
      </section>

      <section class="log">
        <div class="log-head">
          <span>运行日志</span>
          <span id="dumpPath"></span>
        </div>
        <pre id="log">等待操作...</pre>
      </section>
    </div>
  </main>

  <script>
    const flashBtn = document.getElementById('flashBtn');
    const retryBtn = document.getElementById('retryBtn');
    const resetBtn = document.getElementById('resetBtn');
    const prevBtn = document.getElementById('prevBtn');
    const nextBtn = document.getElementById('nextBtn');
    const logEl = document.getElementById('log');
    const rowsEl = document.getElementById('rows');

    function fmt(v, digits = 3) {
      return typeof v === 'number' ? v.toFixed(digits) : '-';
    }

    function setBusy(running) {
      flashBtn.disabled = running;
      retryBtn.disabled = running;
      resetBtn.disabled = running;
      prevBtn.disabled = running;
      nextBtn.disabled = running;
      document.getElementById('dot').classList.toggle('running', running);
      document.getElementById('statusText').textContent = running ? '正在烧录 / 校准 / 读取' : '空闲';
    }

    function render(s) {
      setBusy(s.running);
      const done = s.results.filter(r => r.ok).length;
      const limited = s.count > 0;
      const finished = limited && s.current > s.count;
      document.getElementById('currentNo').textContent = s.current;
      document.getElementById('currentTitle').textContent = `第 ${s.current} 片`;
      document.getElementById('doneCount').textContent = limited ? `${done} / ${s.count}` : `${done}`;
      const latest = s.results[s.results.length - 1];
      document.getElementById('lastOffset').textContent = latest && latest.cal ? `${fmt(latest.cal.offset_pa)} Pa` : '-';
      document.getElementById('lastStddev').textContent = latest && latest.cal ? `${fmt(latest.cal.stddev_pa)} Pa` : '-';
      document.getElementById('hint').textContent = finished ? '设定批次已完成。需要继续就重新开始，或用不带 --count 的方式启动。' : '压好烧录针并保持静止，然后点击开始。';
      flashBtn.textContent = finished ? '设定批次完成' : `烧录第 ${s.current} 片`;
      flashBtn.disabled = s.running || finished;
      logEl.textContent = s.last_log || '等待操作...';
      document.getElementById('dumpPath').textContent = latest && latest.dump ? latest.dump : '';
      rowsEl.innerHTML = s.results.map(r => `
        <tr class="${r.ok ? 'ok' : 'fail'}">
          <td>#${r.board}</td>
          <td>${r.ok ? 'OK' : 'FAIL'}</td>
          <td>${r.cal ? fmt(r.cal.offset_pa) : '-'}</td>
          <td>${r.cal ? fmt(r.cal.stddev_pa) : '-'}</td>
          <td>${r.cal ? r.cal.samples : '-'}</td>
          <td>${r.cal ? fmt(r.cal.temp_c) : '-'}</td>
        </tr>
      `).join('');
    }

    async function refresh() {
      const res = await fetch('/api/status');
      render(await res.json());
    }

    async function flash(retry = false) {
      setBusy(true);
      logEl.textContent = retry ? '开始重试：将先擦除校准区再烧录...' : '开始执行...';
      const res = await fetch('/api/flash', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ retry })
      });
      render(await res.json());
    }

    async function resetBatch() {
      const res = await fetch('/api/reset', { method: 'POST' });
      render(await res.json());
    }

    async function step(delta) {
      const res = await fetch('/api/step', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ delta })
      });
      render(await res.json());
    }

    flashBtn.addEventListener('click', () => flash(false));
    retryBtn.addEventListener('click', () => flash(true));
    resetBtn.addEventListener('click', resetBatch);
    prevBtn.addEventListener('click', () => step(-1));
    nextBtn.addEventListener('click', () => step(1));
    refresh();
  </script>
</body>
</html>
"""


def run_command(args, timeout=60):
    try:
        proc = subprocess.run(
            args,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
        )
        return proc.returncode, proc.stdout
    except FileNotFoundError as exc:
        return 127, f"Command not found: {args[0]}\n{exc}\n"


def parse_calibration(path):
    data = Path(path).read_bytes()
    valid = []
    for index, offset in enumerate(range(0, len(data) - RECORD_LEN + 1, RECORD_LEN)):
        rec = data[offset : offset + RECORD_LEN]
        if rec == b"\xff" * RECORD_LEN:
            continue
        version, length, seq, diff, b1, b2, temp, stddev, samples, flags, reserved, crc, mg = struct.unpack(
            "<HHIfffffIIIII", rec
        )
        calc = binascii.crc32(rec[:40]) & 0xFFFFFFFF
        ok = (
            mg == MAGIC
            and version == 1
            and length == RECORD_LEN
            and samples != 0
            and calc == crc
            and -10000.0 < diff < 10000.0
        )
        if ok:
            valid.append(
                {
                    "slot": index,
                    "addr": f"0x{CAL_BASE + offset:08X}",
                    "seq": seq,
                    "offset_pa": diff,
                    "baro1_mean_pa": b1,
                    "baro2_mean_pa": b2,
                    "temp_c": temp,
                    "stddev_pa": stddev,
                    "samples": samples,
                    "crc_ok": calc == crc,
                }
            )
    return max(valid, key=lambda item: item["seq"]) if valid else None


def erase_calibration_area():
    erase_cmd = [
        OPENOCD,
        "-s",
        OPENOCD_SCRIPTS,
        "-f",
        "interface/cmsis-dap.cfg",
        "-f",
        "target/stm32g0x.cfg",
        "-c",
        "init",
        "-c",
        "reset halt",
        "-c",
        "flash erase_address 0x0800F000 0x1000",
        "-c",
        "reset run",
        "-c",
        "shutdown",
    ]
    return run_command(erase_cmd, timeout=30)


def flash_board(board, force_recalibrate=False):
    logs = []
    start = time.strftime("%Y%m%d-%H%M%S")
    dump_dir = ROOT / ".pio" / "cal-dump"
    dump_dir.mkdir(parents=True, exist_ok=True)
    dump_path = dump_dir / f"calibration_flash_ui_board{board}_{start}.bin"
    tmp_dump_path = Path(tempfile.gettempdir()) / f"pwlink2_cal_board{board}_{start}.bin"

    if force_recalibrate:
        logs.append("Retry requested: erasing calibration Flash area 0x0800F000-0x0800FFFF before upload.\n")
        code, out = erase_calibration_area()
        logs.append(out)
        if code != 0:
            return {
                "board": board,
                "ok": False,
                "cal": None,
                "dump": None,
                "log": "\n".join(logs),
            }

    code, out = run_command([PIO, "run", "-e", "stm32g031g8u6-pwlink2", "-t", "upload"], timeout=90)
    logs.append(out)
    if code != 0 or "Verified OK" not in out:
        return {
            "board": board,
            "ok": False,
            "cal": None,
            "dump": None,
            "log": "\n".join(logs),
        }

    logs.append("\nWaiting 5 seconds for startup calibration...\n")
    time.sleep(5)

    openocd_cmd = [
        OPENOCD,
        "-s",
        OPENOCD_SCRIPTS,
        "-f",
        "interface/cmsis-dap.cfg",
        "-f",
        "target/stm32g0x.cfg",
        "-c",
        "init",
        "-c",
        "reset halt",
        "-c",
        f"dump_image {tmp_dump_path} 0x0800F000 0x1000",
        "-c",
        "reset run",
        "-c",
        "shutdown",
    ]
    code, out = run_command(openocd_cmd, timeout=30)
    logs.append(out)
    if code == 0 and tmp_dump_path.exists():
        dump_path.write_bytes(tmp_dump_path.read_bytes())
    cal = parse_calibration(dump_path) if code == 0 and dump_path.exists() else None
    ok = code == 0 and cal is not None
    return {
        "board": board,
        "ok": ok,
        "cal": cal,
        "dump": str(dump_path.relative_to(ROOT)) if dump_path.exists() else None,
        "log": "\n".join(logs),
    }


class Handler(BaseHTTPRequestHandler):
    def do_HEAD(self):
        if self.path == "/" or self.path.startswith("/?"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            return
        if self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            return
        self.send_error(404)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            self.send_html(HTML)
            return
        if self.path == "/api/status":
            self.send_json(snapshot())
            return
        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/reset":
            with state_lock:
                state["current"] = 1
                state["running"] = False
                state["last_log"] = ""
                state["results"] = []
            self.send_json(snapshot())
            return
        if path == "/api/step":
            length = int(self.headers.get("Content-Length", "0") or "0")
            payload = {}
            if length:
                try:
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                except json.JSONDecodeError:
                    payload = {}
            delta = int(payload.get("delta", 0))
            with state_lock:
                if not state["running"]:
                    state["current"] = max(1, state["current"] + delta)
            self.send_json(snapshot())
            return
        if path == "/api/flash":
            length = int(self.headers.get("Content-Length", "0") or "0")
            payload = {}
            if length:
                try:
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                except json.JSONDecodeError:
                    payload = {}
            retry = bool(payload.get("retry"))
            with state_lock:
                if state["running"]:
                    self.send_json(snapshot(), status=409)
                    return
                board = state["current"]
                if state["count"] > 0 and board > state["count"]:
                    self.send_json(snapshot())
                    return
                state["running"] = True
                state["last_log"] = f"Starting board #{board}{' with forced recalibration' if retry else ''}...\n"
            try:
                result = flash_board(board, force_recalibrate=retry)
            except Exception as exc:
                result = {
                    "board": board,
                    "ok": False,
                    "cal": None,
                    "dump": None,
                    "log": f"Unhandled error while flashing board #{board}: {exc!r}",
                }
            with state_lock:
                state["running"] = False
                state["last_log"] = result["log"]
                state["results"].append(result)
                if result["ok"]:
                    state["current"] = board + 1
            self.send_json(snapshot())
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        return

    def send_html(self, body):
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, status=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def snapshot():
    with state_lock:
        return json.loads(json.dumps(state, ensure_ascii=False))


def main():
    global OPENOCD, OPENOCD_SCRIPTS, PIO

    parser = argparse.ArgumentParser(description="PWLINK2 batch flashing UI")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--count", type=int, default=0, help="Optional batch limit. Omit or set 0 for unlimited.")
    parser.add_argument("--pio", default=PIO, help="PlatformIO executable. Defaults to PIO env var or PATH lookup.")
    parser.add_argument("--openocd", default=OPENOCD, help="OpenOCD executable. Defaults to OPENOCD env var or PlatformIO package path.")
    parser.add_argument(
        "--openocd-scripts",
        default=OPENOCD_SCRIPTS,
        help="OpenOCD scripts directory. Defaults to OPENOCD_SCRIPTS env var or PlatformIO package path.",
    )
    args = parser.parse_args()
    PIO = args.pio
    OPENOCD = args.openocd
    OPENOCD_SCRIPTS = args.openocd_scripts

    with state_lock:
        state["count"] = args.count
        state["current"] = 1

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"PWLINK2 UI: http://{args.host}:{args.port}")
    print("Press Ctrl-C to stop.")
    server.serve_forever()


if __name__ == "__main__":
    main()
