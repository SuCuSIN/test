"""Browser-based AnySkin contact monitor.

This avoids WSLg/pygame window focus issues by serving a small local web page.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import time
from typing import Callable

from manual_gripper_shell import (
    AnySkinMonitor,
    parse_float_list,
    parse_one_based_index_set,
    parse_ports,
)


HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AnySkin Contact Monitor</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, Segoe UI, Arial, sans-serif;
      background: #101418;
      color: #eef2f5;
    }
    body { margin: 0; padding: 28px; }
    h1 { margin: 0 0 8px; font-size: 28px; }
    #meta { color: #aab4bf; margin-bottom: 24px; font-size: 15px; }
    #controls {
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
      margin: 0 0 22px;
    }
    button {
      border: 1px solid #72808f;
      border-radius: 6px;
      background: #252b34;
      color: #eef2f5;
      padding: 10px 16px;
      font-size: 15px;
      font-weight: 750;
      cursor: pointer;
    }
    button:hover { background: #303844; }
    button.primary {
      border-color: #57c7aa;
      background: #17413d;
    }
    button.primary:hover { background: #1c504b; }
    button.safe {
      border-color: #f45d54;
      background: #4b2023;
    }
    button.safe:hover { background: #61292d; }
    #command-status { color: #b9c1ca; font-size: 14px; }
    #slip-monitor { max-width: 900px; padding: 16px 0; margin-bottom: 24px; border-block: 1px solid #69717d; overflow-wrap: anywhere; }
    #slip-monitor h2 { font-size: 18px; margin: 0 0 12px; }
    #slip-state { color: #e0b95d; }
    #slip-sensors { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 20px; margin: 14px 0; }
    .slip-sensor { min-width: 0; font-size: 14px; line-height: 1.6; }
    .slip-sensor progress { display: block; width: 100%; height: 14px; accent-color: #57c7aa; }
    .slip-sensor.high progress { accent-color: #e0b95d; }
    #slip-confirmed { font-weight: 750; margin-bottom: 8px; }
    #slip-confirmed.confirmed { color: #f45d54; }
    #slip-motion, #slip-reason { font-size: 14px; line-height: 1.5; }
    #slip-reason { color: #b9c1ca; }
    .name { overflow-wrap: anywhere; }
    @media (max-width: 480px) {
      body { padding: 14px; }
      #cards { grid-template-columns: minmax(0, 1fr); }
      .mag-row { grid-template-columns: 26px minmax(12px, 1fr) repeat(3, 48px); gap: 4px; font-size: 11px; }
    }
    #material-hint {
      display: none;
      max-width: 900px;
      margin: -8px 0 22px;
      border: 2px solid #69717d;
      border-radius: 8px;
      padding: 12px 14px;
      background: #20252d;
      color: #dbe4ec;
      font-size: 14px;
      line-height: 1.45;
    }
    #material-hint.soft { border-color: #57c7aa; background: #172d30; }
    #material-hint.medium { border-color: #e0b95d; background: #39321c; }
    #material-hint.hard { border-color: #f45d54; background: #421c20; }
    #material-hint.unknown { border-color: #69717d; background: #20252d; }
    #material-hint .hint-label {
      font-size: 18px;
      font-weight: 800;
      text-transform: uppercase;
      margin-right: 10px;
    }
    #material-hint .hint-detail { color: #c7d0d9; }
    #cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; max-width: 900px; }
    .card { border: 3px solid #69717d; border-radius: 8px; padding: 18px; background: #20252d; min-height: 190px; }
    .card.clear { border-color: #57c7aa; background: #172d30; }
    .card.contact { border-color: #f45d54; background: #421c20; }
    .card.lost { border-color: #8a929d; background: #343741; }
    .name { font-size: 22px; font-weight: 750; margin-bottom: 18px; }
    .strength { font-size: 24px; font-weight: 750; margin-bottom: 18px; }
    .bar { width: 100%; height: 34px; border-radius: 5px; background: #343943; overflow: hidden; margin-bottom: 18px; }
    .fill { height: 100%; width: 0%; background: #58c7a9; transition: width 80ms linear; }
    .contact .fill { background: #f45d54; }
    .state { font-size: 22px; font-weight: 800; letter-spacing: .03em; }
    .detail { color: #b9c1ca; margin-top: 12px; font-size: 13px; line-height: 1.4; }
    .mag-list { margin-top: 18px; display: grid; gap: 8px; }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 8px;
      margin: 14px 0 2px;
    }
    .metric {
      border: 1px solid #3b4551;
      border-radius: 6px;
      padding: 8px 10px;
      color: #c8d3dd;
      font-size: 12px;
      line-height: 1.35;
    }
    .metric strong {
      display: block;
      color: #eef2f5;
      font-size: 16px;
      margin-top: 3px;
      font-variant-numeric: tabular-nums;
    }
    .mag-row {
      display: grid;
      grid-template-columns: 42px 1fr 72px 72px 72px;
      align-items: center;
      gap: 10px;
      color: #dbe4ec;
      font-size: 13px;
      font-weight: 650;
    }
    .mag-head { color: #8f9ba8; font-size: 11px; text-transform: uppercase; }
    .mini-bar { height: 12px; border-radius: 4px; background: #343943; overflow: hidden; }
    .mini-fill { height: 100%; width: 0%; background: #58c7a9; transition: width 80ms linear; }
    .contact .mini-fill { background: #f45d54; }
    .mag-value { text-align: right; font-variant-numeric: tabular-nums; }
  </style>
</head>
<body>
  <h1>AnySkin contact monitor</h1>
  <div id="meta">connecting...</div>
  <div id="controls">
    <button class="primary" onclick="sendCommand('open')">Open</button>
    <button class="safe" onclick="sendCommand('close_safe')">Close Safe</button>
    <button onclick="sendCommand('empty_close_calibrate')">Calibrate Empty Close</button>
    <span id="command-status">commands ready</span>
  </div>
  <section id="slip-monitor" aria-label="Slip monitor">
    <h2>Slip monitor</h2>
    <div id="slip-state" role="status">CONNECTING</div>
    <div id="slip-sensors"></div>
    <div id="slip-confirmed">Confirmed slip: unavailable</div>
    <div id="slip-motion">Reinforcement: unavailable</div>
    <div id="slip-reason"></div>
  </section>
  <div id="material-hint"></div>
  <div id="cards"></div>
  <script>
    function sensorClass(sensor) {
      if (sensor.ignored) return "lost";
      if (sensor.lost) return "lost";
      return sensor.contact ? "contact" : "clear";
    }
    function fmt(value) {
      return Number.isFinite(value) ? value.toFixed(2) : "lost";
    }
    function pct(value, threshold) {
      if (!Number.isFinite(value) || threshold <= 0) return 0;
      return Math.max(0, Math.min(100, value / threshold * 100));
    }
    async function refresh() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        const data = await response.json();
        const rule = data.rule_description || `fixed thresholds: ${data.sensors.map(s => s.threshold.toFixed(1)).join(", ")}`;
        const calibration = data.calibration || {};
        const calibratedAt = calibration.calibrated_at_unix
          ? new Date(calibration.calibrated_at_unix * 1000).toLocaleTimeString()
          : "not calibrated";
        document.getElementById("meta").textContent =
          `${rule} | overall contact: ${data.contact} | ` +
          `zero calibration: ${calibration.samples || 0} samples @ ${calibratedAt}`;
        document.getElementById("controls").style.display = data.commands_available ? "flex" : "none";
        const cards = document.getElementById("cards");
        cards.innerHTML = "";
        data.sensors.forEach((sensor, index) => {
          const cls = sensorClass(sensor);
          const card = document.createElement("div");
          card.className = `card ${cls}`;
          const displayThreshold = Number.isFinite(sensor.effective_threshold)
            ? sensor.effective_threshold
            : sensor.threshold;
          const displayStrength = Number.isFinite(sensor.over_empty_baseline)
            ? sensor.over_empty_baseline
            : sensor.strength;
          const strengthPct = pct(displayStrength, displayThreshold);
          const magRows = (sensor.per_mag || []).map((value, magIndex) => {
            const overEmpty = sensor.per_mag_over_empty?.[magIndex];
            const barValue = Number.isFinite(overEmpty) ? overEmpty : value;
            const barThreshold = Number.isFinite(sensor.effective_margin)
              ? sensor.effective_margin
              : sensor.threshold;
            const magPct = pct(barValue, barThreshold);
            return `
              <div class="mag-row">
                <div>M${magIndex + 1}</div>
                <div class="mini-bar"><div class="mini-fill" style="width:${magPct}%"></div></div>
                <div class="mag-value">${fmt(value)}</div>
                <div class="mag-value">${fmt(sensor.per_mag_empty?.[magIndex])}</div>
                <div class="mag-value">${fmt(overEmpty)}</div>
              </div>
            `;
          }).join("");
          const magHeader = (sensor.per_mag || []).length ? `
            <div class="mag-row mag-head">
              <div>mag</div><div></div><div>now</div><div>empty</div><div>delta</div>
            </div>
          ` : "";
          card.innerHTML = `
            <div class="name">AnySkin ${index + 1} &nbsp; ${sensor.port}</div>
            <div class="strength">max strength ${fmt(sensor.strength)}</div>
            <div class="bar"><div class="fill" style="width:${strengthPct}%"></div></div>
            <div class="state">${sensor.ignored ? "IGNORED" : (sensor.lost ? "LOST" : (sensor.contact ? "CONTACT" : "CLEAR"))}</div>
            <div class="metric-grid">
              <div class="metric">fixed threshold<strong>${fmt(sensor.threshold)}</strong></div>
              <div class="metric">empty margin<strong>${fmt(sensor.effective_margin)}</strong></div>
              <div class="metric">max over empty<strong>${fmt(sensor.over_empty_baseline)}</strong></div>
            </div>
            <div class="mag-list">${magHeader}${magRows || "<div class='detail'>no per-magnet data</div>"}</div>
            <div class="detail">samples=${sensor.samples} age=${sensor.age_sec.toFixed(2)}s connected=${sensor.connected} error=${sensor.error || "none"}</div>
          `;
          cards.appendChild(card);
        });
      } catch (err) {
        document.getElementById("meta").textContent = `disconnected: ${err}`;
      }
    }
    async function sendCommand(command) {
      const status = document.getElementById("command-status");
      status.textContent = `${command} requested...`;
      try {
        const response = await fetch("/command", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({command})
        });
        const data = await response.json();
        status.textContent = data.message || (data.ok ? `${command} started` : `${command} failed`);
      } catch (err) {
        status.textContent = `command failed: ${err}`;
      }
    }
    let commandStatusBusy = false;
    async function refreshCommandStatus() {
      if (commandStatusBusy) return;
      commandStatusBusy = true;
      try {
        const response = await fetch("/command_status", {cache: "no-store", signal: AbortSignal.timeout(1500)});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const data = await response.json();
        renderSlip(data);
        if (data.message) {
          const change = (data.regrasp_change_by_sensor || []).map((v, i) =>
            `${(data.regrasp_direction_by_sensor || [])[i] || ""} ${Number(v).toFixed(1)}`).join(", ");
          const ranges = (data.regrasp_range_high || []).map((upper, i) =>
            `S${Math.floor(i / 3) + 1}G${i % 3 + 1}: ${Number(data.regrasp_range_low[i]).toFixed(1)}..${Number(upper).toFixed(1)}`
          ).join("; ");
          const regrasp = data.regrasp_enabled
            ? ` | Regrasp: ${data.regrasp_state} | outside range: ${change} / ${data.regrasp_change_threshold} | attempts: ${data.regrasp_attempts}/3 | ${ranges}`
            : ("regrasp_enabled" in data ? " | Regrasp: OFF" : "");
          document.getElementById("command-status").textContent = data.message + regrasp;
        }
        renderMaterialHint(data.material_hint);
      } catch (err) {
        renderSlip(null);
      } finally {
        commandStatusBusy = false;
      }
    }
    function renderSlip(data) {
      const monitor = data?.slip_monitor;
      const state = document.getElementById("slip-state");
      const confirmed = document.getElementById("slip-confirmed");
      const sensors = document.getElementById("slip-sensors");
      sensors.replaceChildren();
      confirmed.className = "";
      if (!monitor) {
        state.textContent = "SLIP STATUS UNAVAILABLE";
        confirmed.textContent = "Confirmed slip: unavailable";
        document.getElementById("slip-motion").textContent = "Reinforcement: unavailable";
        document.getElementById("slip-reason").textContent = "No live controller status";
        return;
      }
      state.textContent = `${monitor.state} | Slip episodes: ${monitor.slip_count ?? 0} this grasp / ${monitor.total_slip_count ?? 0} session`;
      monitor.sensors.forEach((sensor, index) => {
        const row = document.createElement("div");
        row.className = `slip-sensor${sensor.high ? " high" : ""}`;
        const score = document.createElement("div");
        score.textContent = `Sensor ${index + 1}: ${Number.isFinite(sensor.score) ? sensor.score.toFixed(3) : "unavailable"} / ${monitor.threshold.toFixed(2)}`;
        const bar = document.createElement("progress");
        bar.max = 1;
        bar.value = Number.isFinite(sensor.score) ? sensor.score : 0;
        bar.setAttribute("aria-label", `Sensor ${index + 1} model score`);
        const detail = document.createElement("div");
        detail.textContent = `${sensor.high ? "MODEL HIGH" : "MODEL LOW / UNAVAILABLE"} | consecutive ${sensor.count}/${monitor.required_samples} | contact ${sensor.contact ? "YES" : "NO"}`;
        row.append(score, bar, detail);
        sensors.appendChild(row);
      });
      confirmed.className = "";
      confirmed.textContent = monitor.confirmed_this_grasp
        ? `Slip history: last confirmed ${monitor.last_confirmed_age_sec.toFixed(1)}s ago (not current activity)`
        : "Slip history: no confirmed event in this grasp";
      document.getElementById("slip-motion").textContent =
        `Reinforcement ${monitor.reinforcement_inhibited ? "INHIBITED" : "status"} | attempts: ${data.regrasp_attempts ?? 0}/3 | ${data.regrasp_motion_result || "not commanded"}`;
      document.getElementById("slip-reason").textContent =
        monitor.reason + (monitor.waiting_for_rearm
          ? ` | slip-end clear samples ${monitor.clear_samples}/3`
          : monitor.state === "CONTACT SETTLING"
            ? ` | settling ${monitor.settling_remaining_sec.toFixed(2)}s remaining`
            : "");
    }
    function renderMaterialHint(hint) {
      const card = document.getElementById("material-hint");
      if (!hint) {
        card.style.display = "none";
        return;
      }
      const label = hint.label || "unknown";
      const peak = Number.isFinite(hint.peak_delta) ? hint.peak_delta.toFixed(1) : "-";
      const avg = Number.isFinite(hint.average_delta) ? hint.average_delta.toFixed(1) : "-";
      const rise = Number.isFinite(hint.rise_rate) ? hint.rise_rate.toFixed(1) : "-";
      const ratio = Number.isFinite(hint.contact_ratio) ? `${(hint.contact_ratio * 100).toFixed(0)}%` : "-";
      const elapsed = Number.isFinite(hint.elapsed_sec) ? `${hint.elapsed_sec.toFixed(2)}s` : "-";
      const basis = hint.classification_basis || "-";
      const note = hint.note || "";
      card.className = label;
      card.style.display = "block";
      card.innerHTML = `
        <span class="hint-label">Object hint: ${label}</span>
        <span class="hint-detail">
          rise rate ${rise}/s, peak delta ${peak}, average delta ${avg},
          close ratio ${ratio}, elapsed ${elapsed}, basis ${basis}.
          ${note}
        </span>
      `;
    }
    refresh();
    refreshCommandStatus();
    setInterval(refresh, 100);
    setInterval(refreshCommandStatus, 300);
  </script>
</body>
</html>
"""


class AnySkinWebHandler(BaseHTTPRequestHandler):
    monitor: AnySkinMonitor
    command_handler: Callable[[str], dict] | None = None
    command_status_handler: Callable[[], dict] | None = None

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/status":
            body = json.dumps(self._status()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/command_status":
            status = {"ok": False, "message": "commands unavailable"}
            if self.command_status_handler is not None:
                status = self.command_status_handler()
            self._send_json(status)
            return

        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/command":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = self.rfile.read(min(length, 4096))
            data = json.loads(payload.decode("utf-8") or "{}")
            command = str(data.get("command", "")).strip().lower()
        except Exception as exc:
            self._send_json({"ok": False, "message": f"bad command request: {exc}"}, 400)
            return

        if self.command_handler is None:
            self._send_json({"ok": False, "message": "commands unavailable"}, 503)
            return

        result = self.command_handler(command)
        status_code = 200 if result.get("ok", False) else 409
        self._send_json(result, status_code)

    def log_message(self, format: str, *args) -> None:  # noqa: A002
        return

    def _send_json(self, value: dict, status_code: int = 200) -> None:
        body = json.dumps(value).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _status(self) -> dict:
        monitor = self.monitor
        now = time.monotonic()
        command_status = {}
        if self.command_status_handler is not None:
            try:
                command_status = self.command_status_handler()
            except Exception as exc:
                command_status = {"ok": False, "message": f"command status error: {exc}"}
        expected_matrix = command_status.get("expected_magnet_strengths") or []
        empty_margin = command_status.get("empty_baseline_margin")
        empty_baseline_available = bool(
            command_status.get("empty_baseline_available") and expected_matrix
        )
        try:
            empty_margin_value = float(empty_margin)
        except (TypeError, ValueError):
            empty_margin_value = float("nan")

        def json_number(value: float | None) -> float | None:
            if value is None:
                return None
            try:
                value = float(value)
            except (TypeError, ValueError):
                return None
            if math.isnan(value) or math.isinf(value):
                return None
            return value

        sensors = []
        contact = False
        all_per_mag = monitor.magnet_strengths()
        for index, (port, stream) in enumerate(zip(monitor.ports, monitor.streams)):
            threshold = monitor.threshold_for_index(index)
            ignored = index in monitor.ignored_indexes
            last_time = getattr(stream, "last_sample_time", 0.0)
            age_sec = 999.0 if last_time <= 0.0 else now - last_time
            per_mag = all_per_mag[index] if index < len(all_per_mag) else []
            finite_per_mag = [value for value in per_mag if not math.isnan(value)]
            strength = max(finite_per_mag) if finite_per_mag else float("nan")
            expected_row = []
            if index < len(expected_matrix) and isinstance(expected_matrix[index], list):
                expected_row = [json_number(value) for value in expected_matrix[index]]
            per_mag_over_empty = []
            for mag_index, value in enumerate(per_mag):
                expected = expected_row[mag_index] if mag_index < len(expected_row) else None
                if expected is None or math.isnan(value):
                    per_mag_over_empty.append(None)
                else:
                    per_mag_over_empty.append(float(value) - float(expected))
            finite_over_empty = [
                value for value in per_mag_over_empty if value is not None and not math.isnan(value)
            ]
            over_empty_baseline = max(finite_over_empty) if finite_over_empty else float("nan")
            lost = math.isnan(strength)
            if empty_baseline_available and not math.isnan(empty_margin_value):
                sensor_contact = (
                    not ignored
                    and bool(finite_over_empty)
                    and over_empty_baseline >= empty_margin_value
                )
            else:
                sensor_contact = not ignored and not lost and strength >= threshold
            contact = contact or sensor_contact
            sensors.append(
                {
                    "port": port,
                    "strength": json_number(strength),
                    "per_mag": [json_number(value) for value in per_mag],
                    "per_mag_empty": expected_row,
                    "per_mag_over_empty": per_mag_over_empty,
                    "over_empty_baseline": json_number(over_empty_baseline),
                    "effective_margin": json_number(empty_margin_value)
                    if empty_baseline_available
                    else None,
                    "effective_threshold": json_number(empty_margin_value)
                    if empty_baseline_available
                    else json_number(threshold),
                    "threshold": threshold,
                    "contact": sensor_contact,
                    "lost": lost,
                    "ignored": ignored,
                    "samples": getattr(stream, "sample_cnt", 0),
                    "age_sec": age_sec,
                    "connected": bool(getattr(stream, "connected", False)),
                    "error": getattr(stream, "last_error", ""),
                }
            )
        return {
            "contact": contact,
            "num_mags": monitor.num_mags,
            "commands_available": self.command_handler is not None,
            "rule_description": command_status.get("rule_description"),
            "calibration": monitor.calibration_snapshot(),
            "command_status": command_status,
            "sensors": sensors,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", required=True, help="Comma-separated AnySkin ports")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--num-mags", type=int, default=5)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--warmup-sec", type=float, default=1.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=8.0)
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--ascii", action="store_true")
    parser.add_argument("--stale-timeout-sec", type=float, default=1.0)
    parser.add_argument("--reconnect-delay-sec", type=float, default=0.5)
    parser.add_argument("--set-control-lines", action="store_true")
    parser.add_argument("--filter-alpha", type=float, default=0.35)
    parser.add_argument("--filter-release-alpha", type=float, default=0.75)
    parser.add_argument("--spike-step-limit", type=float, default=80.0)
    parser.add_argument("--contact-threshold", type=float, default=110.0)
    parser.add_argument("--contact-thresholds", default="")
    parser.add_argument("--ignore-anyskin-indexes", default="")
    parser.add_argument("--calibration-samples", type=int, default=20)
    args = parser.parse_args()

    monitor = AnySkinMonitor(
        parse_ports(args.ports),
        args.num_mags,
        args.baudrate,
        args.contact_threshold,
        parse_float_list(args.contact_thresholds),
        parse_one_based_index_set(args.ignore_anyskin_indexes),
        args.calibration_samples,
        args.warmup_sec,
        args.startup_timeout_sec,
        args.allow_partial,
        False,
        30.0,
        not args.ascii,
        args.stale_timeout_sec,
        args.reconnect_delay_sec,
        args.set_control_lines,
        args.filter_alpha,
        args.filter_release_alpha,
        args.spike_step_limit,
    )

    try:
        monitor.start()
        AnySkinWebHandler.monitor = monitor
        server = ThreadingHTTPServer((args.host, args.web_port), AnySkinWebHandler)
        print(f"AnySkin web monitor: http://{args.host}:{args.web_port}")
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            server.server_close()
        except Exception:
            pass
        monitor.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
