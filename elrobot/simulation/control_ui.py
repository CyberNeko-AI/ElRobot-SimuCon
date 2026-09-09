#!/usr/bin/env python3
"""Web-based joint-angle control UI for the ElRobot follower arm.

Serves a browser page with one slider per motor joint plus a live 3-D view of
the arm. The 3-D view is a smooth MJPEG stream (JPEG frames over
``multipart/x-mixed-replace``) and supports mouse orbit/zoom.

Usage:
    python control_ui.py                # browser UI (opens automatically)
    python control_ui.py --no-browser   # don't auto-open the browser
    python control_ui.py --port 9000    # use a specific port
    python control_ui.py --width 1280 --height 960   # render resolution
    mjpython control_ui.py --viewer     # ALSO open the native MuJoCo window

Then open http://127.0.0.1:<port> in any browser if it didn't open by itself.
Controls in the browser:
    left-drag : orbit the camera
    scroll    : zoom
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import mujoco
import mujoco.viewer  # submodule is not auto-imported by ``import mujoco``

import controller

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "elrobot_follower.xml")

ARM_JOINTS = [
    ("rev_motor_01", "J1 基座偏航 (yaw)"),
    ("rev_motor_02", "J2 肩部俯仰 (shoulder)"),
    ("rev_motor_03", "J3 肘部 (elbow)"),
    ("rev_motor_04", "J4 肘部2 (elbow2)"),
    ("rev_motor_05", "J5 腕部横滚 (wrist roll)"),
    ("rev_motor_06", "J6 腕部俯仰 (wrist pitch)"),
    ("rev_motor_07", "J7 腕部横滚2 (wrist roll2)"),
]
GRIPPER = ("rev_motor_08", "G 夹爪开合 (gripper)")

JPEG_QUALITY = 85


def _jpeg_bytes(rgb: np.ndarray, quality: int = JPEG_QUALITY) -> bytes:
    from PIL import Image  # imported lazily so --help still works without it
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).save(buf, format="JPEG",
                                                    quality=quality)
    return buf.getvalue()


class SharedState:
    """Thread-safe bridge between the HTTP server and the sim/render loop."""

    def __init__(self, joint_names: list[str]) -> None:
        self.lock = threading.Lock()
        self.targets = {n: 0.0 for n in joint_names}          # degrees
        self.actual = {n: 0.0 for n in joint_names}           # degrees
        self.frame = b""                                      # latest JPEG
        self.frame_counter = 0
        self.cam = {"azimuth": -35.0, "elevation": -16.0,
                    "distance": 1.05, "lookat": [0.0, 0.14, 0.13]}


def make_handler(state: SharedState, html: str, joint_names: list[str]):
    class Handler(BaseHTTPRequestHandler):
        # -- GET ------------------------------------------------------------
        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                body = html.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/stream":
                self._stream_mjpeg()
            elif path == "/frame":
                with state.lock:
                    body = state.frame
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/state":
                with state.lock:
                    payload = {"targets": dict(state.targets),
                               "actual": dict(state.actual),
                               "cam": dict(state.cam)}
                body = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        # -- POST -----------------------------------------------------------
        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                data = json.loads(raw.decode("utf-8"))
            except (ValueError, json.JSONDecodeError):
                self.send_error(400)
                return

            if path == "/set":
                with state.lock:
                    for name in joint_names:
                        if name in data.get("targets", {}):
                            state.targets[name] = float(data["targets"][name])
            elif path == "/cam":
                with state.lock:
                    c = state.cam
                    if "azimuth" in data:
                        c["azimuth"] = float(data["azimuth"])
                    if "elevation" in data:
                        c["elevation"] = float(data["elevation"])
                    if "distance" in data:
                        c["distance"] = float(data["distance"])
                    if "lookat" in data and len(data["lookat"]) == 3:
                        c["lookat"] = [float(x) for x in data["lookat"]]
            else:
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Content-Length", "0")
            self.end_headers()

        # -- MJPEG stream ---------------------------------------------------
        def _stream_mjpeg(self) -> None:
            boundary = "mjpegframe"
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=" + boundary)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            last_counter = -1
            try:
                while True:
                    with state.lock:
                        counter = state.frame_counter
                        jpeg = state.frame
                    if jpeg and counter != last_counter:
                        head = (f"--{boundary}\r\n"
                                f"Content-Type: image/jpeg\r\n"
                                f"Content-Length: {len(jpeg)}\r\n\r\n").encode()
                        self.wfile.write(head)
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()  # push each frame immediately
                        last_counter = counter
                    time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass  # client closed the stream

        def log_message(self, *args) -> None:
            pass

    return Handler


def build_html(joints: list[tuple[str, str, float, float]]) -> str:
    rows = []
    for name, label, lo, hi in joints:
        rows.append(
            f"""
            <div class="row">
              <label>{label}</label>
              <input type="range" id="t_{name}" min="{lo:.1f}" max="{hi:.1f}"
                     step="0.5" value="0">
              <span class="val" id="v_{name}">0.0°</span>
            </div>""")
    rows_html = "\n".join(rows)
    joints_json = json.dumps([{"name": n, "label": l} for n, l, _, _ in joints])

    return f"""<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>ElRobot 关节控制</title>
<style>
  html, body {{ margin: 0; height: 100%; }}
  body {{ font-family: -apple-system, "PingFang SC", sans-serif;
          display: flex; }}
  #panel {{ width: 360px; padding: 14px; box-sizing: border-box;
            overflow-y: auto; border-right: 1px solid #ddd; }}
  #panel h3 {{ margin: 4px 0 2px; }}
  #panel .hint {{ color: #777; font-size: 12px; margin-bottom: 10px; }}
  .row {{ margin: 9px 0; }}
  .row label {{ display: inline-block; width: 210px; font-size: 13px; }}
  .row input[type=range] {{ width: 100%; }}
  .row .val {{ font-size: 12px; color: #357; }}
  button {{ margin: 4px 6px 0 0; padding: 6px 14px; }}
  #view {{ flex: 1; display: flex; flex-direction: column; align-items: center;
           justify-content: center; background: #15171a; position: relative; }}
  #view img {{ max-width: 100%; max-height: 100%; cursor: grab;
              -webkit-user-drag: none; user-drag: none;
              -webkit-user-select: none; user-select: none; }}
  #view img.dragging {{ cursor: grabbing; }}
  #status {{ position: absolute; top: 8px; left: 12px; color: #aaa;
             font-size: 12px; pointer-events: none; }}
</style>
</head>
<body>
  <div id="panel">
    <h3>ElRobot 关节控制</h3>
    <div class="hint">拖动滑杆设定目标角度；右侧为实际角度。<br>
      夹爪 0°=闭合，最大=张开（两爪自动反向同步）。</div>
    {rows_html}
    <button onclick="setHome()">复位到零位 (Home)</button>
    <button onclick="resetView()">复位视角 (View)</button>
  </div>
  <div id="view">
    <div id="status">拖拽旋转 · 滚轮缩放</div>
    <img id="frame" src="/stream" alt="arm" draggable="false">
  </div>
<script>
const JOINTS = {joints_json};
let cam = {{azimuth: -35, elevation: -16, distance: 1.05}};

function collect() {{
  const out = {{}};
  for (const j of JOINTS)
    out[j.name] = parseFloat(document.getElementById("t_" + j.name).value);
  return out;
}}
function send() {{
  fetch("/set", {{
    method: "POST", headers: {{"Content-Type": "application/json"}},
    body: JSON.stringify({{targets: collect()}})
  }});
}}
function setHome() {{
  for (const j of JOINTS) {{
    document.getElementById("t_" + j.name).value = 0;
    document.getElementById("v_" + j.name).textContent = "0.0°";
  }}
  send();
}}
function sendCam() {{
  fetch("/cam", {{
    method: "POST", headers: {{"Content-Type": "application/json"}},
    body: JSON.stringify(cam)
  }});
}}
function resetView() {{ cam = {{azimuth: -35, elevation: -16, distance: 1.05}}; sendCam(); }}

for (const j of JOINTS)
  document.getElementById("t_" + j.name).addEventListener("input", send);

// --- mouse orbit / zoom on the streamed image ---
const img = document.getElementById("frame");
let dragging = false, lastX = 0, lastY = 0;
// disable the browser's native image drag-and-drop so left-drag orbits instead
img.addEventListener("dragstart", e => e.preventDefault());
img.addEventListener("mousedown", e => {{
  dragging = true; lastX = e.clientX; lastY = e.clientY;
  img.classList.add("dragging");
}});
window.addEventListener("mousemove", e => {{
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
  cam.azimuth -= dx * 0.4;
  cam.elevation -= dy * 0.4;
  cam.elevation = Math.max(-89, Math.min(89, cam.elevation));
  sendCam();
}});
window.addEventListener("mouseup", () => {{
  dragging = false; img.classList.remove("dragging");
}});
img.addEventListener("wheel", e => {{
  e.preventDefault();
  cam.distance *= (1 + e.deltaY * 0.0012);
  cam.distance = Math.max(0.25, Math.min(4.0, cam.distance));
  sendCam();
}}, {{passive: false}});

async function pollState() {{
  try {{
    const r = await fetch("/state");
    const s = await r.json();
    for (const j of JOINTS) {{
      const a = s.actual[j.name];
      if (a !== undefined)
        document.getElementById("v_" + j.name).textContent =
          (a >= 0 ? "+" : "") + a.toFixed(1) + "°";
    }}
  }} catch (e) {{}}
  setTimeout(pollState, 150);
}}
pollState();
</script>
</body>
</html>"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Web joint control UI")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0,
                        help="target stream frame rate")
    parser.add_argument("--viewer", action="store_true",
                        help="also open the native MuJoCo 3-D window")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    servo = controller.ServoController(model)

    all_joints = ARM_JOINTS + [GRIPPER]
    joint_names = [n for n, _ in all_joints]

    joints_with_range = []
    for name, label in all_joints:
        j = model.joint(name)
        joints_with_range.append((name, label,
                                  math.degrees(float(j.range[0])),
                                  math.degrees(float(j.range[1]))))

    state = SharedState(joint_names)
    html = build_html(joints_with_range)
    handler = make_handler(state, html, joint_names)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"[control_ui] serving at {url}  "
          f"(render {args.width}x{args.height}, JPEG, MJPEG stream)")
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    viewer = None
    if args.viewer:
        try:
            viewer = mujoco.viewer.launch_passive(model, data)
        except Exception as exc:
            print(f"[control_ui] native viewer unavailable: {exc}")

    last = time.perf_counter()
    last_render = 0.0
    period = 1.0 / max(1.0, args.fps)

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - last
            last = now

            with state.lock:
                targets = dict(state.targets)
                c = dict(state.cam)

            target_rad = {name: math.radians(targets[name])
                          for name, _ in ARM_JOINTS}
            target_rad[controller.GRIPPER_JOINT] = math.radians(targets[GRIPPER[0]])
            servo.set_targets(target_rad)

            n_steps = max(1, min(int(elapsed / model.opt.timestep), 200))
            for _ in range(n_steps):
                servo.step(data)
                mujoco.mj_step(model, data)

            with state.lock:
                for name in joint_names:
                    state.actual[name] = math.degrees(
                        float(data.qpos[model.joint(name).qposadr[0]]))

            if now - last_render >= period:
                cam.lookat[:] = c["lookat"]
                cam.distance = c["distance"]
                cam.azimuth = c["azimuth"]
                cam.elevation = c["elevation"]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=cam)
                jpeg = _jpeg_bytes(renderer.render())
                with state.lock:
                    state.frame = jpeg
                    state.frame_counter += 1
                last_render = now

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[control_ui] interrupted")
    finally:
        server.shutdown()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
