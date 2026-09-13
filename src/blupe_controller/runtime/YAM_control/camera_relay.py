"""Robot-site camera relay: serve local webcams as multipart MJPEG over HTTP.

The M2 camera path (PLAN "Remote topology"): cameras stay at the robot site; the eval on the
operator's machine pulls these streams (CameraGrabber opens the URLs), composites the HUD,
and re-encodes to the headset. This keeps the Quest talking only to the operator node.

Run on the Orin (cv2 lives in the conda `xr` env, NOT the i2rt venv):
    ~/miniforge3/envs/xr/bin/python ~/blupe-evals/YAM_control/camera_relay.py --devices 0 1 2

Consume: one stream per device, e.g. http://<host>:8089/0, /1, /2.
"""

import argparse
import json
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

BOUNDARY = b"--frame"
CAPTURE_BACKEND = getattr(cv2, "CAP_AVFOUNDATION", None) if sys.platform == "darwin" else None
# AVFoundation is unreliable when identical USB cameras are released/opened in
# parallel.  A single relay owns all devices, so serialize recovery as well as
# startup and let each capture thread continue dropping frames independently.
CAPTURE_REOPEN_LOCK = threading.Lock()


class Cam:
    """Capture thread holding the latest JPEG for one device (drop frames, never queue)."""

    def __init__(self, dev, width, height, fps, quality, stale_timeout_s, reopen_after_failures, reopen_delay_s):
        self.dev = dev
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg = None
        self.lock = threading.Lock()
        self.quality = quality
        self.stale_timeout_s = float(stale_timeout_s)
        self.reopen_after_failures = max(1, int(reopen_after_failures))
        self.reopen_delay_s = max(0.0, float(reopen_delay_s))
        self.cap = None
        self.ok = False
        self.frame_count = 0
        self.last_frame_mono = 0.0
        self.last_frame_wall = 0.0
        self.error = ""
        self.consecutive_failures = 0
        self._open_capture()
        threading.Thread(target=self._run, daemon=True).start()
        print(f"[relay] /dev/video{dev}: {'open' if self.ok else 'FAILED'}", flush=True)

    def _open_capture(self):
        if self.cap is not None:
            self.cap.release()
        if CAPTURE_BACKEND is None:
            cap = cv2.VideoCapture(self.dev)
        else:
            cap = cv2.VideoCapture(self.dev, CAPTURE_BACKEND)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        self.ok = cap.isOpened()
        self.consecutive_failures = 0
        if not self.ok:
            self.error = "open failed"
        else:
            self.error = ""

    def _reopen_capture(self, reason):
        print(f"[relay] /dev/video{self.dev}: reopening after {reason}", flush=True)
        with self.lock:
            self.ok = False
            self.error = str(reason)
            # Keep the last frame and timestamp. Existing MJPEG clients will
            # wait once it exceeds stale_timeout_s, but a sub-threshold reopen
            # no longer turns a harmless USB hiccup into an immediate 503.
        with CAPTURE_REOPEN_LOCK:
            time.sleep(self.reopen_delay_s)
            self._open_capture()
        print(f"[relay] /dev/video{self.dev}: {'reopened' if self.ok else 'reopen FAILED'}", flush=True)

    def fresh_jpeg(self):
        with self.lock:
            jpeg = self.jpeg
            last_frame_mono = self.last_frame_mono
            error = self.error
        if jpeg is None or last_frame_mono <= 0:
            return None, "no frame yet" if not error else error
        age_s = time.monotonic() - last_frame_mono
        if age_s > self.stale_timeout_s:
            return None, f"stale frame age={age_s:.2f}s"
        return jpeg, ""

    def status(self):
        with self.lock:
            age_s = None if self.last_frame_mono <= 0 else round(time.monotonic() - self.last_frame_mono, 3)
            return {
                "device": self.dev,
                "opened": bool(self.ok),
                "fresh": bool(self.jpeg is not None and age_s is not None and age_s <= self.stale_timeout_s),
                "frame_count": self.frame_count,
                "frame_age_s": age_s,
                "last_frame_wall": self.last_frame_wall,
                "error": self.error,
                "consecutive_failures": self.consecutive_failures,
            }

    def _run(self):
        while True:
            if self.cap is None or not self.ok:
                self._reopen_capture(self.error or "not open")
                continue
            ok, bgr = self.cap.read()
            if not ok or bgr is None:
                self.consecutive_failures += 1
                with self.lock:
                    self.error = f"read failed x{self.consecutive_failures}"
                    last_frame_mono = self.last_frame_mono
                # One delayed AVFoundation read can already exceed the stale
                # threshold. Reopen only after repeated read failures; otherwise
                # a single dropped frame causes a reopen/503/reconnect cascade.
                if self.consecutive_failures >= self.reopen_after_failures:
                    self._reopen_capture(self.error)
                time.sleep(0.1)
                continue
            self.consecutive_failures = 0
            sequence = self.frame_count + 1
            # Staleness diagnostic: burn capture wall-time + frame counter into the pixels, so
            # EVERY consumer (browser, eval, headset) displays how old the frame it shows is.
            capture_mono = time.monotonic()
            now_wall = time.time()
            stamp = f"{time.strftime('%H:%M:%S')}.{int((now_wall % 1) * 10)} #{sequence}"
            cv2.rectangle(bgr, (0, 0), (250, 28), (0, 0, 0), -1)
            cv2.putText(bgr, stamp, (6, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()
                    self.frame_count = sequence
                    self.last_frame_mono = capture_mono
                    self.last_frame_wall = now_wall
                    self.error = ""


CAMS = {}
ALIGNMENT_HTML_PATH = Path(__file__).with_name("camera_alignment.html")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):           # quiet
        pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in {"/align", "/alignment"}:
            try:
                body = ALIGNMENT_HTML_PATH.read_bytes()
            except OSError as exc:
                self.send_error(500, f"alignment UI unavailable: {exc}")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path.endswith('/snapshot.jpg'):
            key = path.strip('/').split('/')[0]
            cam = CAMS.get(key)
            if cam is None:
                self.send_error(404)
                return
            with cam.lock:
                jpeg, captured, sequence = cam.jpeg, cam.last_frame_mono, cam.frame_count
            if jpeg is None or time.monotonic() - captured > cam.stale_timeout_s:
                self.send_error(503, 'No fresh camera frame')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('X-Capture-Monotonic', str(captured))
            self.send_header('X-Frame-Sequence', str(sequence))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        key = path.strip("/")
        if key in {"", "health", "status"}:
            payload = {
                "ok": all(cam.status()["fresh"] for cam in CAMS.values()),
                "cameras": {name: cam.status() for name, cam in CAMS.items()},
            }
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(200 if payload["ok"] else 503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        cam = CAMS.get(key)
        if cam is None:
            self.send_error(404, f"no camera {key!r}; have {sorted(CAMS)}")
            return
        jpeg, stale_reason = cam.fresh_jpeg()
        if jpeg is None:
            detail = stale_reason or cam.error or "not open"
            self.send_error(503, f"camera {key!r} unavailable: {detail}")
            return
        # Small send buffer + drop-on-backpressure: a slow consumer (Wi-Fi dip, cloud-relay
        # congestion) must get FEWER frames, not OLDER frames. Blocking writes with a big
        # default buffer queue seconds of video in TCP and the stream never catches up.
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        print(f"[relay] client {self.client_address} -> /{key}", flush=True)
        last = None
        try:
            while True:
                jpeg, stale_reason = cam.fresh_jpeg()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                if jpeg is last:                       # only NEW frames (fps trap: re-sending
                    time.sleep(0.005)                  # dupes wastes the path's bandwidth)
                    continue
                _, writable, _ = select.select([], [self.connection], [], 0)
                if not writable:                       # client stalled -> drop, stay current
                    time.sleep(1.0 / 30)
                    continue
                last = jpeg
                self.wfile.write(BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            print(f"[relay] client {self.client_address} left /{key}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--devices", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8089)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--stale-timeout-s", type=float, default=2.0)
    ap.add_argument("--reopen-after-failures", type=int, default=5)
    ap.add_argument("--reopen-delay-s", type=float, default=1.0)
    args = ap.parse_args()

    for d in args.devices:
        CAMS[str(d)] = Cam(
            d,
            args.width,
            args.height,
            args.fps,
            args.quality,
            args.stale_timeout_s,
            args.reopen_after_failures,
            args.reopen_delay_s,
        )
    if not any(c.ok for c in CAMS.values()):
        raise SystemExit("[relay] no cameras opened")
    print(f"[relay] serving {sorted(k for k, c in CAMS.items() if c.ok)} on :{args.port}", flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
