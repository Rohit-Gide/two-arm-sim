"""HTTP server: the API under /api, the 3D viewer at /, the robot's meshes under /robot.

    python -m twoarm.server            # http://127.0.0.1:8700
    python -m twoarm.server --port 9000 --host 0.0.0.0
"""
from __future__ import annotations

import argparse
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qsl, urlparse

from .api import Api
from .reality import Reality
from .sim import Sim

ROOT = Path(__file__).resolve().parents[1]
STATIC = {"/robot/": ROOT / "robot", "/": ROOT / "viewer"}


def make_handler(api: Api):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _json(self, status, obj):
            data = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/api" or u.path.startswith("/api/"):
                return self._json(*api.handle("GET", u.path, None, dict(parse_qsl(u.query))))
            for prefix, folder in STATIC.items():
                if u.path.startswith(prefix):
                    f = (folder / (u.path[len(prefix):] or "index.html")).resolve()
                    if folder.resolve() in f.parents and f.is_file():
                        data = f.read_bytes()
                        self.send_response(200)
                        self.send_header("Content-Type", mimetypes.guess_type(f.name)[0] or "application/octet-stream")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
            self._json(404, {"ok": False, "error": "not_found", "message": self.path})

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if not isinstance(body, dict):
                    raise ValueError("the body must be a JSON object")
            except ValueError as e:
                return self._json(400, {"ok": False, "error": "bad_json", "message": str(e)})
            self._json(*api.handle("POST", u.path, body, dict(parse_qsl(u.query))))
    return Handler


def serve(host="127.0.0.1", port=8700, realtime=True, reality=None):
    sim = Sim(realtime=realtime, reality=reality)
    api = Api(sim)
    httpd = ThreadingHTTPServer((host, port), make_handler(api))
    httpd.daemon_threads = True
    url = f"http://127.0.0.1:{httpd.server_address[1]}"
    sim.start()
    return httpd, sim, api, url


def main():
    ap = argparse.ArgumentParser(description="Two FANUC M-20iA arms in one cell: simulator, API and viewer.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8700)
    ap.add_argument("--fast", action="store_true", help="run as fast as the machine allows instead of in real time")
    ap.add_argument("--seed", type=int, default=1, help="which draw of the real-world errors this cell has (see twoarm/reality.py)")
    ap.add_argument("--ideal", action="store_true", help="no real-world errors: the controllers' model is the world")
    a = ap.parse_args()
    reality = Reality.ideal() if a.ideal else Reality.sample(a.seed)
    httpd, sim, _, url = serve(a.host, a.port, realtime=not a.fast, reality=reality)
    print(f"two-arm-sim   viewer {url}/   API {url}/api   (Ctrl-C to stop)   " +
          ("IDEAL world" if a.ideal else f"real-world errors, seed {a.seed} (GET /api/reality)"), flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop_thread()


if __name__ == "__main__":
    main()
