"""A tiny HTTP client for the simulator (standard library only).

    from twoarm.client import Client
    c = Client("http://127.0.0.1:8700")
    c.post("/api/arm/a/move_line", pos=[-0.3, 0.2, 1.0], tool_down=0, speed=0.5, wait=True)
    print(c.get("/api/state")["arms"]["a"]["tcp"]["world"]["pos"])
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request


class ApiError(Exception):
    def __init__(self, status, body):
        super().__init__(f"{status} {body.get('error')}: {body.get('message')}")
        self.status, self.body = status, body


class Client:
    def __init__(self, url="http://127.0.0.1:8700", timeout=300):
        self.url, self.timeout = url.rstrip("/"), timeout

    def _do(self, req):
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            raise ApiError(e.code, json.loads(e.read() or b"{}")) from None

    def get(self, path):
        return self._do(urllib.request.Request(self.url + path))

    def post(self, path, **body):
        return self._do(urllib.request.Request(self.url + path, data=json.dumps(body).encode(),
                                               headers={"Content-Type": "application/json"}, method="POST"))
