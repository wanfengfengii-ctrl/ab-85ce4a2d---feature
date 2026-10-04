"""HTTP 入口：仅依赖 Python 标准库。

环境变量:
  API_PORT   服务监听端口（默认 8000）
  API_HOST   监听地址（默认 0.0.0.0）
"""

from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .solver import ValidationError, trace


class Handler(BaseHTTPRequestHandler):
    server_version = "HorizonTrace/1.0"

    def _send_json(self, status_code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.rstrip("/") in ("", "/health", "/healthz"):
            self._send_json(200, {"status": "ok"})
        else:
            self._send_json(404, {"error": "not_found", "message": "路径不存在"})

    def do_POST(self):
        if self.path != "/api/horizons/trace":
            self._send_json(404, {"error": "not_found", "message": "路径不存在"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b""
            payload = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self._send_json(
                400, {"status": "error", "error": "invalid_json", "message": "请求体不是合法 JSON"}
            )
            return

        try:
            result = trace(payload)
        except ValidationError as exc:
            self._send_json(
                422, {"status": "error", "error": "validation_error", "message": str(exc)}
            )
            return
        # 求解器对"数据合法但无可行组合"返回 200 + feasible=false，
        # 这是明确的业务结果而非请求错误。
        self._send_json(200, result)

    def log_message(self, fmt, *args):  # 安静日志
        return


def main():
    host = os.environ.get("API_HOST", "0.0.0.0")
    port = int(os.environ.get("API_PORT", "8000"))
    httpd = ThreadingHTTPServer((host, port), Handler)
    print(f"horizon-trace listening on {host}:{port}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
