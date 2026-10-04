"""容器健康检查：探测本实例 /health，成功退出 0。"""

import os
import sys
import urllib.request

port = os.environ.get("API_PORT", "8000")
try:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=3) as r:
        sys.exit(0 if r.status == 200 else 1)
except Exception:
    sys.exit(1)
