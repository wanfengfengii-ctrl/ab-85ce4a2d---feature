"""针对运行中服务的联合拾取 API 冒烟测试（仅用标准库）。

环境变量:
  BASE_URL  服务地址（默认 http://app:8000）

全部断言通过时退出码 0，否则非零。
"""

import json
import os
import sys
import urllib.error
import urllib.request

BASE_URL = os.environ.get("BASE_URL", "http://app:8000")
TIMEOUT = 5


def request(method, path, body=None):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(
        BASE_URL + path, data=data, headers=headers, method=method
    )
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def col(cands):
    return {"candidates": [
        {"id": cid, "depth": d, "confidence": c} for cid, d, c in cands
    ]}


LIMITS = {
    "min_thickness": 6,
    "max_thickness": 10,
    "max_slope": 1,
    "max_thickness_change": 1,
    "max_second_diff": 1,
}

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))
    mark = "PASS" if cond else "FAIL"
    print(f"[{mark}] {name}" + (f" -- {detail}" if detail and not cond else ""))


def main():
    # 1. 健康检查。
    status, payload = request("GET", "/health")
    check("GET /health 返回 200", status == 200, str(payload))
    check("健康状态为 ok", payload.get("status") == "ok")

    # 2. 可行联合拾取：高置信干扰点 vs 平滑双界面。
    columns = [
        col([("U", 10 + i, 5), ("L", 18 + i, 5), ("X", 0, 9), ("Y", 30, 9)])
        for i in range(8)
    ]
    status, payload = request("POST", "/api/horizons/trace",
                              {"columns": columns, "limits": LIMITS})
    check("可行用例返回 200", status == 200, str(payload))
    check("feasible=true", payload.get("feasible") is True)
    check("status=ok", payload.get("status") == "ok")

    upper = payload.get("upper_horizon", [])
    lower = payload.get("lower_horizon", [])
    check("两条界面各 8 点", len(upper) == 8 and len(lower) == 8)
    check("联合追踪拒绝逐列最强干扰点",
          [p["id"] for p in upper] == ["U"] * 8
          and [p["id"] for p in lower] == ["L"] * 8,
          f"upper={[p['id'] for p in upper]}")
    check("上界面逐点严格位于下界面之上",
          all(upper[i]["depth"] < lower[i]["depth"] for i in range(8)))
    check("逐列厚度均为 8 且在限值内",
          [t["thickness"] for t in payload.get("thicknesses", [])] == [8] * 8)
    check("相邻坡差全部给出且受限",
          len(payload.get("slopes", {}).get("upper", [])) == 7
          and all(abs(s["slope"]) <= LIMITS["max_slope"]
                  for s in payload["slopes"]["upper"]
                  + payload["slopes"]["lower"]))
    check("二阶差全部给出且最大为 0",
          len(payload.get("second_diffs", {}).get("upper", [])) == 6
          and payload["verdict"]["max_second_diff"] == 0)
    verdict = payload.get("verdict", {})
    check("裁决值完整（置信度80/二阶差0/行程14）",
          verdict == {"total_confidence": 80,
                      "max_second_diff": 0, "total_travel": 14},
          str(verdict))

    # 3. 第二组联合拾取：倾斜剖面，厚度恒定。
    columns2 = [
        col([("u", 2 * i, 4), ("l", 2 * i + 7, 6), ("z", 50, 9)])
        for i in range(8)
    ]
    limits2 = {
        "min_thickness": 5, "max_thickness": 9,
        "max_slope": 2, "max_thickness_change": 0, "max_second_diff": 1,
    }
    status, payload2 = request("POST", "/api/horizons/trace",
                               {"columns": columns2, "limits": limits2})
    check("倾斜剖面返回 200 且可行",
          status == 200 and payload2.get("feasible") is True, str(payload2))
    check("倾斜剖面厚度恒定为 7",
          [t["thickness"] for t in payload2.get("thicknesses", [])] == [7] * 8)

    # 4. 无可行组合：厚度被钉死为 9（实际只有 8）。
    limits_no = dict(LIMITS, min_thickness=9, max_thickness=9)
    status, payload = request("POST", "/api/horizons/trace",
                              {"columns": columns, "limits": limits_no})
    check("无解用例返回 200", status == 200)
    check("明确 feasible=false", payload.get("feasible") is False)
    check("无解状态 no_solution", payload.get("status") == "no_solution")
    check("不伪造任何局部轨迹",
          "upper_horizon" not in payload and "lower_horizon" not in payload,
          str(payload))

    # 5. 入参校验：列数不足。
    status, payload = request("POST", "/api/horizons/trace",
                              {"columns": columns[:5], "limits": LIMITS})
    check("列数不足返回 422", status == 422, str(payload))

    # 6. 非法 JSON。
    req = urllib.request.Request(
        BASE_URL + "/api/horizons/trace",
        data=b"{not-json", headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=TIMEOUT)
        check("非法 JSON 返回 400", False)
    except urllib.error.HTTPError as exc:
        check("非法 JSON 返回 400", exc.code == 400)

    # 7. 未知路径。
    status, _ = request("GET", "/nope")
    check("未知路径返回 404", status == 404)

    # 8. 尖灭（pinchout）联合拾取：透镜体剖面，上下界面在第 3~4 列闭合。
    pinch_limits = {
        "min_thickness": 6, "max_thickness": 10,
        "max_slope": 3, "max_thickness_change": 1, "max_second_diff": 6,
    }
    pinch_columns = []
    for i in range(8):
        if i in (3, 4):
            cands = [("M", 15 + 3 * (i - 3), 9)]
        else:
            cands = [("U", 10 + i, 5), ("L", 16 + i, 5)]
        cands += [("X", 0, 9), ("Y", 100, 9)]
        pinch_columns.append(col(cands))
    pinch_cfg = {"max_columns": 3, "max_transition_change": 6}

    status, p = request("POST", "/api/horizons/trace",
                        {"columns": pinch_columns, "limits": pinch_limits,
                         "pinchout": pinch_cfg})
    check("尖灭用例返回 200 且可行",
          status == 200 and p.get("feasible") is True, str(p))
    check("尖灭段零基起止列与长度正确",
          p.get("pinchout") == {"start_column": 3, "end_column": 4, "length": 2},
          str(p.get("pinchout")))
    thick = [t["thickness"] for t in p.get("thicknesses", [])]
    check("尖灭段内厚度为零、段外为正",
          thick == [6, 6, 6, 0, 0, 6, 6, 6], str(thick))
    check("尖灭列上下界面拾取同一候选",
          all(p["upper_horizon"][i]["id"] == "M"
              and p["lower_horizon"][i]["id"] == "M" for i in (3, 4)))
    check("尖灭裁决值完整（置信度78/二阶差6/行程26）",
          p.get("verdict") == {"total_confidence": 78,
                               "max_second_diff": 6, "total_travel": 26},
          str(p.get("verdict")))

    # 9. 同一透镜体剖面省略 pinchout：强制正厚度下无解（既有行为不变）。
    status, p = request("POST", "/api/horizons/trace",
                        {"columns": pinch_columns, "limits": pinch_limits})
    check("省略 pinchout 时透镜体剖面无解",
          status == 200 and p.get("feasible") is False
          and p.get("status") == "no_solution", str(p))
    check("兼容请求响应不含 pinchout 字段", "pinchout" not in p)

    # 10. 过渡限值过小：闭合段无法接入 -> no_solution 且不输出局部界面。
    status, p = request("POST", "/api/horizons/trace",
                        {"columns": pinch_columns, "limits": pinch_limits,
                         "pinchout": {"max_columns": 3,
                                      "max_transition_change": 5}})
    check("过渡限值过小时 no_solution",
          status == 200 and p.get("status") == "no_solution", str(p))
    check("不伪造尖灭前后局部界面",
          "upper_horizon" not in p and "lower_horizon" not in p
          and "pinchout" not in p, str(p))

    # 11. pinchout 配置非法：按字段 422。
    for bad in ({"max_columns": 4, "max_transition_change": 6},
                {"max_columns": 0, "max_transition_change": 6},
                {"max_columns": 2},
                {"max_columns": 2, "max_transition_change": -1},
                {"max_columns": 2.5, "max_transition_change": 6}):
        status, p = request("POST", "/api/horizons/trace",
                            {"columns": pinch_columns, "limits": pinch_limits,
                             "pinchout": bad})
        check(f"非法 pinchout 返回 422: {bad}",
              status == 422 and p.get("error") == "validation_error", str(p))

    failed = [name for name, ok, _ in checks if not ok]
    print(f"\n冒烟结果: {len(checks) - len(failed)}/{len(checks)} 通过")
    if failed:
        print("失败项:", ", ".join(failed))
        return 1
    print("全部冒烟通过")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # 网络层异常同样判定失败
        print(f"[FAIL] 冒烟脚本异常: {exc!r}")
        sys.exit(2)
