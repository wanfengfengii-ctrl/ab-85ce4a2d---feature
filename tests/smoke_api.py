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

    # 5. 透镜状尖灭：中部两列双界面汇于同一候选 P，未启用尖灭时无解，
    #    启用后必须给出唯一尖灭段（零基 4~5、长度 2）并完整重开。
    lens = [
        col([("U", 10, 5), ("L", 18, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("U", 11, 5), ("L", 17, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("U", 12, 5), ("L", 16, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("U", 13, 5), ("L", 15, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("P", 14, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("P", 14, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("U", 13, 5), ("L", 15, 5), ("X", 0, 9), ("Y", 30, 9)]),
        col([("U", 12, 5), ("L", 16, 5), ("X", 0, 9), ("Y", 30, 9)]),
    ]
    lens_limits = {
        "min_thickness": 2, "max_thickness": 10,
        "max_slope": 2, "max_thickness_change": 2, "max_second_diff": 1,
    }
    status, payload_p = request("POST", "/api/horizons/trace",
                                {"columns": lens, "limits": lens_limits})
    check("透镜剖面未启用尖灭时无解",
          status == 200 and payload_p.get("status") == "no_solution",
          str(payload_p))

    pinch = {"max_columns": 3, "max_transition_change": 2}
    status, payload_p = request(
        "POST", "/api/horizons/trace",
        {"columns": lens, "limits": lens_limits, "pinchout": pinch})
    check("尖灭用例返回 200 且可行",
          status == 200 and payload_p.get("feasible") is True, str(payload_p))
    check("尖灭段证据为零基 4~5、长度 2",
          payload_p.get("pinchout") ==
          {"start_column": 4, "end_column": 5, "length": 2},
          str(payload_p.get("pinchout")))
    th = [t["thickness"] for t in payload_p.get("thicknesses", [])]
    check("厚度序列 8,6,4,2,0,0,2,4", th == [8, 6, 4, 2, 0, 0, 2, 4], str(th))
    up = payload_p.get("upper_horizon", [])
    lo = payload_p.get("lower_horizon", [])
    check("尖灭段内上下界面选择同一候选且同深度",
          all(up[i]["id"] == "P" and lo[i]["id"] == "P"
              and up[i]["depth"] == lo[i]["depth"] for i in (4, 5)))
    check("段外上下界面严格分离",
          all(up[i]["depth"] < lo[i]["depth"] for i in (0, 1, 2, 3, 6, 7)))
    check("尖灭响应仍给出坡差与二阶差证据",
          len(payload_p.get("slopes", {}).get("upper", [])) == 7
          and len(payload_p.get("second_diffs", {}).get("upper", [])) == 6)
    check("尖灭段不含首末列",
          payload_p["pinchout"]["start_column"] >= 1
          and payload_p["pinchout"]["end_column"] <= 6)

    # 5b. 边界厚度变化新限值过紧 -> no_solution，不返回局部界面。
    bad_pinch = dict(pinch, max_transition_change=1)
    status, payload_p = request(
        "POST", "/api/horizons/trace",
        {"columns": lens, "limits": lens_limits, "pinchout": bad_pinch})
    check("无法闭合时 no_solution 且无局部界面",
          status == 200 and payload_p.get("status") == "no_solution"
          and "upper_horizon" not in payload_p
          and "lower_horizon" not in payload_p,
          str(payload_p))

    # 5c. 无法形成完整闭合再开轨迹（末列只有汇聚候选）。
    lens_dead = [dict(c) for c in lens]
    lens_dead[7] = col([("P", 14, 5), ("X", 0, 9), ("Y", 30, 9)])
    status, payload_p = request(
        "POST", "/api/horizons/trace",
        {"columns": lens_dead, "limits": lens_limits, "pinchout": pinch})
    check("末列无法重开时 no_solution",
          status == 200 and payload_p.get("status") == "no_solution"
          and "upper_horizon" not in payload_p, str(payload_p))

    # 5d. pinchout 非法配置按字段返回 422。
    for bad_pinch in (
        {},
        {"max_columns": 3},
        {"max_transition_change": 2},
        {"max_columns": 0, "max_transition_change": 2},
        {"max_columns": 4, "max_transition_change": 2},
        {"max_columns": "2", "max_transition_change": 2},
        {"max_columns": 2, "max_transition_change": -1},
    ):
        status, payload_p = request(
            "POST", "/api/horizons/trace",
            {"columns": lens, "limits": lens_limits, "pinchout": bad_pinch})
        check(f"非法 pinchout {bad_pinch} 返回 422",
              status == 422, f"status={status} {payload_p}")

    # 6. 入参校验：列数不足。
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
