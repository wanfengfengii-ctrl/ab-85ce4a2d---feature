"""求解器单元测试：构造场景 + 随机实例暴力枚举对照。"""

import os
import random
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.solver import (  # noqa: E402
    MAX_CANDIDATES,
    MAX_COLS,
    MIN_CANDIDATES,
    MIN_COLS,
    ValidationError,
    _build_states,
    parse_request,
    solve,
    trace,
)

DEFAULT_LIMITS = {
    "min_thickness": 6,
    "max_thickness": 10,
    "max_slope": 1,
    "max_thickness_change": 1,
    "max_second_diff": 1,
}


def col(cands):
    """cands: [(id, depth, confidence), ...]"""
    return {"candidates": [
        {"id": cid, "depth": d, "confidence": c} for cid, d, c in cands
    ]}


def payload(columns, limits=None):
    return {"columns": columns, "limits": limits or DEFAULT_LIMITS}


def smooth_columns(n=8):
    """平滑地层 + 每列高置信干扰点的典型剖面。

    平滑轨迹：上界面 10+i，下界面 18+i，置信度 5；
    干扰点深度 0 / 30，置信度 9——逐列选最强会得到交叉突跳。
    """
    return [
        col([("U", 10 + i, 5), ("L", 18 + i, 5), ("X", 0, 9), ("Y", 30, 9)])
        for i in range(n)
    ]


def chosen_pairs(columns, result):
    """从结果反解每列 (上候选下标, 下候选下标)（columns 已按深度排序）。"""
    pairs = []
    for i, c in enumerate(columns):
        index = {cand["id"]: k for k, cand in enumerate(c)}
        pairs.append((
            index[result["upper_horizon"][i]["id"]],
            index[result["lower_horizon"][i]["id"]],
        ))
    return pairs


def validate_result(columns, limits, result, pinchout=None):
    """对成功结果做完整约束复核，返回裁决三元组。

    pinchout 非 None 时按尖灭规则复核：恰好一个不含首末列、长度
    1~max_columns 的连续零厚度段；段外严格分离且厚度在限值内；
    跨越尖灭边界的厚度变化改用 max_transition_change。
    """
    n_cols = len(columns)
    assert result["feasible"] is True
    upper, lower = result["upper_horizon"], result["lower_horizon"]
    assert len(upper) == len(lower) == n_cols

    du = [p["depth"] for p in upper]
    dl = [p["depth"] for p in lower]
    pairs = chosen_pairs(columns, result)
    total_conf = 0

    if pinchout is None:
        assert "pinchout" not in result
        p_start, p_end = None, None
    else:
        info = result["pinchout"]
        p_start, p_end = info["start_column"], info["end_column"]
        assert info["length"] == p_end - p_start + 1
        assert 1 <= info["length"] <= pinchout["max_columns"]
        assert 1 <= p_start and p_end <= n_cols - 2  # 不含首末列

    # 逐列分离规则与厚度限值。
    for i in range(n_cols):
        assert upper[i]["column"] == lower[i]["column"] == i
        ui, li = pairs[i]
        total_conf += columns[i][ui]["confidence"]
        if li != ui:
            total_conf += columns[i][li]["confidence"]
        t = dl[i] - du[i]
        assert t == result["thicknesses"][i]["thickness"]
        if pinchout is not None and p_start <= i <= p_end:
            assert ui == li and t == 0  # 尖灭段内拾取同一候选
        else:
            assert ui != li
            assert limits["min_thickness"] <= t <= limits["max_thickness"]

    if pinchout is not None:
        # 恰好一个连续尖灭段：段外厚度全部为正。
        assert all(
            (dl[i] - du[i]) == 0 for i in range(p_start, p_end + 1)
        )
        assert all(
            (dl[i] - du[i]) > 0
            for i in list(range(0, p_start)) + list(range(p_end + 1, n_cols))
        )

    # 相邻坡差与厚度变化。
    for i in range(1, n_cols):
        su, sl = du[i] - du[i - 1], dl[i] - dl[i - 1]
        assert result["slopes"]["upper"][i - 1]["slope"] == su
        assert result["slopes"]["lower"][i - 1]["slope"] == sl
        assert abs(su) <= limits["max_slope"]
        assert abs(sl) <= limits["max_slope"]
        t1 = dl[i] - du[i]
        t0 = dl[i - 1] - du[i - 1]
        if pinchout is not None and (t0 == 0) != (t1 == 0):
            limit = pinchout["max_transition_change"]
        else:
            limit = limits["max_thickness_change"]
        assert abs(t1 - t0) <= limit

    # 连续三列二阶差与裁决值。
    max_sec = 0
    for i in range(2, n_cols):
        su = du[i] - 2 * du[i - 1] + du[i - 2]
        sl = dl[i] - 2 * dl[i - 1] + dl[i - 2]
        assert result["second_diffs"]["upper"][i - 2]["second_diff"] == su
        assert result["second_diffs"]["lower"][i - 2]["second_diff"] == sl
        cap = limits.get("max_second_diff")
        if cap is not None:
            assert abs(su) <= cap
            assert abs(sl) <= cap
        max_sec = max(max_sec, abs(su), abs(sl))

    travel = sum(
        abs(du[i] - du[i - 1]) + abs(dl[i] - dl[i - 1])
        for i in range(1, n_cols)
    )
    verdict = result["verdict"]
    assert verdict["total_confidence"] == total_conf
    assert verdict["max_second_diff"] == max_sec
    assert verdict["total_travel"] == travel
    return total_conf, max_sec, travel


class ScenarioTests(unittest.TestCase):
    def test_smooth_joint_trace_beats_greedy_strongest(self):
        # 逐列最强回波是深度 0/30 的干扰点（成对置信度 18），
        # 但任何跨列延续都违反坡差；联合追踪必须给出平滑双界面。
        columns, limits, _ = parse_request(payload(smooth_columns()))
        result = trace(payload(smooth_columns()))
        self.assertEqual(result["status"], "ok")
        validate_result(columns, limits, result)
        self.assertEqual([p["id"] for p in result["upper_horizon"]], ["U"] * 8)
        self.assertEqual([p["id"] for p in result["lower_horizon"]], ["L"] * 8)
        self.assertEqual([p["depth"] for p in result["upper_horizon"]],
                         [10 + i for i in range(8)])
        self.assertEqual([p["depth"] for p in result["lower_horizon"]],
                         [18 + i for i in range(8)])
        self.assertEqual(result["verdict"]["max_second_diff"], 0)
        self.assertEqual(result["verdict"]["total_travel"], 14)
        self.assertEqual(result["verdict"]["total_confidence"], 80)

    def test_second_diff_renders_no_solution(self):
        # 深度序列 0,0,3,3,6,6,...：坡差/厚度变化全部放行、二阶差
        # 上限为 2 时无可行组合；放宽到 3 后有解且裁决值恰为 3。
        zigzag = [
            col([("U", 3 * (i // 2), 5), ("L", 3 * (i // 2) + 10, 5),
                 ("Z", 100, 9)])
            for i in range(8)
        ]
        limits = {
            "min_thickness": 8, "max_thickness": 12,
            "max_slope": 3, "max_thickness_change": 1, "max_second_diff": 2,
        }
        result = trace(payload(zigzag, limits))
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")
        self.assertNotIn("upper_horizon", result)  # 不伪造局部轨迹

        limits["max_second_diff"] = 3
        columns, plimits, _ = parse_request(payload(zigzag, limits))
        ok = trace(payload(zigzag, limits))
        self.assertTrue(ok["feasible"])
        validate_result(columns, plimits, ok)
        self.assertEqual(ok["verdict"]["max_second_diff"], 3)

    def test_thickness_range_renders_no_solution(self):
        limits = dict(DEFAULT_LIMITS, min_thickness=9, max_thickness=9)
        # 平滑对厚度恰为 8，干扰点无法成合法对 -> 无解。
        result = trace(payload(smooth_columns(), limits))
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")

    def test_optional_second_diff_still_adjudicated(self):
        # 省略 max_second_diff：无二阶差硬约束，但裁决仍在最大置信度
        # 前提下最小化最大二阶差。
        columns, _limits, _p = parse_request(payload(smooth_columns()))
        limits = {k: DEFAULT_LIMITS[k] for k in
                  ("min_thickness", "max_thickness",
                   "max_slope", "max_thickness_change")}
        result = solve(columns, limits)
        validate_result(columns, limits, result)
        self.assertEqual(result["verdict"]["max_second_diff"], 0)
        self.assertEqual(result["verdict"]["total_confidence"], 80)

    def test_stable_tie_break_by_candidate_id(self):
        # 每个上/下位置各有两个等深、等置信、不同编号候选，
        # 目标值完全相同，逐列编号字典序最小者稳定胜出。
        columns, limits, _ = parse_request(payload([
            col([("b", 5, 3), ("a", 5, 3), ("d", 12, 3), ("c", 12, 3)])
            for _ in range(8)
        ]))
        result = solve(columns, limits)
        validate_result(columns, limits, result)
        self.assertEqual([p["id"] for p in result["upper_horizon"]], ["a"] * 8)
        self.assertEqual([p["id"] for p in result["lower_horizon"]], ["c"] * 8)

    def test_confidence_priority_over_curvature(self):
        # 高置信折线 H/LH（二阶差 2）与低置信直线 S/LS（二阶差 0）
        # 都可行：第一目标置信度优先，必须选折线。
        upper_zig = [10, 10, 12, 12, 14, 14, 16, 16]
        zigzag = []
        for i in range(8):
            uz = upper_zig[i]
            zigzag.append(col([
                ("H", uz, 9),    # 高置信折线上界面
                ("S", 9, 1),     # 低置信直线上界面
                ("LH", uz + 8, 5),   # 跟随 H 的下界面
                ("LS", 17, 5),       # 跟随 S 的下界面
            ]))
        limits = {
            "min_thickness": 6, "max_thickness": 10,
            "max_slope": 2, "max_thickness_change": 2, "max_second_diff": 2,
        }
        columns, plimits, _ = parse_request(payload(zigzag, limits))
        result = solve(columns, plimits)
        validate_result(columns, plimits, result)
        self.assertEqual([p["id"] for p in result["upper_horizon"]], ["H"] * 8)
        self.assertEqual(result["verdict"]["max_second_diff"], 2)
        self.assertEqual(result["verdict"]["total_confidence"], 14 * 8)

    def test_column_count_bounds(self):
        with self.assertRaises(ValidationError):
            parse_request(payload(smooth_columns(MIN_COLS - 1)))
        with self.assertRaises(ValidationError):
            parse_request(payload(smooth_columns(MAX_COLS + 1)))

    def test_candidate_count_bounds(self):
        too_few = smooth_columns()
        too_few[0] = col([("a", 0, 1), ("b", 5, 1)])
        with self.assertRaises(ValidationError):
            parse_request(payload(too_few))

        too_many = smooth_columns()
        too_many[0] = col([(c, i, 1) for i, c in enumerate("abcdefghi")])
        with self.assertRaises(ValidationError):
            parse_request(payload(too_many))

        just_right = smooth_columns()
        just_right[0] = col([("a", 0, 1), ("b", 5, 1), ("c", 9, 1)])
        parse_request(payload(just_right))  # 3 个合法

    def test_validation_errors(self):
        base = smooth_columns()
        with self.assertRaises(ValidationError):
            parse_request([])
        with self.assertRaises(ValidationError):
            parse_request({"columns": base})  # 缺 limits
        with self.assertRaises(ValidationError):
            parse_request(payload(base, {**DEFAULT_LIMITS, "min_thickness": 0}))
        with self.assertRaises(ValidationError):
            parse_request(payload(base, {**DEFAULT_LIMITS, "max_slope": -1}))
        with self.assertRaises(ValidationError):
            parse_request(payload(base, {
                **DEFAULT_LIMITS, "min_thickness": 9, "max_thickness": 8}))

        dup = payload(smooth_columns())
        dup["columns"][0]["candidates"][2]["id"] = "U"
        with self.assertRaises(ValidationError):
            parse_request(dup)

        zero_conf = payload(smooth_columns())
        zero_conf["columns"][1]["candidates"][0]["confidence"] = 0
        with self.assertRaises(ValidationError):
            parse_request(zero_conf)

        bool_depth = payload(smooth_columns())
        bool_depth["columns"][1]["candidates"][0]["depth"] = True
        with self.assertRaises(ValidationError):
            parse_request(bool_depth)

        float_depth = payload(smooth_columns())
        float_depth["columns"][1]["candidates"][0]["depth"] = 1.5
        with self.assertRaises(ValidationError):
            parse_request(float_depth)


def brute_force(columns, limits):
    """穷举所有联合状态序列，返回与求解器同口径的最优键。

    key = (-总置信度, 最大二阶差, 总行程, 状态序号元组)；None 表示无解。
    """
    col_states = [_build_states(c, limits) for c in columns]
    if any(not s for s in col_states):
        return None
    n = len(columns)
    best = None

    def rec(ci, ranks, pdu, pdl, ppdu, ppdl, neg_conf, max_sec, travel):
        nonlocal best
        if ci == n:
            key = (neg_conf, max_sec, travel, tuple(ranks))
            if best is None or key < best:
                best = key
            return
        for rank, (ui, li) in enumerate(col_states[ci]):
            up, lo = columns[ci][ui], columns[ci][li]
            thick = lo["depth"] - up["depth"]
            if pdu is not None:
                us = abs(up["depth"] - pdu)
                ls = abs(lo["depth"] - pdl)
                if us > limits["max_slope"] or ls > limits["max_slope"]:
                    continue
                if abs(thick - (pdl - pdu)) > limits["max_thickness_change"]:
                    continue
                nmax, ntrav = max_sec, travel + us + ls
                if ppdu is not None:
                    cap2 = limits.get("max_second_diff")
                    cap2 = float("inf") if cap2 is None else cap2
                    su = abs(up["depth"] - 2 * pdu + ppdu)
                    sl = abs(lo["depth"] - 2 * pdl + ppdl)
                    if su > cap2 or sl > cap2:
                        continue
                    nmax = max(nmax, su, sl)
            else:
                nmax, ntrav = 0, 0
            rec(ci + 1, ranks + [rank], up["depth"], lo["depth"],
                pdu, pdl,
                neg_conf - up["confidence"] - lo["confidence"], nmax, ntrav)

    rec(0, [], None, None, None, None, 0, 0, 0)
    return best


def result_rank_path(columns, limits, result):
    """结果 -> 每列在 _build_states 中的状态序号（与决胜序号口径一致）。"""
    ranks = []
    for i, c in enumerate(columns):
        index = {cand["id"]: k for k, cand in enumerate(c)}
        pair = (
            index[result["upper_horizon"][i]["id"]],
            index[result["lower_horizon"][i]["id"]],
        )
        ranks.append(_build_states(c, limits).index(pair))
    return tuple(ranks)


def random_body(rng):
    n = rng.randint(MIN_COLS, 10)
    ids = ["a", "b", "c"]
    columns = []
    for _ in range(n):
        depths = rng.sample(range(0, 9), k=3)  # 同列深度互异
        columns.append(col([
            (ids[j], depths[j], rng.randint(1, 5)) for j in range(3)
        ]))
    limits = {
        "min_thickness": 1,
        "max_thickness": 9,
        "max_slope": rng.randint(3, 7),
        "max_thickness_change": rng.randint(3, 7),
        "max_second_diff": rng.randint(2, 8),
    }
    # 约一半实例省略二阶差硬上限（裁决仍最小化最大二阶差）。
    if rng.random() < 0.5:
        del limits["max_second_diff"]
    return payload(columns, limits)


class FuzzTests(unittest.TestCase):
    def test_matches_brute_force(self):
        rng = random.Random(20261001)
        feasible_hits = 0
        for _ in range(300):
            body = random_body(rng)
            columns, limits, pinchout = parse_request(body)
            self.assertIsNone(pinchout)  # 旧生成器不携带 pinchout
            expected = brute_force(columns, limits)
            actual = solve(columns, limits)
            if expected is None:
                self.assertFalse(actual["feasible"])
                continue
            self.assertTrue(actual["feasible"])
            validate_result(columns, limits, actual)
            key = (
                -actual["verdict"]["total_confidence"],
                actual["verdict"]["max_second_diff"],
                actual["verdict"]["total_travel"],
                result_rank_path(columns, limits, actual),
            )
            self.assertEqual(key, expected)
            feasible_hits += 1
        self.assertGreater(feasible_hits, 50)


class PerformanceTests(unittest.TestCase):
    def test_max_size_runs_fast(self):
        rng = random.Random(42)
        ids = [f"p{j}" for j in range(MAX_CANDIDATES)]
        columns, limits, _ = parse_request(payload([
            col([(ids[j], rng.randint(0, 40), rng.randint(1, 9))
                 for j in range(MAX_CANDIDATES)])
            for _ in range(MAX_COLS)
        ], {
            "min_thickness": 2, "max_thickness": 30,
            "max_slope": 12, "max_thickness_change": 10, "max_second_diff": 20,
        }))
        t0 = time.time()
        result = solve(columns, limits)
        elapsed = time.time() - t0
        self.assertTrue(result["feasible"])
        self.assertLess(elapsed, 5.0)

    def test_max_size_pinchout_runs_fast(self):
        rng = random.Random(42)
        ids = [f"p{j}" for j in range(MAX_CANDIDATES)]
        body = payload([
            col([(ids[j], rng.randint(0, 40), rng.randint(1, 9))
                 for j in range(MAX_CANDIDATES)])
            for _ in range(MAX_COLS)
        ], {
            "min_thickness": 2, "max_thickness": 30,
            "max_slope": 12, "max_thickness_change": 10, "max_second_diff": 20,
        })
        body["pinchout"] = {"max_columns": 3, "max_transition_change": 15}
        columns, limits, pinchout = parse_request(body)
        t0 = time.time()
        result = solve(columns, limits, pinchout)
        elapsed = time.time() - t0
        self.assertTrue(result["feasible"])
        validate_result(columns, limits, result, pinchout)
        self.assertLess(elapsed, 10.0)


def pinchout_columns(n=8, start=3, merges=(15, 18)):
    """透镜体尖灭剖面：段外厚度 6，段内上下界面闭合于同一候选。

    上界面深度 10+i（坡差 1）向闭合点俯冲；闭合点深度由几何唯一确定
    （须同时落在相邻上/下界面 max_slope=3 范围内）；另有高置信干扰点
    X/Y——逐列选最强会得到交叉突跳。该剖面在不启用 pinchout 时无解
    （闭合列凑不出严格分离对），正是"强制正厚度误判连续反射层"的场景。
    """
    columns = []
    for i in range(n):
        if start <= i < start + len(merges):
            cands = [("M", merges[i - start], 9)]
        else:
            cands = [("U", 10 + i, 5), ("L", 16 + i, 5)]
        cands.append(("X", 0, 9))
        cands.append(("Y", 100, 9))
        columns.append(col(cands))
    return columns


PINCH_LIMITS = {
    "min_thickness": 6,
    "max_thickness": 10,
    "max_slope": 3,
    "max_thickness_change": 1,
    "max_second_diff": 6,
}
PINCHOUT = {"max_columns": 3, "max_transition_change": 6}


class PinchoutScenarioTests(unittest.TestCase):
    def test_pinchout_segment_recovered(self):
        body = payload(pinchout_columns(), PINCH_LIMITS)
        body["pinchout"] = dict(PINCHOUT)
        columns, limits, pinch = parse_request(body)
        result = trace(body)
        self.assertEqual(result["status"], "ok")
        validate_result(columns, limits, result, pinch)

        info = result["pinchout"]
        self.assertEqual(info, {"start_column": 3, "end_column": 4, "length": 2})
        for i in (3, 4):
            self.assertEqual(result["upper_horizon"][i]["id"], "M")
            self.assertEqual(result["lower_horizon"][i]["id"], "M")
            self.assertEqual(result["thicknesses"][i]["thickness"], 0)
        for i in (0, 1, 2, 5, 6, 7):
            self.assertEqual(result["upper_horizon"][i]["id"], "U")
            self.assertEqual(result["lower_horizon"][i]["id"], "L")
        # 尖灭列置信度只计一次：6 列分离对 * 10 + 2 列闭合 * 9。
        self.assertEqual(result["verdict"]["total_confidence"], 78)
        self.assertEqual(result["verdict"]["max_second_diff"], 6)
        self.assertEqual(result["verdict"]["total_travel"], 26)

    def test_same_profile_no_solution_without_pinchout(self):
        # 同一剖面省略 pinchout：闭合列无严格分离对 -> 沿用 no_solution。
        result = trace(payload(pinchout_columns(), PINCH_LIMITS))
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")
        self.assertNotIn("upper_horizon", result)
        self.assertNotIn("pinchout", result)

    def test_transition_change_limit_enforced(self):
        # 过渡限值 5 < 边界厚度落差 6：闭合段无法接入 -> no_solution，
        # 且不得输出尖灭前后的局部界面。
        body = payload(pinchout_columns(), PINCH_LIMITS)
        body["pinchout"] = {"max_columns": 3, "max_transition_change": 5}
        result = trace(body)
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")
        self.assertNotIn("upper_horizon", result)
        self.assertNotIn("lower_horizon", result)
        self.assertNotIn("pinchout", result)

    def test_second_diff_rule_unchanged_across_pinchout(self):
        # 跨段三列二阶差仍按 limits.max_second_diff 硬约束：
        # 剖面最优轨迹最大二阶差为 6，收紧到 5 即无解。
        body = payload(pinchout_columns(), dict(PINCH_LIMITS, max_second_diff=5))
        body["pinchout"] = dict(PINCHOUT)
        result = trace(body)
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")

    def test_max_columns_bounds_segment_length(self):
        # 尖灭段长度 3：max_columns=3 可解且段长恰为 3；收紧到 2 无解。
        columns3 = pinchout_columns(n=9, start=3, merges=(15, 17, 19))
        body = payload(columns3, PINCH_LIMITS)
        body["pinchout"] = {"max_columns": 3, "max_transition_change": 6}
        result = trace(body)
        self.assertTrue(result["feasible"])
        self.assertEqual(result["pinchout"],
                         {"start_column": 3, "end_column": 5, "length": 3})

        body["pinchout"] = {"max_columns": 2, "max_transition_change": 6}
        result = trace(body)
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")

    def test_thick_smooth_profile_cannot_close(self):
        # 厚度 8 的平滑剖面任何单列都无法同时贴近上/下界面：
        # 启用 pinchout 后不存在完整闭合再重开轨迹 -> no_solution。
        body = payload(smooth_columns())
        body["pinchout"] = {"max_columns": 3, "max_transition_change": 8}
        result = trace(body)
        self.assertFalse(result["feasible"])
        self.assertEqual(result["status"], "no_solution")

    def test_pinchout_validation_errors(self):
        base = payload(smooth_columns())
        for bad in (3, "x", [], True):
            with self.assertRaises(ValidationError):
                parse_request({**base, "pinchout": bad})
        for bad_pinch in (
            {},                                     # 缺字段
            {"max_columns": 2},                     # 缺 max_transition_change
            {"max_transition_change": 1},           # 缺 max_columns
            {"max_columns": 0, "max_transition_change": 1},
            {"max_columns": 4, "max_transition_change": 1},
            {"max_columns": 1.5, "max_transition_change": 1},
            {"max_columns": True, "max_transition_change": 1},
            {"max_columns": 2, "max_transition_change": -1},
            {"max_columns": 2, "max_transition_change": 1.5},
            {"max_columns": 2, "max_transition_change": "1"},
        ):
            with self.assertRaises(ValidationError, msg=str(bad_pinch)):
                parse_request({**base, "pinchout": bad_pinch})

        # 边界合法值：max_columns 1~3、max_transition_change 非负（含 0）。
        for good in (
            {"max_columns": 1, "max_transition_change": 0},
            {"max_columns": 3, "max_transition_change": 0},
            {"max_columns": 2, "max_transition_change": 12},
        ):
            _, _, pinch = parse_request({**base, "pinchout": good})
            self.assertEqual(pinch, good)


def _brute_force_window(columns, col_states, limits, transition_change=None):
    """固定尖灭段窗口后穷举全部联合状态序列，返回最优键或 None。"""
    n = len(columns)
    best = None

    def rec(ci, ranks, pdu, pdl, ppdu, ppdl, neg_conf, max_sec, travel):
        nonlocal best
        if ci == n:
            key = (neg_conf, max_sec, travel, tuple(ranks))
            if best is None or key < best:
                best = key
            return
        for rank, (ui, li) in enumerate(col_states[ci]):
            up, lo = columns[ci][ui], columns[ci][li]
            thick = lo["depth"] - up["depth"]
            conf = up["confidence"] + (lo["confidence"] if li != ui else 0)
            if pdu is not None:
                us = abs(up["depth"] - pdu)
                ls = abs(lo["depth"] - pdl)
                if us > limits["max_slope"] or ls > limits["max_slope"]:
                    continue
                pthick = pdl - pdu
                if (transition_change is not None
                        and (pthick == 0) != (thick == 0)):
                    tlimit = transition_change
                else:
                    tlimit = limits["max_thickness_change"]
                if abs(thick - pthick) > tlimit:
                    continue
                nmax, ntrav = max_sec, travel + us + ls
                if ppdu is not None:
                    cap2 = limits.get("max_second_diff")
                    cap2 = float("inf") if cap2 is None else cap2
                    su = abs(up["depth"] - 2 * pdu + ppdu)
                    sl = abs(lo["depth"] - 2 * pdl + ppdl)
                    if su > cap2 or sl > cap2:
                        continue
                    nmax = max(nmax, su, sl)
            else:
                nmax, ntrav = 0, 0
            rec(ci + 1, ranks + [rank], up["depth"], lo["depth"],
                pdu, pdl, neg_conf - conf, nmax, ntrav)

    rec(0, [], None, None, None, None, 0, 0, 0)
    return best


def brute_force_pinchout(columns, limits, pinchout):
    """尖灭模式穷举：枚举合法尖灭段窗口 + 全部联合状态序列。

    返回 ((最优键), (start, end))；None 表示无解。窗口枚举顺序与
    求解器一致（长度升序、起点升序），键全平时保留先出现者。
    """
    n = len(columns)
    best = None
    max_len = min(pinchout["max_columns"], n - 2)
    for length in range(1, max_len + 1):
        for start in range(1, n - length):
            col_states = [
                _build_states(c, limits, pinchout=start <= i < start + length)
                for i, c in enumerate(columns)
            ]
            if any(not s for s in col_states):
                continue
            key = _brute_force_window(
                columns, col_states, limits,
                pinchout["max_transition_change"],
            )
            if key is None:
                continue
            if best is None or key < best[0]:
                best = (key, (start, start + length))
    return best


def result_rank_path_pinchout(columns, limits, result):
    """尖灭结果 -> 每列在对应窗口 _build_states 中的状态序号。"""
    start = result["pinchout"]["start_column"]
    end = result["pinchout"]["end_column"] + 1
    ranks = []
    for i, c in enumerate(columns):
        index = {cand["id"]: k for k, cand in enumerate(c)}
        pair = (
            index[result["upper_horizon"][i]["id"]],
            index[result["lower_horizon"][i]["id"]],
        )
        states = _build_states(c, limits, pinchout=start <= i < end)
        ranks.append(states.index(pair))
    return tuple(ranks)


def random_pinchout_body(rng):
    body = random_body(rng)
    body["pinchout"] = {
        "max_columns": rng.randint(1, 3),
        "max_transition_change": rng.randint(0, 8),
    }
    return body


class PinchoutFuzzTests(unittest.TestCase):
    def test_matches_brute_force(self):
        rng = random.Random(20261004)
        feasible_hits = 0
        for _ in range(200):
            body = random_pinchout_body(rng)
            columns, limits, pinchout = parse_request(body)
            expected = brute_force_pinchout(columns, limits, pinchout)
            actual = solve(columns, limits, pinchout)
            if expected is None:
                self.assertFalse(actual["feasible"])
                self.assertNotIn("upper_horizon", actual)
                continue
            self.assertTrue(actual["feasible"])
            validate_result(columns, limits, actual, pinchout)
            expected_key, (exp_start, exp_end) = expected
            key = (
                -actual["verdict"]["total_confidence"],
                actual["verdict"]["max_second_diff"],
                actual["verdict"]["total_travel"],
                result_rank_path_pinchout(columns, limits, actual),
            )
            self.assertEqual(key, expected_key)
            self.assertEqual(
                (actual["pinchout"]["start_column"],
                 actual["pinchout"]["end_column"] + 1),
                (exp_start, exp_end),
            )
            feasible_hits += 1
        self.assertGreater(feasible_hits, 20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
