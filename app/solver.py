"""双反射界面联合追踪求解器。

每列必须同时选出严格分离的上/下两个候选点；相邻两列同时校验
两条界面的坡差与厚度变化，任何连续三列的界面二阶差也参与约束
与全局裁决。采用联合状态动态规划：一列的状态是一个
(上候选, 下候选) 有序对，转移时一次性检查两条界面的全部约束，
因此不会出现"先追一条界面、再为另一条补点"的伪轨迹。

可选的 pinchout（透镜体尖灭）模式：请求携带合法 pinchout 配置时，
轨迹必须包含且仅包含一个不含首末列的连续尖灭段——段内上下界面
拾取同一候选、厚度为零；段外仍严格分离并满足原厚度范围。跨越
尖灭边界的相邻列厚度变化改用 pinchout.max_transition_change，
其余坡差、厚度变化与二阶差规则不变。裁决目标与词典序完全沿用
既有定义（尖灭列的候选置信度只计入一次）。

最优性按词典序：
  1. 置信度总和最大
  2. 两条界面最大二阶差最小
  3. 两条界面总行程最小
  4. 逐列候选编号（上界面编号、下界面编号）字典序最小（稳定决胜）
"""

from __future__ import annotations

# 各字段规模上限：调用方提交 8~24 列，每列 3~8 个候选点。
MIN_COLS = 8
MAX_COLS = 24
MIN_CANDIDATES = 3
MAX_CANDIDATES = 8

# 尖灭段长度上限（闭区间列数）。
MIN_PINCHOUT_COLUMNS = 1
MAX_PINCHOUT_COLUMNS = 3


class ValidationError(ValueError):
    """请求数据不满足接口契约。"""


def _as_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(f"{name} 必须是整数")
    return value


def _parse_column(raw, index):
    """解析并校验单列数据，返回按深度排序的候选列表。"""
    if not isinstance(raw, dict):
        raise ValidationError(f"columns[{index}] 必须是对象")
    cands_raw = raw.get("candidates")
    if not isinstance(cands_raw, list):
        raise ValidationError(f"columns[{index}].candidates 必须是数组")
    if not (MIN_CANDIDATES <= len(cands_raw) <= MAX_CANDIDATES):
        raise ValidationError(
            f"columns[{index}] 候选点数量必须在 {MIN_CANDIDATES}~{MAX_CANDIDATES} 之间"
        )

    seen_ids = set()
    cands = []
    for j, item in enumerate(cands_raw):
        if not isinstance(item, dict):
            raise ValidationError(f"columns[{index}].candidates[{j}] 必须是对象")
        cid = item.get("id")
        if not isinstance(cid, str) or not cid:
            raise ValidationError(
                f"columns[{index}].candidates[{j}].id 必须是非空字符串"
            )
        if cid in seen_ids:
            raise ValidationError(f"columns[{index}] 内候选编号重复: {cid}")
        seen_ids.add(cid)

        depth = _as_int(
            item.get("depth"), f"columns[{index}].candidates[{j}].depth"
        )
        conf = _as_int(
            item.get("confidence"),
            f"columns[{index}].candidates[{j}].confidence",
        )
        if conf <= 0:
            raise ValidationError(
                f"columns[{index}].candidates[{j}].confidence 必须是正整数"
            )
        cands.append({"id": cid, "depth": depth, "confidence": conf})

    # 深度相同的候选不影响正确性（它们之间无法配成严格分离对），
    # 排序仅用于输出稳定与编号决胜。
    cands.sort(key=lambda c: (c["depth"], c["id"]))
    return cands


def _parse_limits(payload):
    if not isinstance(payload, dict):
        raise ValidationError("limits 必须是对象")

    def get(name):
        if name not in payload:
            raise ValidationError(f"limits.{name} 缺失")
        return _as_int(payload[name], f"limits.{name}")

    def get(name):
        if name not in payload:
            raise ValidationError(f"limits.{name} 缺失")
        return _as_int(payload[name], f"limits.{name}")

    raw_slope = get("max_slope")
    raw_tchange = get("max_thickness_change")
    for name, value in (
        ("max_slope", raw_slope),
        ("max_thickness_change", raw_tchange),
    ):
        if value < 0:
            raise ValidationError(f"limits.{name} 不能为负")

    # 二阶差上限可选：缺省表示不加二阶差硬约束（但仍作为裁决目标）。
    raw_second = payload.get("max_second_diff")
    if raw_second is not None:
        raw_second = _as_int(raw_second, "limits.max_second_diff")
        if raw_second < 0:
            raise ValidationError("limits.max_second_diff 不能为负")

    limits = {
        "max_thickness": get("max_thickness"),
        "min_thickness": get("min_thickness"),
        "max_slope": raw_slope,
        "max_thickness_change": raw_tchange,
        "max_second_diff": raw_second,
    }
    if limits["min_thickness"] < 1:
        raise ValidationError("limits.min_thickness 必须 >= 1")
    if limits["max_thickness"] < limits["min_thickness"]:
        raise ValidationError(
            "limits.max_thickness 不能小于 limits.min_thickness"
        )
    if limits["max_slope"] < 0:
        raise ValidationError("limits.max_slope 不能为负")
    return limits


def _parse_pinchout(payload):
    """解析可选 pinchout 配置；省略返回 None，非法按字段抛 422。"""
    if payload is None:
        return None
    if not isinstance(payload, dict):
        raise ValidationError("pinchout 必须是对象")

    def get(name):
        if name not in payload:
            raise ValidationError(f"pinchout.{name} 缺失")
        return _as_int(payload[name], f"pinchout.{name}")

    max_columns = get("max_columns")
    if not (MIN_PINCHOUT_COLUMNS <= max_columns <= MAX_PINCHOUT_COLUMNS):
        raise ValidationError(
            f"pinchout.max_columns 必须在 {MIN_PINCHOUT_COLUMNS}"
            f"~{MAX_PINCHOUT_COLUMNS} 之间"
        )
    max_transition_change = get("max_transition_change")
    if max_transition_change < 0:
        raise ValidationError("pinchout.max_transition_change 不能为负")
    return {
        "max_columns": max_columns,
        "max_transition_change": max_transition_change,
    }


def parse_request(payload):
    """校验请求体，返回 (columns, limits, pinchout)。"""
    if not isinstance(payload, dict):
        raise ValidationError("请求体必须是 JSON 对象")
    cols_raw = payload.get("columns")
    if not isinstance(cols_raw, list):
        raise ValidationError("columns 必须是数组")
    if not (MIN_COLS <= len(cols_raw) <= MAX_COLS):
        raise ValidationError(f"columns 数量必须在 {MIN_COLS}~{MAX_COLS} 之间")

    columns = [_parse_column(col, i) for i, col in enumerate(cols_raw)]
    limits = _parse_limits(payload.get("limits"))
    pinchout = _parse_pinchout(payload.get("pinchout"))
    return columns, limits, pinchout


def _build_states(column, limits, pinchout=False):
    """构造单列的全部合法联合状态：(上候选下标, 下候选下标)。

    pinchout=True 时构造尖灭列状态：上下界面拾取同一候选（厚度 0）；
    否则要求严格分离且厚度落在 limits 范围内。
    """
    states = []
    n = len(column)
    if pinchout:
        for ui in range(n):
            states.append((ui, ui))
    else:
        for ui in range(n):
            upper = column[ui]
            for li in range(n):
                if li == ui:
                    continue
                lower = column[li]
                thickness = lower["depth"] - upper["depth"]
                if limits["min_thickness"] <= thickness <= limits["max_thickness"]:
                    states.append((ui, li))
    # 决胜稳定用：编号字典序（上编号, 下编号）。
    states.sort(key=lambda s: (column[s[0]]["id"], column[s[1]]["id"]))
    return states


def _better(candidate, current):
    """固定二阶差上限下的词典序比较：(-累计置信度, 总行程) 越小越优，
    全平则按逐列编号状态序号路径决胜。"""
    if current is None:
        return True
    (ck, cpath), (bk, bpath) = candidate, current
    if ck != bk:
        return ck < bk
    return cpath < bpath


def _run_dp(columns, col_states, limits, cap, collect_costs=False, pinchout=None):
    """在"每条连续三列的二阶差 <= cap"硬约束下做分层状态 DP。

    固定 cap 后只剩置信度（最大化）与行程（最小化）两个可加目标，
    每个 (上一列状态, 当前列状态) 只保留唯一词典序最优前缀。

    collect_costs=True 时顺带收集全部局部可行三元组产生的二阶差值
    （用于二分全局最优 cap 的候选集合）。

    pinchout 非 None 时，跨越尖灭边界的相邻列厚度变化改用
    pinchout["max_transition_change"]，其余规则不变。

    返回:
      feasible              是否存在全程可行路径
      best                  最优记录 ((neg_conf, travel), path) 或 None
      costs                 collect_costs 时为二阶差值集合，否则为空
    """
    n_cols = len(columns)
    costs = set()
    hard_cap = float("inf") if cap is None else cap
    # 第 0 层：rank -> ((neg_conf, travel), path)，每状态天然唯一。
    # 尖灭状态 (k, k) 的候选置信度只计一次。
    prev = {}
    for rank, (ui, li) in enumerate(col_states[0]):
        conf = columns[0][ui]["confidence"]
        if li != ui:
            conf += columns[0][li]["confidence"]
        prev[rank] = ((-conf, 0), [rank])

    for ci in range(1, n_cols):
        cur_col = columns[ci]
        prev_col = columns[ci - 1]
        cur_states = col_states[ci]
        prev_states = col_states[ci - 1]
        nxt = {}

        if ci == 1:
            def predecessors_of(pr):
                entry = prev.get(pr)
                return ((None, entry),) if entry is not None else ()
        else:
            by_prev_rank = {}
            for (ppr, qpr), entry in prev.items():
                by_prev_rank.setdefault(qpr, []).append((ppr, entry))

            def predecessors_of(pr):
                return by_prev_rank.get(pr, ())

        for rnk, (ui, li) in enumerate(cur_states):
            up, lo = cur_col[ui], cur_col[li]
            thickness = lo["depth"] - up["depth"]
            conf_pair = up["confidence"]
            if li != ui:
                conf_pair += lo["confidence"]

            for pr, (pui, pli) in enumerate(prev_states):
                pup, plo = prev_col[pui], prev_col[pli]
                uslope = abs(up["depth"] - pup["depth"])
                lslope = abs(lo["depth"] - plo["depth"])
                if uslope > limits["max_slope"] or lslope > limits["max_slope"]:
                    continue
                pthick = plo["depth"] - pup["depth"]
                # 跨越尖灭边界的相邻列改用 pinchout 过渡限值。
                if pinchout is not None and (pui == pli) != (ui == li):
                    tchange_limit = pinchout["max_transition_change"]
                else:
                    tchange_limit = limits["max_thickness_change"]
                if abs(thickness - pthick) > tchange_limit:
                    continue

                records = predecessors_of(pr)
                if ci >= 2 and collect_costs:
                    for ppui0, ppli0 in col_states[ci - 2]:
                        cu = abs(up["depth"] - 2 * pup["depth"]
                                 + columns[ci - 2][ppui0]["depth"])
                        cl = abs(lo["depth"] - 2 * plo["depth"]
                                 + columns[ci - 2][ppli0]["depth"])
                        c = max(cu, cl)
                        if c <= hard_cap:
                            costs.add(c)

                for ppr, (old_key, path) in records:
                    if ci >= 2:
                        ppui, ppli = col_states[ci - 2][ppr]
                        sec_u = abs(
                            up["depth"] - 2 * pup["depth"]
                            + columns[ci - 2][ppui]["depth"]
                        )
                        sec_l = abs(
                            lo["depth"] - 2 * plo["depth"]
                            + columns[ci - 2][ppli]["depth"]
                        )
                        if max(sec_u, sec_l) > hard_cap:
                            continue

                    neg_conf, travel = old_key
                    cand = (
                        (neg_conf - conf_pair, travel + uslope + lslope),
                        path + [rnk],
                    )
                    state_key = (pr, rnk)
                    if _better(cand, nxt.get(state_key)):
                        nxt[state_key] = cand
        prev = nxt
        if not prev:
            return False, None, costs

    best = None
    for entry in prev.values():
        if _better(entry, best):
            best = entry
    return best is not None, best, costs


def solve(columns, limits, pinchout=None):
    """联合追踪求解。

    策略（严格按裁决词典序）：
      1. 在用户给定上限 L 下跑一次 DP 取得全局最大置信度 C*，同时收集
         可达的局部二阶差值；上限 L 下不可行即明确无解（不输出轨迹）；
      2. 对候选二阶差值二分：求最小的 K，使"在硬约束 K 下仍能达到
         置信度 C*"（可达置信度对 K 单调）；
      3. 在 K 下做最终（置信度, 行程, 编号路径）词典序 DP。

    pinchout 非 None 时，轨迹必须包含且仅包含一个不含首末列、长度
    1~max_columns 的连续尖灭段（段内上下界面拾取同一候选、厚度为 0，
    段外严格分离）；跨越尖灭边界的厚度变化改用
    pinchout["max_transition_change"]。实现上对每个候选尖灭段窗口
    各跑一次上述流程，再按同一词典序对全部窗口的结果整体裁决，
    因此尖灭段的位置与长度本身也参与全局最优选择。
    """
    n_cols = len(columns)
    if pinchout is None:
        windows = [None]
    else:
        max_len = min(pinchout["max_columns"], n_cols - 2)
        windows = [
            (start, start + length)
            for length in range(1, max_len + 1)
            for start in range(1, n_cols - length)
        ]

    # 第一遍：每个窗口在用户上限下求最大置信度，同时收集二阶差候选。
    # 置信度是裁决第一关键字，达不到全局最大置信度的窗口不可能胜出，
    # 其后续二分与最终 DP 直接剪枝（不改变裁决结果）。
    window_runs = []
    best_conf = None
    for window in windows:
        if window is None:
            col_states = [_build_states(col, limits) for col in columns]
        else:
            start, end = window
            col_states = [
                _build_states(col, limits, pinchout=start <= ci < end)
                for ci, col in enumerate(columns)
            ]
        if any(not states for states in col_states):
            continue

        cap_limit = limits.get("max_second_diff")
        feasible, best_l, costs = _run_dp(
            columns, col_states, limits, cap_limit,
            collect_costs=True, pinchout=pinchout,
        )
        if not feasible:
            continue
        max_confidence = -best_l[0][0]
        window_runs.append((window, col_states, max_confidence, costs))
        if best_conf is None or max_confidence > best_conf:
            best_conf = max_confidence

    if not window_runs:
        return {"feasible": False}

    # 第二遍：达到全局最大置信度的窗口按同一词典序整体裁决。
    best_overall = None
    for window, col_states, max_confidence, costs in window_runs:
        if max_confidence < best_conf:
            continue

        # 0 始终是候选（二阶差可能恰好为 0；列数 >= 8 必有连续三列）。
        candidates = sorted(costs | {0})

        def reaches_max_confidence(cap):
            ok, best, _ = _run_dp(
                columns, col_states, limits, cap, pinchout=pinchout
            )
            return ok and -best[0][0] == max_confidence

        lo_i, hi_i = 0, len(candidates) - 1
        while lo_i < hi_i:
            mid = (lo_i + hi_i) // 2
            if reaches_max_confidence(candidates[mid]):
                hi_i = mid
            else:
                lo_i = mid + 1
        optimal_cap = candidates[lo_i]

        _, best, _ = _run_dp(
            columns, col_states, limits, optimal_cap, pinchout=pinchout
        )
        (neg_conf, total_travel), ranks = best
        key = (neg_conf, optimal_cap, total_travel, tuple(ranks))
        if best_overall is None or key < best_overall[0]:
            best_overall = (key, window, col_states, ranks)

    (_, optimal_cap, total_travel, _), window, col_states, ranks = best_overall
    path = [col_states[ci][rank] for ci, rank in enumerate(ranks)]
    total_conf = -best_overall[0][0]
    return _build_result(
        columns, path, total_conf, optimal_cap, total_travel, window
    )


def _build_result(columns, path, total_conf, max_second_diff, total_travel,
                  pinchout_window=None):
    upper, lower, thicknesses = [], [], []
    u_slopes, l_slopes = [], []
    u_seconds, l_seconds = [], []

    for ci, (ui, li) in enumerate(path):
        up, lo = columns[ci][ui], columns[ci][li]
        upper.append({"column": ci, "id": up["id"], "depth": up["depth"]})
        lower.append({"column": ci, "id": lo["id"], "depth": lo["depth"]})
        thicknesses.append(
            {"column": ci, "thickness": lo["depth"] - up["depth"]}
        )
        if ci >= 1:
            pu = columns[ci - 1][path[ci - 1][0]]["depth"]
            pl = columns[ci - 1][path[ci - 1][1]]["depth"]
            u_slopes.append(
                {"from": ci - 1, "to": ci, "slope": up["depth"] - pu}
            )
            l_slopes.append(
                {"from": ci - 1, "to": ci, "slope": lo["depth"] - pl}
            )
        if ci >= 2:
            ppu = columns[ci - 2][path[ci - 2][0]]["depth"]
            ppl = columns[ci - 2][path[ci - 2][1]]["depth"]
            pu = columns[ci - 1][path[ci - 1][0]]["depth"]
            pl = columns[ci - 1][path[ci - 1][1]]["depth"]
            u_seconds.append(
                {"columns": [ci - 2, ci - 1, ci], "second_diff": up["depth"] - 2 * pu + ppu}
            )
            l_seconds.append(
                {"columns": [ci - 2, ci - 1, ci], "second_diff": lo["depth"] - 2 * pl + ppl}
            )

    result = {
        "feasible": True,
        "upper_horizon": upper,
        "lower_horizon": lower,
        "thicknesses": thicknesses,
        "slopes": {"upper": u_slopes, "lower": l_slopes},
        "second_diffs": {"upper": u_seconds, "lower": l_seconds},
        "verdict": {
            "total_confidence": total_conf,
            "max_second_diff": max_second_diff,
            "total_travel": total_travel,
        },
    }
    if pinchout_window is not None:
        start, end = pinchout_window
        result["pinchout"] = {
            "start_column": start,
            "end_column": end - 1,
            "length": end - start,
        }
    return result


def trace(payload):
    """供 HTTP 层调用的入口：校验 -> 求解。"""
    columns, limits, pinchout = parse_request(payload)
    result = solve(columns, limits, pinchout)
    if not result["feasible"]:
        result["status"] = "no_solution"
        if pinchout is None:
            result["message"] = "不存在满足全部约束的上下界面联合拾取组合"
        else:
            result["message"] = (
                "不存在满足全部约束且含完整闭合再重开尖灭段的"
                "上下界面联合拾取组合"
            )
    else:
        result["status"] = "ok"
    return result
