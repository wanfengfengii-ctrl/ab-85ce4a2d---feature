"""双反射界面联合追踪求解器。

每列必须同时选出严格分离的上/下两个候选点；相邻两列同时校验
两条界面的坡差与厚度变化，任何连续三列的界面二阶差也参与约束
与全局裁决。采用联合状态动态规划：一列的状态是一个
(上候选, 下候选) 有序对，转移时一次性检查两条界面的全部约束，
因此不会出现"先追一条界面、再为另一条补点"的伪轨迹。

启用 pinchout 后，每列额外允许"上下界面为同一候选"的零厚度重合
状态；DP 以相位（段前 / 段内第 L 列 / 重开后）强制全程恰好出现
一个不含首末列、长度不超过 max_columns 的连续尖灭段。进入或离开
尖灭段的厚度变化改用 max_transition_change，其余约束不变。

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

# 尖灭段长度上限的取值范围（列数，不含首末列）。
PINCHOUT_MIN_COLUMNS = 1
PINCHOUT_MAX_COLUMNS = 3


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


def _parse_pinchout(raw):
    """校验可选的 pinchout 配置，返回归一化字典。

    必须同时给出 max_columns（1~3）与 max_transition_change（非负整数），
    二者都按字段名在 422 消息中定位。
    """
    if not isinstance(raw, dict):
        raise ValidationError("pinchout 必须是对象")
    if "max_columns" not in raw:
        raise ValidationError("pinchout.max_columns 缺失")
    if "max_transition_change" not in raw:
        raise ValidationError("pinchout.max_transition_change 缺失")
    max_columns = _as_int(raw["max_columns"], "pinchout.max_columns")
    if not (PINCHOUT_MIN_COLUMNS <= max_columns <= PINCHOUT_MAX_COLUMNS):
        raise ValidationError(
            f"pinchout.max_columns 必须是 {PINCHOUT_MIN_COLUMNS}"
            f"~{PINCHOUT_MAX_COLUMNS} 之间的整数"
        )
    max_transition_change = _as_int(
        raw["max_transition_change"], "pinchout.max_transition_change"
    )
    if max_transition_change < 0:
        raise ValidationError("pinchout.max_transition_change 不能为负")
    return {
        "max_columns": max_columns,
        "max_transition_change": max_transition_change,
    }


def parse_request(payload):
    """校验请求体，返回 (columns, limits, pinchout)。

    pinchout 省略时返回 None，求解走不允许零厚度的既有路径。
    """
    if not isinstance(payload, dict):
        raise ValidationError("请求体必须是 JSON 对象")
    cols_raw = payload.get("columns")
    if not isinstance(cols_raw, list):
        raise ValidationError("columns 必须是数组")
    if not (MIN_COLS <= len(cols_raw) <= MAX_COLS):
        raise ValidationError(f"columns 数量必须在 {MIN_COLS}~{MAX_COLS} 之间")

    columns = [_parse_column(col, i) for i, col in enumerate(cols_raw)]
    limits = _parse_limits(payload.get("limits"))
    pinchout = (
        _parse_pinchout(payload["pinchout"]) if "pinchout" in payload else None
    )
    return columns, limits, pinchout


def _build_states(column, limits):
    """构造单列的全部合法联合状态：(上候选下标, 下候选下标)。"""
    states = []
    n = len(column)
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


def _build_pinch_states(column, limits):
    """尖灭模式的单列状态：(上候选下标, 下候选下标, 种类)。

    种类 0 为严格分离的普通对（厚度仍在 min/max 限值内）；种类 1 为
    上下界面共用同一候选的零厚度重合点。全部状态按
    （上编号、下编号）字典序统一排序，使第四级决胜与普通模式完全
    同口径——重合点 (k,k) 视为普通的编号有序对参与排序。
    """
    states = [(ui, li, 0) for ui, li in _build_states(column, limits)]
    n = len(column)
    states.extend((k, k, 1) for k in range(n))
    states.sort(key=lambda s: (column[s[0]]["id"], column[s[1]]["id"]))
    return states


def _state_parts(state):
    """归一化状态元组为 (上候选下标, 下候选下标, 种类)。"""
    if len(state) == 3:
        return state
    return state[0], state[1], 0


def _state_confidence(column, state):
    ui, li, _kind = _state_parts(state)
    # 裁决口径不变：两条界面各自所选候选的置信度之和。尖灭列两条界面
    # 恰好选中同一候选，该候选的置信度按两次选择计入总和。
    return column[ui]["confidence"] + column[li]["confidence"]


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
    """在每条连续三列的二阶差 <= cap 硬约束下做分层状态 DP。

    固定 cap 后只剩置信度（最大化）与行程（最小化）两个可加目标，
    每个 (上一列状态, 当前列状态) 只保留唯一词典序最优前缀。

    pinchout 为 None 时严格走原有双界面逻辑（状态为二元组、厚度恒正）；
    给出尖灭配置时状态为三元组，并在标签上附带相位：
      相位 0         段前普通列（严格分离）
      相位 10+L      尖灭段内第 L 列（上下同一候选、厚度 0）
      相位 2         重开后的普通列
    从而强制恰好一个不含首末列的连续尖灭段。进入/离开尖灭段的两条
    边界采用 max_transition_change 校验厚度变化，其余列间仍用
    max_thickness_change；坡差与二阶差规则不变。

    collect_costs=True 时顺带收集全部局部可行三元组产生的二阶差值
    （用于二分全局最优 cap 的候选集合）。

    返回:
      feasible              是否存在全程可行路径
      best                  最优记录 ((neg_conf, travel), path) 或 None
      costs                 collect_costs 时为二阶差值集合，否则为空
    """
    n_cols = len(columns)
    costs = set()
    hard_cap = float("inf") if cap is None else cap
    pinch_on = pinchout is not None
    pmax = pinchout["max_columns"] if pinch_on else 0
    transition_change = (
        pinchout["max_transition_change"] if pinch_on else None
    )

    if pinch_on:
        normal_ranks = [
            [r for r, s in enumerate(states) if s[2] == 0]
            for states in col_states
        ]
        pinch_ranks = [
            [r for r, s in enumerate(states) if s[2] == 1]
            for states in col_states
        ]

        def labels_at(ci):
            """列 ci 上允许出现的 (状态序号, 相位) 标签。"""
            if ci == 0:
                return [(r, 0) for r in normal_ranks[0]]
            if ci == n_cols - 1:
                # 末列必须已重开：只接受相位 2，尖灭段不可能抵达末列。
                return [(r, 2) for r in normal_ranks[n_cols - 1]]
            labels = [(r, 0) for r in normal_ranks[ci]]
            labels.extend((r, 2) for r in normal_ranks[ci])
            # 段长 L 要求段首列 ci-L+1 >= 1（不含首列）。
            for length in range(1, min(pmax, ci) + 1):
                labels.extend((r, 10 + length) for r in pinch_ranks[ci])
            return labels

        def rank_of(label):
            return label[0]
    else:
        def labels_at(ci):
            return range(len(col_states[ci]))

        def rank_of(label):
            return label

    # 第 0 层：label -> ((neg_conf, travel), path)，每状态天然唯一。
    prev = {}
    for label in labels_at(0):
        rank = rank_of(label)
        prev[label] = (
            (-_state_confidence(columns[0], col_states[0][rank]), 0),
            [rank],
        )

    for ci in range(1, n_cols):
        cur_col = columns[ci]
        prev_col = columns[ci - 1]
        cur_states = col_states[ci]
        prev_states = col_states[ci - 1]
        cur_labels = labels_at(ci)
        prev_labels = labels_at(ci - 1)
        nxt = {}

        if ci == 1:
            def records_for(label):
                entry = prev.get(label)
                return ((None, entry),) if entry is not None else ()
        else:
            by_prev_label = {}
            for (plabel, clabel), entry in prev.items():
                by_prev_label.setdefault(clabel, []).append((plabel, entry))

            def records_for(label):
                return by_prev_label.get(label, ())

        for clabel in cur_labels:
            rnk = rank_of(clabel)
            ui, li, kind = _state_parts(cur_states[rnk])
            up, lo = cur_col[ui], cur_col[li]
            thickness = 0 if kind == 1 else lo["depth"] - up["depth"]
            conf_pair = _state_confidence(cur_col, cur_states[rnk])

            for plabel in prev_labels:
                prnk = rank_of(plabel)
                pui, pli, pkind = _state_parts(prev_states[prnk])

                if pinch_on:
                    pphase, cphase = plabel[1], clabel[1]
                    if cphase == 0:
                        if pphase != 0:
                            continue
                        crossing = False
                    elif cphase == 2:
                        # 重开后延续，或正好离开尖灭段（后者算跨越边界）。
                        if pphase != 2 and pphase < 10:
                            continue
                        crossing = pphase >= 10
                    else:
                        length = cphase - 10
                        if pphase == 0:
                            if length != 1:
                                continue
                        elif pphase != 10 + length - 1:
                            continue
                        # 段内延续厚度 0->0；只有段首跨越算边界。
                        crossing = pphase == 0
                else:
                    crossing = False

                pup, plo = prev_col[pui], prev_col[pli]
                uslope = abs(up["depth"] - pup["depth"])
                lslope = abs(lo["depth"] - plo["depth"])
                if uslope > limits["max_slope"] or lslope > limits["max_slope"]:
                    continue
                pthick = 0 if pkind == 1 else plo["depth"] - pup["depth"]
                thick_limit = (
                    transition_change if crossing
                    else limits["max_thickness_change"]
                )
                if abs(thickness - pthick) > thick_limit:
                    continue

                records = records_for(plabel)
                if ci >= 2 and collect_costs:
                    for pp_state in col_states[ci - 2]:
                        ppui0, ppli0, _ = _state_parts(pp_state)
                        cu = abs(up["depth"] - 2 * pup["depth"]
                                 + columns[ci - 2][ppui0]["depth"])
                        cl = abs(lo["depth"] - 2 * plo["depth"]
                                 + columns[ci - 2][ppli0]["depth"])
                        c = max(cu, cl)
                        if c <= hard_cap:
                            costs.add(c)

                for pplabel, (old_key, path) in records:
                    if ci >= 2:
                        pprank = rank_of(pplabel)
                        ppui, ppli, _ = _state_parts(
                            col_states[ci - 2][pprank]
                        )
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
                    state_key = (plabel, clabel)
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

    pinchout 非空时状态为三元组，DP 额外强制唯一尖灭段。
    """
    if pinchout is None:
        col_states = [_build_states(col, limits) for col in columns]
    else:
        col_states = [_build_pinch_states(col, limits) for col in columns]
    if any(not states for states in col_states):
        return {"feasible": False}

    cap_limit = limits.get("max_second_diff")
    feasible, best_l, costs = _run_dp(
        columns, col_states, limits, cap_limit,
        collect_costs=True, pinchout=pinchout,
    )
    if not feasible:
        return {"feasible": False}
    max_confidence = -best_l[0][0]

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
    path = [col_states[ci][rank] for ci, rank in enumerate(ranks)]
    result = _build_result(
        columns, path, -neg_conf, optimal_cap, total_travel
    )
    if pinchout is not None:
        kinds = [_state_parts(state)[2] for state in path]
        start = kinds.index(1)
        end = start
        while end + 1 < len(kinds) and kinds[end + 1] == 1:
            end += 1
        result["pinchout"] = {
            "start_column": start,
            "end_column": end,
            "length": end - start + 1,
        }
    return result


def _build_result(columns, path, total_conf, max_second_diff, total_travel):
    upper, lower, thicknesses = [], [], []
    u_slopes, l_slopes = [], []
    u_seconds, l_seconds = [], []

    for ci, state in enumerate(path):
        ui, li, _kind = _state_parts(state)
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

    return {
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


def trace(payload):
    """供 HTTP 层调用的入口：校验 -> 求解。"""
    columns, limits, pinchout = parse_request(payload)
    result = solve(columns, limits, pinchout)
    if not result["feasible"]:
        result["status"] = "no_solution"
        result["message"] = "不存在满足全部约束的上下界面联合拾取组合"
    else:
        result["status"] = "ok"
    return result
