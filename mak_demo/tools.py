"""把 semantic_map / nav_client 包成 MAF 能调用的 function tools。

三条铁律（这是防 LLM 幻觉的闸门，别绕过）：
  1. 工具只接受**中文文本**或 **object_id**，绝不接受 LLM 自己算出来的坐标。
     坐标只有一个来源：find_place 的返回值。
  2. 每个工具返回**字符串**（LLM 只读得懂文本），但内部判断全是确定性代码。
  3. 有副作用的工具（go_to / go_to_room）在动手前再校验一次 id 是否真实存在。

工具清单：
    list_rooms()                    有哪些房间
    find_place(text, room='')       查房间/物体坐标 + 可传给 go_to 的 object_id
    go_to(object_id)                开到某个物体旁边（贴到 0.55 m，用于"到旁边"）
    look_at(object_id)              开到观察站位（退到 1.40 m）+ 视觉确认它是什么
    go_to_room(room)                开到某个房间中心
    where_am_i()                    车现在在哪
"""
from __future__ import annotations

import math
import time

import bearing


def build_tools(sm, nav):
    """返回 (工具列表, 调用流水)。

    流水是测试断言的关键：可以证明"LLM 真的调了工具"，而不是它自己编了个答案。
    """
    log: list[dict] = []

    def _note(name, args, ret):
        log.append({'tool': name, 'args': args, 'ret': ret,
                    't': time.strftime('%H:%M:%S')})

    # ---------------- 只读工具 ----------------
    def list_rooms() -> str:
        """列出这户人家所有房间的中文名和中心坐标。只在用户问"有哪些房间"时需要。"""
        lines = []
        for r in sm.rooms:
            cx, cy = r['center'][0], r['center'][1]
            lines.append(f'{r["label_zh"]}（{r["name"]}）中心 ({cx:+.2f}, {cy:+.2f})')
        out = '房间列表：\n' + '\n'.join(lines)
        _note('list_rooms', {}, out)
        return out

    def find_place(text: str, room: str = '') -> str:
        """查一个地点的坐标。text 填用户说的中文名词，例如 冰箱 / 沙发 / 洗手间 / 书架 / 客厅。
        room 可选，用来在多个同类物体里限定房间，例如 卧室2 / 书房 / 厨房。
        返回候选物体（含 object_id、中心坐标、可站位姿）或房间信息。"""
        t = (text or '').strip()
        if not t:
            out = '❌ text 不能为空'
            _note('find_place', {'text': text, 'room': room}, out)
            return out

        r = sm.resolve_room(t) or (sm.resolve_room(room) if room else None)
        objs = sm.resolve_object(t, room=r['name'] if r else None)

        # 方位/距离由代码算好喂进去：实测 8B 模型会自己瞎估（"走廊南边十米"用在了
        # 两个不同房间上），而这两个量本来就是我们已知的真值。
        try:
            here = nav.pose()
            here = (here[0], here[1]) if here else None
        except Exception:
            here = None

        parts = [f'查询「{t}」' + (f'（限定房间 {room}）' if room else '')]
        if r:
            cx, cy = r['center']
            rel = f'；从你现在的位置看：{bearing.relative_text(here, (cx, cy))}' if here else ''
            parts.append(f'房间匹配：{r["label_zh"]}（{r["name"]}）中心 ({cx:+.2f}, {cy:+.2f})'
                         f'{rel}'
                         f' —— 要去这个房间请调 go_to_room(room="{r["name"]}")')
        else:
            parts.append('房间匹配：无')
        if not objs:
            parts.append('物体匹配：无（放弃开车，直接告诉用户没找到）')
        else:
            parts.append(f'物体候选 {len(objs)} 个（按匹配度降序）：')
            for i, (o, score, why) in enumerate(objs[:5], 1):
                pose = sm.standoff_pose(o)
                if pose is None:
                    parts.append(f'  {i}. {o["id"]} {sm.class_zh(o["class"])} '
                                 f'房间 {o["room"]} 分数 {score} [{why}] '
                                 f'中心({o["x"]:+.2f},{o["y"]:+.2f}) 站姿：无（不可达）')
                    continue
                px, py, _yaw, in_room = pose
                d = math.hypot(px - o['x'], py - o['y'])
                rel = (f' 从你现在的位置看：{bearing.relative_text(here, (o["x"], o["y"]))}'
                       if here else '')
                parts.append(
                    f'  {i}. object_id={o["id"]} {sm.class_zh(o["class"])} '
                    f'房间 {o["room"]} 分数 {score} [{why}] '
                    f'中心({o["x"]:+.2f},{o["y"]:+.2f}){rel} '
                    f'站姿({px:+.2f},{py:+.2f}){"（房间内）" if in_room else "（门口）"} '
                    f'站姿距中心 {d:.2f} m')
            parts.append('要开车就把最合适的 object_id 原样传给 go_to。')
        out = '\n'.join(parts)
        _note('find_place', {'text': text, 'room': room}, out)
        return out

    def where_am_i() -> str:
        """机器人现在在哪（map 坐标系的位置和朝向）。"""
        p = nav.pose()
        if not p:
            out = '❌ 收不到车的位置（agent_bridge 在跑吗？）'
        else:
            out = (f'车现在在 map 系 ({p[0]:+.2f}, {p[1]:+.2f})，'
                   f'朝向 {math.degrees(p[2]):+.1f}°')
        _note('where_am_i', {}, out)
        return out

    # ---------------- 有副作用的工具 ----------------
    def go_to(object_id: str) -> str:
        """把机器人开到某个物体的可站位姿。object_id 必须原样来自 find_place 的返回值，
        不能自己编。命令会阻塞到到达或失败（最多约 3 分钟），
        返回最终状态、耗时、以及终点离物体中心多远。"""
        obj = sm.object_by_id((object_id or '').strip())
        if obj is None:
            out = (f'❌ 没有 object_id={object_id!r}。'
                   f'请先调用 find_place 拿到准确的 object_id，不要自己编。')
            _note('go_to', {'object_id': object_id}, out)
            return out

        pose = sm.standoff_pose(obj)
        if pose is None:
            out = f'❌ {obj["id"]} 周围找不到可站的位姿，放弃。'
            _note('go_to', {'object_id': object_id}, out)
            return out

        x, y, yaw, in_room = pose
        d_plan = math.hypot(x - obj['x'], y - obj['y'])
        t0 = time.time()
        state, st = nav.go_to(x, y, yaw, timeout=180.0)
        el = time.time() - t0

        p = st.get('pose') or nav.pose()
        d_now = math.hypot(p[0] - obj['x'], p[1] - obj['y']) if p else float('nan')
        ok = state == 'succeeded'
        out = (f'{"✅" if ok else "❌"} 去 {sm.class_zh(obj["class"])}（{obj["id"]}）'
               f'{"" if ok else " 未到达"}，耗时 {el:.1f}s，状态 {state}，'
               f'恢复 {st.get("recoveries", 0)} 次；'
               f'终点 ({p[0]:+.2f}, {p[1]:+.2f})，距物体中心 {d_now:.2f} m'
               f'（规划站姿 {d_plan:.2f} m）')
        _note('go_to', {'object_id': obj['id'], 'goal': [x, y, yaw]}, out)
        return out

    def look_at(object_id: str) -> str:
        """开到某个物体的"观察站位"（比站位再退 1.4 m，让门缝/把手这类特征进画面），
        然后拍一帧问视觉模型"这是什么"，并与语义地图的预期比对。

        object_id 必须原样来自 find_place 的返回值。
        用这个而不是 go_to 当用户说"看看 X / 确认 X"——贴到 0.55 m 时相机只拍到面板，
        视觉模型会把冰箱认成门（实测）。
        """
        obj = sm.object_by_id((object_id or '').strip())
        if obj is None:
            out = f'❌ 没有 object_id={object_id!r}。请先调用 find_place 拿到准确的 object_id。'
            _note('look_at', {'object_id': object_id}, out)
            return out

        pose = sm.observation_pose(obj)
        if pose is None:
            out = f'❌ {obj["id"]} 周围找不到可站的观察位姿。'
            _note('look_at', {'object_id': object_id}, out)
            return out

        x, y, yaw, _in_room = pose
        rad = max(float(obj.get('w', 0.4)), float(obj.get('d', 0.4))) / 2.0
        t0 = time.time()
        state, _st = nav.go_to(x, y, yaw, timeout=180.0)
        el = time.time() - t0
        if state != 'succeeded':
            out = (f'❌ 没能到达 {sm.class_zh(obj["class"])} 的观察站位'
                   f'（{state}，耗时 {el:.1f}s），没法做视觉确认。')
            _note('look_at', {'object_id': obj['id'], 'goal': [x, y, yaw]}, out)
            return out

        expected = sm.class_zh(obj['class'])
        try:
            import vision
            v = vision.verify_object(expected)
        except Exception as e:                                # 相机/模型不可用不该让任务崩
            out = f'✅ 已到观察站位，但视觉确认失败：{e}'
            _note('look_at', {'object_id': obj['id'], 'goal': [x, y, yaw]}, out)
            return out

        if v['match'] is True:
            verdict = f'视觉确认一致 ✅（VLM 认为主体是「{v["answer"]}」）'
        elif v['match'] is False:
            verdict = (f'⚠️ 视觉与语义地图不符：地图说是「{expected}」，'
                       f'VLM 说是「{v["answer"]}」')
        else:
            verdict = f'视觉无结论（VLM 答「{v["answer"] or "空"}」）'

        out = (f'✅ 已到 {expected}（{obj["id"]}）的观察站位，距中心 {rad + 1.40:.2f} m，'
               f'耗时 {el:.1f}s；{verdict}，依据：{v["evidence"]}；'
               f'VLM {v["latency"]:.1f}s，帧 {v["frame"]}')
        _note('look_at', {'object_id': obj['id'], 'goal': [x, y, yaw]}, out)
        return out

    def check_path() -> str:
        """检查"正前方这条路现在能不能过去"：雷达扇区量测 + 相机视觉，双源对照。

        用这个回答"前面能过去吗 / 这条路通不通 / 有东西挡着吗"。
        它不是导航——不会开车，只是看一眼、量一下，然后给结论。
        """
        import passability

        # ① 雷达（几何真值）：复用 agent 已有的 rosbridge 连接
        try:
            scan = passability.read_scan(link=getattr(nav, 'link', None))
            meas = passability.forward_clearance(scan)
            radar_txt = passability.describe(meas)
        except Exception as e:
            meas = {'verdict': 'no_data'}
            radar_txt = f'雷达量测失败：{e}'

        # ② 视觉（补雷达盲区：贴地线缆/门槛/玻璃）
        try:
            import vision
            v = vision.assess_passability()
            vis_txt = f'答案「{v["answer"] or "空"}」，依据「{v["evidence"]}」，{v["latency"]:.1f}s'
            vis_verdict = {'畅通': 'clear', '被挡住': 'blocked'}.get(v['answer'], 'unknown')
        except Exception as e:
            v = {'frame': '', 'latency': 0.0}
            vis_txt = f'视觉失败：{e}'
            vis_verdict = 'unknown'

        # ③ 合并：雷达是几何真值，优先；视觉只补充它能看见而雷达看不见的东西
        rv = meas['verdict']
        if rv == 'blocked':
            verdict = '❌ 过不去 —— 雷达量到障碍几乎贴着车头'
            if vis_verdict == 'blocked':
                verdict += '，视觉也报告被挡住（两者一致）'
        elif rv == 'tight':
            verdict = f"⚠️ 勉强能过、风险高 —— 雷达量到只能再往前 {meas['free_until']:.2f} m"
            if vis_verdict == 'blocked':
                verdict += '，视觉也报告被挡住（两者一致）'
        elif rv == 'partial':
            verdict = (f"⚠️ 能走一小段，但前方不是通畅通道 —— 雷达量到还能前进 "
                       f"{meas['free_until']:.2f} m 就会碰到东西")
            if vis_verdict == 'blocked':
                verdict += '，视觉也看到阻挡物（两者一致）'
            elif vis_verdict == 'clear':
                verdict += '；视觉看着开着，但雷达的距离是硬约束'
        elif rv in ('open', 'no_return'):
            if vis_verdict == 'blocked':
                verdict = ('⚠️ 两者不一致：雷达说前方畅通、视觉说被挡住 —— '
                           '以雷达几何为准，但可能真有雷达看不见的薄/扁/透明障碍，建议减速或换个方向')
            elif vis_verdict == 'clear':
                verdict = '✅ 可以通过 —— 雷达与视觉一致'
            else:
                verdict = '✅ 前方畅通 —— 雷达 3 m 内没有回波（视觉没给出结论）'
        elif rv == 'no_data':
            verdict = ('⚠️ 雷达数据不可用，只能参考视觉' if vis_verdict != 'unknown'
                       else '⚠️ 雷达与视觉都不可用，无法判断')
        else:
            verdict = f'⚠️ 未识别的雷达判定 {rv}'

        out = (f'可通行性核对（正前方）\n'
               f'  雷达：{radar_txt}\n'
               f'  视觉：{vis_txt}\n'
               f'  结论：{verdict}'
               + (f'\n  帧：{v["frame"]}' if v.get('frame') else ''))
        _note('check_path', {}, out)
        return out

    def go_to_room(room: str) -> str:
        """把机器人开到一个房间的中心。room 填英文房间名（living_room / kitchen / study /
        bedroom_1 / bedroom_2 / bathroom / storage / corridor），也可以填中文（客厅/厨房/书房…）。"""
        key = (room or '').strip()
        rd = sm.room_by_name(key) or sm.resolve_room(key)
        if rd is None:
            names = ', '.join(f'{r["label_zh"]}={r["name"]}' for r in sm.rooms)
            out = f'❌ 没有房间 {key!r}。可选：{names}'
            _note('go_to_room', {'room': room}, out)
            return out

        x, y = float(rd['center'][0]), float(rd['center'][1])
        t0 = time.time()
        state, st = nav.go_to(x, y, 0.0, timeout=180.0)
        el = time.time() - t0
        ok = state == 'succeeded'
        out = (f'{"✅" if ok else "❌"} 去 {rd["label_zh"]}（{rd["name"]}）中心，'
               f'耗时 {el:.1f}s，状态 {state}，剩余 {st.get("distance_remaining")} m')
        _note('go_to_room', {'room': rd['name'], 'goal': [x, y]}, out)
        return out

    return [list_rooms, find_place, where_am_i, go_to, look_at, check_path,
            go_to_room], log
