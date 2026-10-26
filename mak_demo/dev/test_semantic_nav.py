"""步骤 3 验收：中文指令 → 房间/物体 → 可站位姿 → 车真的开过去。

跑法（mak_demo 目录下）：
    .venv/bin/python dev/test_semantic_nav.py 冰箱
    .venv/bin/python dev/test_semantic_nav.py 沙发
    .venv/bin/python dev/test_semantic_nav.py 去卧室2的床边

验收三条：
    1  语义链路能打印出：房间 → 物体 → 位姿（每步都有理由）
    2  导航状态 succeeded，剩余距离不反弹
    3  终点到物体中心的距离落在 0.35~1.1 m（站姿 0.55 m + Nav2 容差 0.20 m）
"""
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from nav_client import NavClient              # noqa: E402
from rosbridge_link import RosbridgeLink       # noqa: E402
from semantic_map import SemanticMap           # noqa: E402

text = sys.argv[1] if len(sys.argv) > 1 else '冰箱'

sm = SemanticMap()
link = RosbridgeLink()
nav = NavClient(link)
if not nav.wait_ready():
    print('❌ 收不到 /agent/nav_status：agent_bridge 起了吗？')
    sys.exit(1)

print('=' * 70)
print(f'指令: 「{text}」')
print(f'车现在: {nav.pose()}')
print('=' * 70)

# ---- ① 语义解析 ----
room = sm.resolve_room(text)
objs = sm.resolve_object(text, room=room['name'] if room else None)
d_plan = 0.0        # 规划站姿到物体中心的距离（无物体时走"房间中心"退化分支，不用它）

if room:
    print(f'① 房间 : {room["label_zh"]}（{room["name"]}）中心 '
          f'({room["center"][0]:+.2f}, {room["center"][1]:+.2f})')
else:
    print('① 房间 : 未提到具体房间')

if not objs:
    print('① 物体 : 没匹配到任何物体')
    if room:
        cx, cy = room['center']
        print(f'   → 退化到"去房间中心" ({cx:.2f}, {cy:.2f})')
        target, why, obj = (cx, cy, 0.0), '房间中心（没说具体物体）', None
    else:
        print('❌ 既没房间也没物体，无法执行')
        sys.exit(1)
else:
    print(f'① 物体 : 匹配 {len(objs)} 个，取分最高的')
    for o, sc, why in objs[:3]:
        print(f'        {o["id"]:20s} {sm.class_zh(o["class"]):6s} '
              f'分数 {sc:.1f}  [{why}]')
    obj, _sc, why = objs[0]

    # ---- ② 可站位姿 ----
    pose = sm.standoff_pose(obj, room=room)
    if pose is None:
        print(f'❌ {obj["id"]} 找不到可站位姿（周围被墙围死）')
        sys.exit(1)
    x, y, yaw, inside = pose
    target = (x, y, yaw)
    print(f'② 物体中心 ({obj["x"]:+.2f}, {obj["y"]:+.2f})  依据: {why}')
    print(f'② 可站位姿 ({x:+.2f}, {y:+.2f}) yaw={yaw:+.2f}'
          f'  {"在房间内 ✅" if inside else "在房间外（门口）"}')
    d_plan = math.hypot(x - obj['x'], y - obj['y'])
    print(f'   站姿距物体中心 {d_plan:.2f} m'
          f'（物体半径 {max(obj.get("w", 0.4), obj.get("d", 0.4)) / 2:.2f} + 0.55）')

# ---- ③ 导航 ----
print('③ 开始导航 …')
t0 = time.time()
state, st = nav.go_to(target[0], target[1], target[2], timeout=240.0)
el = time.time() - t0

print()
print('=' * 70)
print(f'结果: {state}   耗时 {el:.1f}s   恢复 {st.get("recoveries")}')
print(f'终点位姿: {st.get("pose")}')

ok = (state == 'succeeded')
if obj is not None and st.get('pose'):
    fx, fy = st['pose'][0], st['pose'][1]
    d = math.hypot(fx - obj['x'], fy - obj['y'])
    # 判据必须相对"规划站姿"来判，不能拍绝对区间：
    #   Nav2 的 xy_goal_tolerance = 0.20 m，所以最终落点允许偏离规划站姿 ±0.20 m。
    #   原来写 0.35~1.1 的硬区间，在站姿 0.94 m + 偏差 0.17 m 时就误判失败（实测 1.11 m）。
    err = abs(d - d_plan)
    not_inside = d >= max(obj.get('w', 0.4), obj.get('d', 0.4)) / 2 + 0.15
    good = err <= 0.30 and not_inside
    print(f'终点到 {obj["id"]} 中心距离: {d:.2f} m（规划站姿 {d_plan:.2f} m，'
          f'偏差 {err:.2f} m，容差 0.30 = Nav2 0.20 + 余量 0.10）')
    print(f'   未撞进物体: {"✅" if not_inside else "❌ 太近了"}'
          f'   {"✅ 落点符合预期" if good else "⚠️ 偏离规划站姿过多"}')
    ok = ok and good

print('=' * 70)
print('步骤 3 通过 ✅' if ok else '步骤 3 未通过 ❌')
link.close()
sys.exit(0 if ok else 1)
