"""离线验收可通行性：扇区量测数学 + 指令路由（不连 ROS、不开车）。

A 组：用合成 LaserScan 检查 forward_clearance / describe 的判据
B 组：检查"能不能过去"类指令是否被正确路由到模式 E

跑法：.venv/bin/python dev/check_passability.py
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import passability                                              # noqa: E402
from agent_nav import classify_intent, wants_pass, wants_look    # noqa: E402

N = 360
A0 = -math.pi
INC = 2 * math.pi / N


def scan_with(fn):
    """按角度生成合成扫描：fn(angle_deg) -> 距离。"""
    rs = [fn(math.degrees(A0 + i * INC)) for i in range(N)]
    return {'angle_min': A0, 'angle_increment': INC,
            'angle_max': math.pi, 'range_min': 0.12, 'range_max': 12.0, 'ranges': rs}


def wall_at(d, widths=(-90, 90)):
    """在给定角度范围内有距离 d 的墙，其余方向回波 8 m。"""
    lo, hi = widths
    return scan_with(lambda a: d if lo <= a <= hi else 8.0)


CASES = [
    ('正前方 0.30 m 有墙', wall_at(0.30, (-30, 30)), 'blocked', 0.30),
    ('正前方 0.60 m 有墙', wall_at(0.60, (-30, 30)), 'tight', 0.60),
    ('正前方 1.50 m 有墙（能走一段，但不算通道）', wall_at(1.50, (-30, 30)), 'partial', 1.50),
    ('正前方空旷（8 m 外才有墙）', wall_at(8.0), 'no_return', None),
    ('障碍只在左侧 40°（不在扇区内）', scan_with(lambda a: 0.25 if 35 <= a <= 45 else 8.0),
     'no_return', None),
    ('贴车身 0.05 m 的回波（自身遮挡，应被忽略）',
     scan_with(lambda a: 0.05 if -10 <= a <= 10 else 8.0), 'no_return', None),
    ('NaN / null 混在里面', {**wall_at(1.5, (-30, 30)),
                            'ranges': [None if i % 7 == 0 else (1.5 if abs(math.degrees(A0 + i * INC)) <= 30 else 8.0)
                                       for i in range(N)]}, 'partial', 1.5),
    # ↓ 2026-10-23 真机 bug 的回归用例：出生点那 10 个回波是车壳自身
    ('只有车体自遮挡回波（0.13 m @ -25°，实测踩过）',
     scan_with(lambda a: 0.13 if -31 <= a <= -24 else 8.0), 'no_return', None),
    ('真实障碍在车体外的 0.30 m 正前方', wall_at(0.30, (-8, 8)), 'blocked', 0.30),
    ('真实障碍 0.40 m 偏右 20°（车体外）',
     scan_with(lambda a: 0.40 if 17 <= a <= 23 else 8.0), 'tight', 0.40),
    ('完全没有回波（全 inf）', {'angle_min': A0, 'angle_increment': INC,
                              'ranges': [float('inf')] * N}, 'no_return', None),
    ('空消息', {}, 'no_data', None),
]

ROUTE_CASES = [
    ('前面能过去吗', 'ACT', True, False),
    ('这条路通不通', 'ACT', True, False),
    ('有东西挡着吗', 'ACT', True, False),
    ('门开着吗', 'ACT', True, False),
    ('去冰箱旁边', 'ACT', False, False),
    ('冰箱还在吗', 'ACT', False, True),
    ('洗手间在哪', 'ASK', False, False),
]


def main():
    ok = tot = 0
    print('=' * 74)
    print('A 组：扇区量测')
    for name, msg, want_verdict, want_dist in CASES:
        m = passability.forward_clearance(msg)
        good = m['verdict'] == want_verdict
        if want_dist is not None:
            good = good and m['min_dist'] is not None and abs(m['min_dist'] - want_dist) < 0.01
        tot += 1
        ok += good
        print(f'{"✅" if good else "❌"} {name:36s} → {m["verdict"]:10s} '
              f'dist={m["min_dist"]} free={m["free_until"]}')
        if not good:
            print(f'     期望 {want_verdict} / {want_dist}，实际 {m}')
        print(f'     {passability.describe(m)}')

    print()
    print('B 组：指令路由')
    for instr, want_intent, want_pass_, want_look_ in ROUTE_CASES:
        it = classify_intent(instr)
        p = wants_pass(instr)
        lk = wants_look(instr)
        good = (it == want_intent and p == want_pass_ and lk == want_look_)
        tot += 1
        ok += good
        print(f'{"✅" if good else "❌"} {instr:16s} 意图 {it:3s}  '
              f'可通行 {p}  看物体 {lk}')
        if not good:
            print(f'     期望 意图 {want_intent} / 可通行 {want_pass_} / 看物体 {want_look_}')

    print()
    print('=' * 74)
    print(f'{ok}/{tot} 通过')
    return 0 if ok == tot else 1


if __name__ == '__main__':
    sys.exit(main())
