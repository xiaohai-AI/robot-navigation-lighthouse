"""方位与距离的中文表述（纯函数，供 tools.find_place 注入事实）。

为什么必须由代码算：
    实测 qwen3-vl:8b 收到"储藏间在哪"和"洗手间在哪"时，给出**一字不差**的同一句
    "在走廊南边，从这边走过去大概十米" —— 它没有真的用坐标，而是套了模板。
    8B 模型在"多实体区分"这件事上不可靠，而方位/距离本来就是我们已知的真值，
    让它自己估等于把确定性换成幻觉。

约定：
    方位用世界坐标的八方向（东/东北/北/…），因为家里的语义描述就是"走廊南边"这种。
    距离按欧氏直线距离给一位小数；不足 1 m 说"不到一米"。
"""
from __future__ import annotations

import math

_DIRS = ('东', '东北', '北', '西北', '西', '西南', '南', '东南')


def compass(dx: float, dy: float) -> str:
    """从 (dx, dy) 得到中文八方位。世界坐标 x 向东、y 向北。"""
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return '原地'
    ang = math.degrees(math.atan2(dy, dx)) % 360.0
    return _DIRS[int((ang + 22.5) // 45) % 8]


def distance_text(d: float) -> str:
    if d < 1.0:
        return '不到一米'
    if d < 10.0:
        return f'约 {d:.1f} 米'
    return f'约 {d:.0f} 米'


def relative_text(from_xy, to_xy) -> str:
    """'从你现在的位置：东北方向 约 4.2 米'。"""
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    d = math.hypot(dx, dy)
    if d < 0.3:
        return '就在你旁边'
    return f'{compass(dx, dy)}方向 {distance_text(d)}'
