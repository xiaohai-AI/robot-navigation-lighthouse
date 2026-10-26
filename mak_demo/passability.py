"""可通行性判定：从 /scan 量测"正前方这条缝能不能过"。

为什么不用地图/代价地图：
    代价地图是 Nav2 内部状态，站位规划时用得上，但"现在这一秒正前方堵不堵"更直接的问题是
    传感器问题。用原始 /scan 做扇区量测，得到的是当下、无延迟、与规划器无关的事实。

为什么还要配 VLM：
    2D 雷达只在 0.132 m 那个水平面上扫，**看不到贴地的线缆、门槛、地毯边**，
    也看不到玻璃/透明物；但对"是否有一堵墙/一件家具"它是可靠的、毫秒级的。
    所以两者是互补关系：雷达给几何真值，视觉补"扁平/透明/低矮"这一类盲区。
    不一致时**以雷达为准**，但把视觉的怀疑如实报出来（可能真有雷达看不见的东西）。

机器人尺寸（Lighthouse 底盘）：宽 0.281 m，长 0.306 m → 取半径 0.21 m 做安全余量。
"""
from __future__ import annotations

import math

ROBOT_RADIUS = 0.21           # 车体半径 + 一点余量
SECTOR_DEG = 50.0             # 前方 ±25°，约等于车体宽度在 0.5 m 处张开的角
MIN_R = 0.05                  # 只是防 0 值；真正剔除自身回波靠下面的车体框判断
MAX_R = 3.0                   # 超过这个距离不参与"近处能否通过"的判断

# 车体几何（Lighthouse，model.sdf）：base_link 在车体中心，
# 2026-10-23 塔身改造后 base_scan 装在塔顶 z=0.78（水平位置仍是 base_link 前方 -0.064 m），
# 车体 0.306 x 0.281 → 半长 0.153、半宽 0.1405。
#
# 这个车体矩形过滤现在基本是"安全网"：0.78 m 平面上已经没有任何车体部件
# （塔顶座 r=0.040 与相机支架尖都在 0.12 m 死区内，雷达看不见）。
# 但保留它是有价值的 —— 历史上正是因为缺这个判断，"开阔走廊"被 0.124 m 的
# 自身回波判成"贴着车头有障碍"。将来若再往车体上装东西，这一层能兜住。
LIDAR_X = -0.064
LIDAR_Y = 0.0
HALF_X = 0.153
HALF_Y = 0.1405
SELF_MARGIN = 0.02


def to_floats(data) -> list:
    """rosbridge 对 float32[] 可能给 list / base64 / None（NaN 常被编码成 null）。"""
    if isinstance(data, list):
        out = []
        for v in data:
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                out.append(float('nan'))          # null / "nan" 都当 NaN
        return out
    if isinstance(data, dict):
        return to_floats(data.get('data', []))
    if isinstance(data, str):
        import base64
        import struct
        raw = base64.b64decode(data)
        n = len(raw) // 4
        return list(struct.unpack(f'<{n}f', raw))
    return []


def forward_clearance(msg: dict, sector_deg: float = SECTOR_DEG,
                      min_r: float = MIN_R, max_r: float = MAX_R) -> dict:
    """量正前方扇区内最近的障碍距离。

    返回：
        min_dist     扇区内最近有效回波距离（m）；没有回波则 None
        min_angle    最近回波的角度（度，相对车头，右正左负）
        beams        参与统计的有效点数
        free_until   min_dist - ROBOT_RADIUS（车心能前进到多远）；无回波为 None
        verdict      'blocked' / 'tight' / 'clear' / 'no_return'
    """
    ranges = to_floats(msg.get('ranges', []))
    if not ranges:
        return {'min_dist': None, 'min_angle': None, 'beams': 0,
                'free_until': None, 'verdict': 'no_data'}

    a0 = float(msg.get('angle_min', -math.pi))
    inc = float(msg.get('angle_increment', 0.0)) or (2 * math.pi / len(ranges))
    half = math.radians(sector_deg / 2.0)

    best_d, best_a, n, self_hits = None, None, 0, 0
    for i, r in enumerate(ranges):
        if not (r == r) or r in (float('inf'), float('-inf')) or r <= 0:   # NaN/Inf/0
            continue
        a = a0 + i * inc
        if abs(a) > half:
            continue
        if r < min_r or r > max_r:
            continue
        # 车体自遮挡：把回波点换算到 base_link 系，落在车体矩形内就丢掉
        bx = LIDAR_X + r * math.cos(a)
        by = LIDAR_Y + r * math.sin(a)
        if abs(bx) <= HALF_X + SELF_MARGIN and abs(by) <= HALF_Y + SELF_MARGIN:
            self_hits += 1
            continue
        n += 1
        if best_d is None or r < best_d:
            best_d, best_a = r, a

    if best_d is None:
        # 扇区内没有任何有效回波（或全被车体挡住）：正前方是空的
        return {'min_dist': None, 'min_angle': None, 'beams': n,
                'self_rejected': self_hits,
                'free_until': None, 'verdict': 'no_return'}

    free_until = best_d - ROBOT_RADIUS
    if free_until <= 0.15:
        verdict = 'blocked'          # 几乎贴上了
    elif free_until <= 0.60:
        verdict = 'tight'            # 勉强能过、风险高
    elif free_until <= 1.50:
        verdict = 'partial'          # 能走一段，但前面就是东西 —— 不是畅通通道
    else:
        verdict = 'open'
    ang = math.degrees(best_a) if best_a is not None else 0.0
    return {'min_dist': round(best_d, 3), 'min_angle': round(ang, 1),
            'beams': n, 'self_rejected': self_hits,
            'free_until': round(free_until, 3), 'verdict': verdict}


def describe(m: dict) -> str:
    """把量测结果说成一句中文（这是给 VLM/用户看的真值，不让模型自己编）。"""
    v = m['verdict']
    if v == 'no_data':
        return '没收到 /scan（雷达话题不通）'
    if v == 'no_return':
        extra = f"（已忽略 {m['self_rejected']} 个车体自遮挡回波）" if m.get('self_rejected') else ''
        return f"正前方 ±{SECTOR_DEG / 2:.0f}°、{MAX_R:.0f} m 内没有回波 —— 前方空旷{extra}"
    head = (f"正前方 ±{SECTOR_DEG / 2:.0f}° 内最近障碍 {m['min_dist']:.2f} m"
            f"（方位 {m['min_angle']:+.0f}°），车还能前进 {m['free_until']:.2f} m")
    tag = {
        'blocked': '→ 判定：过不去',
        'tight': '→ 判定：勉强能过、风险高',
        'partial': '→ 判定：能走一段，但前方不是畅通通道',
        'open': '→ 判定：前方畅通',
    }[v]
    extra = f"（已忽略 {m['self_rejected']} 个车体自遮挡回波）" if m.get('self_rejected') else ''
    return f'{head} {tag}{extra}'


def read_scan(link=None, timeout: float = 4.0) -> dict:
    """抓一帧 /scan。link 可复用（tools 里已有连接时传进来，避免反复建连）。"""
    import time

    own = link is None
    if own:
        from rosbridge_link import RosbridgeLink
        link = RosbridgeLink()
    got: list = []
    link.subscribe('/scan', 'sensor_msgs/LaserScan', got.append)
    t0 = time.time()
    while time.time() - t0 < timeout and not got:
        time.sleep(0.02)
    if own:
        link.close()
    return got[-1] if got else {}
