"""带仪表的目标执行诊断：发一个语义目标，同时记录 plan / cmd_vel / odom / 局部代价地图。

用途：车"被接受但不动"时，一次性判断到底是
    ① 控制器没出力（cmd_vel 恒 0）        → DWB 找不到合法轨迹（空间太窄/目标不可达）
    ② 控制器有出力但车不动（odom 不变）    → 物理卡住 / 打滑
    ③ 路径本身有问题（起点段反向/绕行）    → 全局规划器的问题
    ④ 局部代价地图把车围住                 → 膨胀/感知问题

跑法：
    .venv/bin/python dev/diag_goto.py 沙发
    .venv/bin/python dev/diag_goto.py 3.0 0.0        # 也可以直接给坐标

输出：终端摘要 + logs/diag_<时间>.json（完整采样，可回看）
"""
import base64
import json
import math
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from nav_client import NavClient              # noqa: E402
from rosbridge_link import RosbridgeLink       # noqa: E402
from semantic_map import SemanticMap           # noqa: E402

SAMPLES = 200          # 200 × 0.2 s = 40 s


def decode_int8(data):
    """rosbridge 对 int8[] 的编码随版本不同：可能是 base64 字符串、int 列表、
    或 {'type':'Buffer','data':[...]}。三种都接住。"""
    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        return decode_int8(data.get('data', []))
    raw = base64.b64decode(data)
    return list(struct.unpack(f'{len(raw)}b', raw))


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 1

    # ---- 目标解析：支持"语义词"或"x y"两种给法 ----
    sm = SemanticMap()
    if len(args) >= 2:
        gx, gy, gyaw = float(args[0]), float(args[1]), 0.0
        label = f'({gx:+.2f}, {gy:+.2f})'
    else:
        text = args[0]
        objs = sm.resolve_object(text)
        if not objs:
            print(f'❌ 语义解析不到「{text}」')
            return 1
        obj = objs[0][0]
        pose = sm.standoff_pose(obj)
        if pose is None:
            print(f'❌ {obj["id"]} 找不到可站位姿')
            return 1
        gx, gy, gyaw, _inside = pose
        label = f'{obj["id"]}({sm.class_zh(obj["class"])}) → 站姿 ({gx:+.2f}, {gy:+.2f})'

    link = RosbridgeLink()
    nav = NavClient(link)
    if not nav.wait_ready():
        print('❌ 收不到 /agent/nav_status')
        return 1

    rec = {'goal': [gx, gy, gyaw], 'label': label,
           't0': time.time(), 'samples': [], 'plan': None, 'costmap': None}

    def on_plan(m):
        rec['plan'] = m

    def on_cost(m):
        rec['costmap'] = m

    cmd = {'vx': 0.0, 'wz': 0.0, 't': 0.0}

    def on_cmd(m):
        cmd['vx'] = m['linear']['x']
        cmd['wz'] = m['angular']['z']
        cmd['t'] = time.time()

    link.subscribe('/plan', 'nav_msgs/Path', on_plan)
    link.subscribe('/local_costmap/costmap', 'nav_msgs/OccupancyGrid', on_cost)
    link.subscribe('/cmd_vel', 'geometry_msgs/Twist', on_cmd)

    start_pose = nav.pose()
    print('=' * 70)
    print(f'目标: {label}')
    print(f'起点: {start_pose}')
    print(f'采样 {SAMPLES * 0.2:.0f} s …')
    print('=' * 70)

    link.publish('/agent/nav_goal', 'geometry_msgs/PoseStamped', {
        'header': {'frame_id': 'map'},
        'pose': {'position': {'x': gx, 'y': gy, 'z': 0.0},
                 'orientation': {'x': 0.0, 'y': 0.0,
                                 'z': math.sin(gyaw / 2), 'w': math.cos(gyaw / 2)}}})
    t_pub = time.time()

    for i in range(SAMPLES):
        time.sleep(0.2)
        st = dict(nav.status or {})
        rec['samples'].append({
            't': round(time.time() - t_pub, 2),
            'state': st.get('state'),
            'dist': st.get('distance_remaining'),
            'recov': st.get('recoveries'),
            'pose': st.get('pose'),
            'vx': round(cmd['vx'], 3),
            'wz': round(cmd['wz'], 3),
        })
        if st.get('state') in ('succeeded', 'failed', 'canceled', 'stalled', 'rejected'):
            # 终态后再多采 2 s，看车是否还在滑行
            for _ in range(10):
                time.sleep(0.2)
                s2 = dict(nav.status or {})
                rec['samples'].append({'t': round(time.time() - t_pub, 2),
                                       'state': s2.get('state'),
                                       'dist': s2.get('distance_remaining'),
                                       'recov': s2.get('recoveries'),
                                       'pose': s2.get('pose')})
            break

    link.close()

    # ---------------- 摘要 ----------------
    print()
    print('---- 状态轨迹 ----')
    seen, last = [], None
    for s in rec['samples']:
        if s['state'] != last:
            print(f"  t={s['t']:5.1f}s  {s['state']:9s} 剩余 {s['dist']} 恢复 {s['recov']}")
            seen.append(s['state'])
            last = s['state']

    poses = [s['pose'] for s in rec['samples'] if s['pose']]
    moved = 0.0
    if len(poses) >= 2:
        moved = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])
    dists = [s['dist'] for s in rec['samples'] if s['dist'] is not None]
    print()
    print(f"位移 {moved:.2f} m   剩余距离 {dists[0] if dists else '-'} → {dists[-1] if dists else '-'}")

    # ---- 控制指令统计：区分"控制器不出力"和"出力但车不动" ----
    vxs = [abs(s.get('vx') or 0.0) for s in rec['samples']]
    wzs = [abs(s.get('wz') or 0.0) for s in rec['samples']]
    active = [s for s in rec['samples'] if s.get('state') == 'active']
    a_vx = [abs(s.get('vx') or 0.0) for s in active]
    a_wz = [abs(s.get('wz') or 0.0) for s in active]
    if a_vx:
        nz = sum(1 for v in a_vx if v > 0.01) + sum(1 for w in a_wz if w > 0.05)
        print(f"cmd_vel（active 期间 {len(active)} 拍）: "
              f"|vx| 最大 {max(a_vx):.3f} m/s，|wz| 最大 {max(a_wz):.3f} rad/s，"
              f"非零拍数 {nz}")
        if max(a_vx) < 0.01 and max(a_wz) < 0.05:
            print('  → 判定：控制器基本没出力（DWB 找不到合法轨迹，或路径退化）')
        elif moved < 0.15:
            print('  → 判定：有速度指令但车几乎不动（物理卡住/打滑，或需要倒车脱困）')
        else:
            print('  → 判定：指令与位移一致，属正常行驶')

    if rec['plan']:
        ps = rec['plan'].get('poses', [])
        pts = [(p['pose']['position']['x'], p['pose']['position']['y']) for p in ps]
        plen = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                   for i in range(len(pts) - 1))
        print(f"路径 {len(pts)} 个点，长度 {plen:.2f} m")
        if len(pts) >= 2 and poses:
            hx, hy, hyaw = poses[0]
            seg = math.atan2(pts[1][1] - hy, pts[1][0] - hx)
            print(f"首段方向 {math.degrees(seg):+.1f}°，车头 {math.degrees(hyaw):+.1f}°，"
                  f"夹角 {math.degrees(abs(seg - hyaw)):.1f}°")
        print('  路径前 5 点:', [(round(x, 2), round(y, 2)) for x, y in pts[:5]])
        print('  路径末 3 点:', [(round(x, 2), round(y, 2)) for x, y in pts[-3:]])
    else:
        print('路径: 没收到 /plan')

    if rec['costmap']:
        cm = rec['costmap']
        info = cm['info']
        data = decode_int8(cm['data'])
        w, h = info['width'], info['height']
        res = info['resolution']
        ox, oy = info['origin']['position']['x'], info['origin']['position']['y']
        if poses:
            px, py, _ = poses[-1]
            c = int((px - ox) / res)
            r = h - 1 - int((py - oy) / res)
            if 0 <= r < h and 0 <= c < w:
                print(f"局部代价地图 {w}x{h} @ {res} m，车所在格代价 {data[r * w + c]}")
                lethal = 0
                n = int(0.8 / res)
                for dr in range(-n, n + 1):
                    for dc in range(-n, n + 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and data[rr * w + cc] >= 99:
                            lethal += 1
                print(f"  车周围 0.8 m 内致命格 {lethal} 个（>40 基本可判定被围死）")

    out = HERE / 'logs' / f"diag_{time.strftime('%H%M%S')}.json"
    out.parent.mkdir(exist_ok=True)
    slim = {k: v for k, v in rec.items() if k not in ('costmap',)}
    out.write_text(json.dumps(slim, ensure_ascii=False, indent=1))
    print()
    print(f'完整采样已存 {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
