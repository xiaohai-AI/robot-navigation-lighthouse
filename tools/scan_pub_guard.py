#!/usr/bin/python3
"""启动断言：/scan 只能有一个发布者。

为什么需要它（不是洁癖）：
    3d 模式下 gz 会**同时**发出 LaserScan（gz 自带 /scan）和 PointCloudPacked。
    我们的 launch 只桥 /scan/points，再由 pointcloud_to_laserscan 投影出 /scan。
    但只要某次改 launch 时不小心把 '/scan@...LaserScan' 也桥进来，就会出现两个发布者：
        · slam_toolbox 用哪一路取决于到达顺序 → 建图时好时坏、重影、清图；
        · Nav2 的障碍层同理；
    而现象是"地图脏了/车乱飘"，不会指向"多了一路 /scan"。排查一次的代价远大于这条断言。
    这段以前只有一行注释，现在变成可执行的检查。

判据（纯函数 judge，便于离线单测）：
    0 个发布者                     → 失败：投影节点/桥接没起来
    ≥2 个发布者                    → 立刻失败：双发布，这是最糟的一种
    1 个但节点名不含 --expect      → 超时后失败：发布者不是预期的那个节点

跑法：
    /usr/bin/python3 tools/scan_pub_guard.py                       # 3d：预期 pointcloud_to_laserscan
    /usr/bin/python3 tools/scan_pub_guard.py --expect gz_bridge     # 2d：预期 gz 桥
    /usr/bin/python3 tools/check_scan_guard.py                     # 离线单测（不需要 ROS）
退出码：0 通过，2 失败（launch 里会打红字，但不会拖垮其它节点）
"""
from __future__ import annotations

import argparse
import sys
import time

TOPIC = '/scan'


def _full(p: dict) -> str:
    """统一成一个斜杠：'/' + name，'/robot1' + name。"""
    ns = (p['node_namespace'] or '').rstrip('/')
    return f"{ns}/{p['node_name']}"


def judge(pubs: list[dict], expect: str) -> tuple[bool, str]:
    """pubs: [{'node_name': str, 'node_namespace': str}, ...]

    返回 (是否通过, 说明)。纯函数 —— 不碰 ROS，离线可测。
    """
    n = len(pubs)
    names = '、'.join(_full(p) for p in pubs)
    if n == 0:
        return False, (f'{TOPIC} 没有任何发布者 —— 投影节点或 gz 桥没起来？'
                       '（3d 模式应有 pointcloud_to_laserscan 在发 /scan）')
    if n > 1:
        return False, (f'❌ {TOPIC} 有 {n} 个发布者：{names}\n'
                       '     这就是双发布：3d 模式绝不能把 gz 自带的 /scan 桥进来。\n'
                       '     后果是 slam_toolbox 用哪一路取决于到达顺序 —— 地图重影/时好时坏/突然清空。')
    p = pubs[0]
    if expect and expect not in p['node_name']:
        return False, (f'{TOPIC} 的发布者是 {_full(p)}，'
                       f'预期应该是 {expect} —— 是不是桥接配错了路数？')
    return True, f'{TOPIC} 发布者 1 个：{_full(p)} ✅ 无双发布'


def _endpoints(node):
    out = []
    for info in node.get_publishers_info_by_topic(TOPIC):
        out.append({'node_name': info.node_name,
                    'node_namespace': info.node_namespace})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description='/scan 发布者数量断言')
    ap.add_argument('--expect', default='pointcloud_to_laserscan',
                    help='预期的发布者节点名（子串匹配；空字符串＝只查数量）')
    ap.add_argument('--timeout', type=float, default=30.0,
                    help='启动宽限期：这段时间内"0 个发布者"不下结论（秒）')
    ap.add_argument('--settle', type=float, default=2.0,
                    help='看到正确结果后，再稳定观察这么久才算通过（秒）')
    ap.add_argument('--watch', action='store_true',
                    help='常驻模式：通过后继续每 --interval 秒复查（抓后来才起的野发布者）')
    ap.add_argument('--interval', type=float, default=30.0, help='--watch 的复查间隔（秒）')
    a = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
    except Exception as e:                                     # noqa: BLE001
        print(f'[scan_guard] 需要系统 python 的 rclpy（{e}）—— 别在项目 venv 里跑')
        return 2

    rclpy.init()
    node = Node('scan_pub_guard')
    t_start = time.time()
    ok_since = None
    last = None                     # 上一次对外给出的结论（None = 还没给过）
    code = 2
    try:
        while True:
            pubs = _endpoints(node)
            ok, msg = judge(pubs, a.expect)

            # 启动宽限期：桥接和投影节点是并行起来的，"暂时还没发布者"不是故障。
            # 但"≥2 个发布者"是确定性的故障，任何时候都立刻算数。
            in_grace = (time.time() - t_start) < a.timeout
            if not ok and not pubs and in_grace:
                ok, msg = None, msg

            if ok is not None and ok != last:
                print(f'[scan_guard] {"✅ 通过" if ok else "❌ 失败"}：{msg}', flush=True)
                if not ok:
                    print('[scan_guard] 继续跑只会产出不可信的地图/定位；'
                          '先修桥接或投影再启动。', flush=True)
                last = ok

            if not a.watch:
                if ok is True:
                    ok_since = ok_since or time.time()
                    if time.time() - ok_since >= a.settle:
                        code = 0
                        break
                else:
                    ok_since = None
                    if ok is False:            # 已下结论的失败，不必等满宽限期
                        code = 2
                        break
                    if not in_grace:           # 宽限期过了还没有发布者
                        print(f'[scan_guard] ❌ 失败：{msg}', flush=True)
                        code = 2
                        break
                time.sleep(0.5)
            else:
                time.sleep(a.interval)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return code


if __name__ == '__main__':
    sys.exit(main())
