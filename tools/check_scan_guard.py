#!/usr/bin/python3
"""tools/scan_pub_guard.py 的离线单测（不需要 ROS、不需要仿真）。

覆盖 judge() 的四种结局：
    0 个发布者            → 失败（投影没起来）
    1 个且名字对          → 通过
    2 个（含 gz 桥）      → 失败（双发布，最糟）
    1 个但名字不对        → 失败（桥接配错）

跑法：/usr/bin/python3 tools/check_scan_guard.py   （退出码 0 = 全过）
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scan_pub_guard import judge            # noqa: E402

CASES = [
    ('0 个发布者',
     [], 'pointcloud_to_laserscan', False),
    ('1 个且名字对（3d 投影节点）',
     [{'node_name': 'pointcloud_to_laserscan', 'node_namespace': '/'}],
     'pointcloud_to_laserscan', True),
    ('1 个且名字对（2d gz 桥）',
     [{'node_name': 'gz_bridge', 'node_namespace': '/'}],
     'gz_bridge', True),
    ('2 个 = 双发布（z 桥 + 投影器）',
     [{'node_name': 'gz_bridge', 'node_namespace': '/'},
      {'node_name': 'pointcloud_to_laserscan', 'node_namespace': '/'}],
     'pointcloud_to_laserscan', False),
    ('1 个但名字不对（桥接配错路数）',
     [{'node_name': 'gz_bridge', 'node_namespace': '/'}],
     'pointcloud_to_laserscan', False),
    ('带命名空间也认',
     [{'node_name': 'pointcloud_to_laserscan', 'node_namespace': '/robot1'}],
     'pointcloud_to_laserscan', True),
]


def main() -> int:
    print('scan 发布者断言 —— 离线单测')
    bad = 0
    for label, pubs, expect, want in CASES:
        ok, msg = judge(pubs, expect)
        good = ok == want
        bad += 0 if good else 1
        want_txt = '通过' if want else '失败'
        extra = '' if good else f'（期望 {want_txt}）'
        print(f'  {"✅" if good else "❌"} {label:<32s} → {"通过" if ok else "失败"}{extra}')
        if not good:
            print(f'      实际信息：{msg}')
    print()
    if bad:
        print(f'❌ {bad}/{len(CASES)} 条不符')
        return 1
    print(f'scan 守卫验收 通过 ✅（{len(CASES)}/{len(CASES)}）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
