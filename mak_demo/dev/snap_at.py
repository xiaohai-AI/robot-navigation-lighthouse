"""开到指定位姿 → 抓一帧 → 问 VLM。用来验证"距离是否足够让物体可识别"。

这是"改相机几何"之前的零风险实验：不动机器人模型、不重启仿真，
只让车停在更远的位置看同一个物体。

跑法：
    # 冰箱正东 1.8 m，朝西看冰箱（推荐先试这个）
    .venv/bin/python dev/snap_at.py -0.86 5.20 180

    # 用现有站姿（0.94 m）做对照
    .venv/bin/python dev/snap_at.py -1.72 5.20 180

    # 自定义问题
    .venv/bin/python dev/snap_at.py -0.86 5.20 180 "画面里的白色高柜是什么？"

判读：如果 1.8 m 能答出"冰箱"而 0.94 m 不能，说明纯距离问题；
      两处都不能，才需要改 FOV / 相机高度。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from check_vlm import ask_vlm, grab_frame                  # noqa: E402
from nav_client import NavClient                           # noqa: E402
from rosbridge_link import RosbridgeLink                   # noqa: E402


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    x, y, yaw_deg = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    question = (sys.argv[4] if len(sys.argv) > 4 else
                '这是家用机器人的前置相机画面。请回答：1) 画面里最近的物体是什么？'
                '2) 能不能判断它是一台冰箱？说明依据（看到哪些特征）。')

    link = RosbridgeLink()
    try:
        nav = NavClient(link)
        if not nav.wait_ready(timeout=6.0):
            print('❌ 收不到 /agent/nav_status')
            return 2
        yaw = math.radians(yaw_deg)
        print(f'开到 ({x:+.2f}, {y:+.2f}) yaw={yaw_deg:+.0f}° …')
        state, st = nav.go_to(x, y, yaw, timeout=180.0, verbose=True)
        print(f'结果 {state}，终点 {st.get("pose")}，剩余 {st.get("distance_remaining")} m')
        if state != 'succeeded':
            print('⚠️ 没到指定位置，下面的画面可能不是你想要的视角')
    finally:
        link.close()

    import time
    time.sleep(1.0)                       # 让渲染跟上（相机 10 Hz，等两帧）
    png, path, size = grab_frame()
    print('=' * 70)
    print(f'抓帧 {path.name}  {size[0]}x{size[1]}  PNG {len(png) / 1024:.1f} KB')
    print('=' * 70)
    answer, el = ask_vlm(png, question, 'qwen3-vl:8b')
    print(answer.strip())
    print()
    print(f'VLM 耗时 {el:.1f}s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
