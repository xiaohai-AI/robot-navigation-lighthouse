"""端到端验收 look_at：开车到观察站位 → 抓帧 → VLM 确认 → 与语义地图比对。

跑法：
    .venv/bin/python dev/test_look_at.py            # 默认 fridge_1
    .venv/bin/python dev/test_look_at.py sofa_1
    .venv/bin/python dev/test_look_at.py bookshelf_2

这是"视觉确认"这条能力的唯一验收口：它会真的开车、真的拍照、真的问模型。
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from nav_client import NavClient                  # noqa: E402
from rosbridge_link import RosbridgeLink          # noqa: E402
from semantic_map import SemanticMap              # noqa: E402
from tools import build_tools                     # noqa: E402


def main():
    oid = sys.argv[1] if len(sys.argv) > 1 else 'fridge_1'
    link = RosbridgeLink()
    try:
        nav = NavClient(link)
        if not nav.wait_ready(timeout=6.0):
            print('❌ 收不到 /agent/nav_status —— agent_bridge.py 起了吗？')
            return 2

        sm = SemanticMap()
        obj = sm.object_by_id(oid)
        if obj is None:
            print(f'❌ 没有 {oid}，可用：{", ".join(o["id"] for o in sm.objects[:5])} …')
            return 1

        fns, _log = build_tools(sm, nav)
        by = {f.__name__: f for f in fns}

        print('=' * 70)
        print(f'look_at({oid})：{sm.class_zh(obj["class"])}，'
              f'房间 {obj["room"]}，中心 ({obj["x"]:+.2f}, {obj["y"]:+.2f})')
        obs = sm.observation_pose(obj)
        if obs is None:
            print(f'❌ {oid} 找不到观察站位')
            return 1
        print(f'观察站位 ({obs[0]:+.2f}, {obs[1]:+.2f}) yaw={obs[2]:+.2f}'
              f' —— 离表面 1.40 m，相机 0.60 m 高')
        print('=' * 70)

        out = by['look_at'](oid)
        print()
        print('--- look_at 返回 ---')
        print(out)

        print()
        print('判读：')
        if '视觉确认一致' in out:
            print('  ✅ 通过：VLM 看到的东西和语义地图一致')
        elif '视觉与语义地图不符' in out:
            print('  ⚠️ 不一致：这本身是有价值的信号（物体被挪走？相机角度不对？标注错了？）')
        elif '视觉确认失败' in out:
            print('  ⚠️ 前端失败：相机或 ollama 不可用')
        else:
            print('  ⚠️ 没到达观察站位，视觉这一步没执行')
        return 0
    finally:
        link.close()


if __name__ == '__main__':
    sys.exit(main())
