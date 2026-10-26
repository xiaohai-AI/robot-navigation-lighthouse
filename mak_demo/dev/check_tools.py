"""离线验收 tools.py：用一个假的 nav 顶替真实链路，检查工具输出格式。

为什么先离线跑：工具返回的文本是 LLM 唯一的输入来源，
格式一乱它就乱调（或干脆不调）。这里不连 ROS，秒级验证。

跑法：.venv/bin/python dev/check_tools.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from semantic_map import SemanticMap
from tools import build_tools


class FakeNav:
    """假的导航：不连 ROS，按预设返回，用来检查工具输出格式。"""

    def __init__(self):
        self.sent = []

    def pose(self):
        return [0.0, 0.0, 0.0]

    def wait_ready(self, timeout=5.0):
        return True

    def go_to(self, x, y, yaw=0.0, timeout=180.0, verbose=True):
        self.sent.append((round(x, 3), round(y, 3), round(yaw, 3)))
        return 'succeeded', {'state': 'succeeded', 'goal': [x, y],
                             'distance_remaining': 0.18, 'recoveries': 0,
                             'pose': [x, y, yaw]}


def main():
    sm = SemanticMap()
    nav = FakeNav()
    fns, log = build_tools(sm, nav)
    by_name = {f.__name__: f for f in fns}

    print('=' * 70)
    print('① list_rooms()  ' + '-' * 50)
    print(by_name['list_rooms']())

    print()
    print('=' * 70)
    print('② find_place("冰箱")  ' + '-' * 42)
    print(by_name['find_place']('冰箱'))

    print()
    print('=' * 70)
    print('③ find_place("床", room="卧室2")  ' + '-' * 30)
    print(by_name['find_place']('床', room='卧室2'))

    print()
    print('=' * 70)
    print('④ find_place("洗手间")  ' + '-' * 42)
    print(by_name['find_place']('洗手间'))

    print()
    print('=' * 70)
    print('⑤ find_place("地下室")  ← 不存在的东西，必须老实说没有  ' + '-' * 12)
    print(by_name['find_place']('地下室'))

    print()
    print('=' * 70)
    print('⑥ go_to 假 id（LLM 幻觉时会这样）  ' + '-' * 28)
    print(by_name['go_to']('fridge_99'))

    print()
    print('=' * 70)
    print('⑦ go_to 真 id  ' + '-' * 50)
    print(by_name['go_to']('fridge_1'))

    print()
    print('=' * 70)
    print('⑧ go_to_room("客厅")  ' + '-' * 44)
    print(by_name['go_to_room']('客厅'))

    print()
    print('=' * 70)
    print(f'工具流水共 {len(log)} 条，实际发车 {len(nav.sent)} 次：{nav.sent}')

    # ---- 断言（按工具名+参数找流水，不依赖顺序下标）----
    fake = next(c for c in log if c['tool'] == 'go_to' and 'fridge_99' in str(c['args']))
    under = next(c for c in log if c['tool'] == 'find_place'
                 and c['args'].get('text') == '地下室')
    bath = next(c for c in log if c['tool'] == 'find_place'
                and c['args'].get('text') == '洗手间')
    ok = True
    checks = [
        ('假 id 被拒且不发车', '❌' in fake['ret'] and len(nav.sent) >= 1
         and nav.sent[0][1] > 4.0),
        ('真 id 出车（冰箱站姿 y≈5.2）', abs(nav.sent[0][1] - 5.2) < 0.01),
        ('地下室无候选', '物体匹配：无' in under['ret']),
        ('洗手间识别为房间', 'bathroom' in bath['ret']),
    ]
    for name, good in checks:
        print(f'  {"✅" if good else "❌"} {name}')
        ok = ok and good
    print('tools.py 验收 ' + ('通过 ✅' if ok else '有失败 ❌'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
