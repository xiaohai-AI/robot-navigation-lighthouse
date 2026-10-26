"""离线验收意图判定 + 确定性兜底（不连 ROS，不需要 LLM）。

为什么必须有这一层：实测 qwen3-vl:8b 收到「带我去沙发看看」时只调了 find_place，
答一句"沙发在客厅正中间"就收工，车一步没动。提示词约束不住这种行为，
祈使句必须由代码保证"真的发一次车"。

跑法：.venv/bin/python dev/check_intent.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_nav import classify_intent, plan_action               # noqa: E402
from semantic_map import SemanticMap                          # noqa: E402
from tools import build_tools                                 # noqa: E402


class FakeNav:
    def __init__(self):
        self.sent = []

    def pose(self):
        return [0.0, 0.0, 0.0]

    def wait_ready(self, timeout=5.0):
        return True

    def go_to(self, x, y, yaw=0.0, timeout=180.0, verbose=True):
        self.sent.append((round(x, 3), round(y, 3)))
        return 'succeeded', {'state': 'succeeded', 'goal': [x, y],
                             'distance_remaining': 0.19, 'recoveries': 0,
                             'pose': [x, y, yaw]}


INTENT_CASES = [
    ('带我去沙发看看', 'ACT'),
    ('带我去冰箱', 'ACT'),
    ('去厨房', 'ACT'),
    ('带我去地下室', 'ACT'),
    ('洗手间在哪', 'ASK'),
    ('有哪些房间', 'ASK'),
    ('看看冰箱在哪', 'ASK'),
    ('沙发在哪个房间', 'ASK'),
]


def main():
    sm = SemanticMap()
    nav = FakeNav()
    fns, _log = build_tools(sm, nav)
    by_name = {f.__name__: f for f in fns}

    print('=' * 70)
    print('① 意图判定')
    print('=' * 70)
    ok = True
    for text, want in INTENT_CASES:
        got = classify_intent(text)
        good = got == want
        ok = ok and good
        print(f'  {"✅" if good else "❌"} {text:16s} → {got:4s}（期望 {want}）')

    print()
    print('=' * 70)
    print('② 执行计划（代码解析，不经过 LLM）')
    print('=' * 70)
    p1 = plan_action(sm, '带我去沙发看看')
    print(f'  带我去沙发看看 → {p1}')
    good1 = p1 is not None and p1[0] == 'object' and p1[1] == 'sofa_1'
    print(f'    {"✅" if good1 else "❌"} 解析为物体 sofa_1')
    ok = ok and good1

    p2 = plan_action(sm, '带我去地下室')
    print(f'  带我去地下室   → {p2}')
    good2 = p2 is None
    print(f'    {"✅" if good2 else "❌"} 解析不到 → 走 LLM 路线如实拒绝')
    ok = ok and good2

    p3 = plan_action(sm, '去厨房')
    print(f'  去厨房         → {p3}')
    good3 = p3 is not None and p3[0] == 'room' and p3[1] == 'kitchen'
    print(f'    {"✅" if good3 else "❌"} 解析为房间 kitchen')
    ok = ok and good3

    # 计划真的能发出车（用假 nav 验证 go_to 路径通）
    if p1:
        r1 = by_name['go_to'](p1[1])
        good4 = r1.startswith('✅') and len(nav.sent) == 1
        print(f'  go_to({p1[1]}) → {r1[:60]}…  {"✅ 真发车" if good4 else "❌"}')
        ok = ok and good4

    print()
    print('验收 ' + ('通过 ✅' if ok else '有失败 ❌'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
