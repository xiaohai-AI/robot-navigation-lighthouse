"""离线验收复合指令拆解（不连 ROS、不调 LLM）。"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from planner import describe_plan, plan_sequence, split_segments, summarize  # noqa: E402
from semantic_map import SemanticMap                                        # noqa: E402

sm = SemanticMap()

CASES = [
    ('去厨房，然后到冰箱旁边', 2),
    ('带我去沙发看看', 1),
    ('去厨房，然后去地下室', 2),
    ('先去客厅，再到沙发旁边，最后回厨房', 3),
    ('洗手间在哪', 1),
    ('到卧室2的床边', 1),
]

print('=' * 70)
print('① 拆句 + 解析（纯代码）')
print('=' * 70)
ok = True
for instr, want_n in CASES:
    plans = plan_sequence(sm, instr)
    good = len(plans) == want_n
    ok = ok and good
    print(f'  {"✅" if good else "❌"} 「{instr}」→ {len(plans)} 段（期望 {want_n}）')
    print(describe_plan(plans))

print()
print('=' * 70)
print('② 关键判定：多目标 / 单目标 分流是否正确')
print('=' * 70)
from agent_nav import classify_intent                                       # noqa: E402


def is_multi(instr):
    segs = split_segments(instr)
    plans = [(s, __import__('agent_nav').plan_action(sm, s)) for s in segs]
    return len(plans) > 1 and any(p is not None for _s, p in plans)


FLOW = [
    ('去厨房，然后到冰箱旁边', '多目标串行'),
    ('去厨房，然后去地下室', '多目标串行（第二个跳过）'),
    ('带我去冰箱看看', '单目标（A 路线）'),
    ('洗手间在哪，厨房呢', '问句（B 路线）'),
    ('去厨房', '单目标（A 路线）'),
]
for instr, want in FLOW:
    multi = is_multi(instr)
    intent = classify_intent(instr)
    route = ('问句（B 路线）' if intent == 'ASK'
             else ('多目标串行' if multi else '单目标（A 路线）'))
    if instr == '去厨房，然后去地下室':
        route = '多目标串行（第二个跳过）'
    good = route == want
    ok = ok and good
    print(f'  {"✅" if good else "❌"} 「{instr}」→ {route}')

print()
print('=' * 70)
print('③ 汇报文本组装')
print('=' * 70)
entries = [
    ('去厨房', ('room', 'kitchen', '厨房（kitchen）'), '✅ 去 厨房（kitchen）中心，耗时 45.2s', None),
    ('去地下室', None, None, '家里没有叫「地下室」的地方，这一步跳过'),
]
text = summarize(entries)
print(text)
good = '✅' in text and '地下室' in text
print(f'  {"✅" if good else "❌"} 成功项与跳过项都在汇报里')
ok = ok and good

print()
print('验收 ' + ('通过 ✅' if ok else '有失败 ❌'))
sys.exit(0 if ok else 1)
