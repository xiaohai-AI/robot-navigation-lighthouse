"""离线验收路线 C：目标不存在时的确定性拒绝 + 动作词剥离。"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_nav import room_list_text, strip_action          # noqa: E402
from semantic_map import SemanticMap                        # noqa: E402

sm = SemanticMap()

CASES = [
    ('带我去地下室', '地下室'),
    ('带我去车库看看', '车库'),
    ('导航到游泳池', '游泳池'),
    ('去太空', '太空'),
]

print(f'{"指令":16s} {"剥离后":10s} {"拒绝回答"}')
ok = True
for instr, want in CASES:
    got = strip_action(instr)
    good = got == want
    ok = ok and good
    print(f'{instr:16s} {got:10s} {"✅" if good else "❌ 期望 " + want}')

print()
text = f'家里没有叫「{strip_action("带我去地下室")}」的地方。现有的房间是：{room_list_text(sm)}。'
print('实际回答：')
print('  ' + text)
checks = [
    ('不含动作词残留', '带我去' not in text),
    ('房间数 7 个（不含走廊）', text.count('、') == 6),
    ('不含走廊', '走廊' not in text),
]
for name, good in checks:
    print(f'  {"✅" if good else "❌"} {name}')
    ok = ok and good

print()
print('验收 ' + ('通过 ✅' if ok else '有失败 ❌'))
sys.exit(0 if ok else 1)
