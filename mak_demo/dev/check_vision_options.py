"""离线验收 vision.full_options / build_prompt / verify_object 的选项集逻辑。

要证明：
    ① full_options() 真的覆盖 33 个类别（不是静默退回 10 个小集合）
    ② 期望类别不在小集合里时（如「椅子」），full_options 也一定包含它
    ③ build_prompt 把选项拼进提示词，且行数/格式固定

跑法：.venv/bin/python dev/check_vision_options.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import vision                                     # noqa: E402
from semantic_map import SemanticMap              # noqa: E402

sm = SemanticMap()
classes = sorted({sm.class_zh(o['class']) for o in sm.objects})
full = vision.full_options()

print('=' * 72)
print(f'① 类别 {len(classes)} 种；full_options {len(full)} 个（含看不出）')
missing = [c for c in classes if c not in full]
print(f'   未覆盖：{missing or "无 ✅"}')

print('\n② 期望类别不在小集合里的情况')
for word in ('椅子', '马桶', '衣柜', '绿植'):
    o = vision.full_options(word)
    print(f'   期望「{word}」→ 在选项里：{"✅" if word in o else "❌"}  '
          f'（小集合里有吗：{"有" if word in vision.OPTIONS else "没有"}）')

print('\n③ 提示词格式')
p = vision.build_prompt(vision.full_options('冰箱'))
print(f'   长度 {len(p)} 字，行数 {len(p.splitlines())}')
print(f'   含「答案=<选项>；依据=」：{"✅" if "答案=<选项>；依据=" in p else "❌"}')
print(f'   含「看不出」：{"✅" if "看不出" in p else "❌"}')

ok = (not missing) and all(w in vision.full_options(w) for w in ('椅子', '马桶', '衣柜', '绿植'))
print()
print('验收', '通过 ✅' if ok else '未通过 ❌')
sys.exit(0 if ok else 1)
