"""验收 semantic_map：61 个物体是否每个都能给出"可站、在房间里、看得见物体"的位姿。

这是第二阶段的第一条硬指标：
    如果某个物体的 standoff 位姿不存在，LLM 再怎么解析也到不了 —— 必须提前暴露。

跑法：cd mak_demo && .venv/bin/python dev/check_standoffs.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from semantic_map import SemanticMap  # noqa: E402

sm = SemanticMap()
ok = inroom = 0
fails = []

print(f'{"物体":22s} {"类别(中)":10s} {"房间":12s} {"位姿(x,y)":22s} {"yaw":7s} 判定')
print('-' * 96)
for o in sm.objects:
    pose = sm.standoff_pose(o)
    if pose is None:
        fails.append((o, 'no pose'))
        verdict = '❌ 找不到可站位姿'
        print(f'{o["id"]:22s} {sm.class_zh(o["class"]):10s} {o["room"]:12s} '
              f'{"-":22s} {"-":7s} {verdict}')
        continue
    x, y, yaw, inside = pose
    ok += 1
    inroom += int(inside)
    if not inside:
        fails.append((o, 'not in room'))
    verdict = '✅ 可站' + ('' if inside else '（在房间外）')
    print(f'{o["id"]:22s} {sm.class_zh(o["class"]):10s} {o["room"]:12s} '
          f'({x:+.2f}, {y:+.2f}):10s      {yaw:+.2f}  {verdict}')

print('-' * 96)
total = len(sm.objects)
print(f'可站位姿 {ok}/{total}   其中在所属房间内 {inroom}/{total}')
if fails:
    print('有问题：')
    for o, why in fails:
        print(f'  {o["id"]:22s} {why}')

print()
print('--- 口语解析自测（不需要 LLM）---')
cases = [
    ('带我去厨房', None),
    ('到沙发旁边', None),
    ('去卧室2的床边', None),
    ('卫生间在哪', None),
    ('看看冰箱', None),
    ('书房的书架', 'study'),
    ('洗手间', None),
]
for text, room in cases:
    r = sm.resolve_room(text)
    objs = sm.resolve_object(text, room=r['name'] if r else room)
    best = objs[0] if objs else None
    tag = f'房间={r["label_zh"]}' if r else '房间=None'
    if best:
        o, sc, why = best
        tag += f' | 物体={o["id"]}({sm.class_zh(o["class"])}) 分数{sc:.1f} [{why}]'
    else:
        tag += ' | 物体=None'
    print(f'  {text:14s} -> {tag}')
