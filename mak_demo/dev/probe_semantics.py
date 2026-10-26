"""探查 config/semantics.json：房间、物体、类别分布。

用途：第二阶段语义导航的"真值"到底有哪些可用的语义实体。
"""
import json
from collections import Counter, defaultdict
from pathlib import Path

SP = Path(__file__).resolve().parents[2] / 'config' / 'semantics.json'
d = json.loads(SP.read_text())

print('frame:', d.get('frame'))
print()
print(f'房间 {len(d["rooms"])} 个:')
for r in d['rooms']:
    print(f'  {r["label_zh"]:4s} {r["name"]:12s} center=({r["center"][0]:+.2f}, {r["center"][1]:+.2f})'
          f'  rect={r["rect"]}  door={r.get("door_center")}')

objs = d['objects']
classes = Counter(o['class'] for o in objs)
print()
print(f'物体 {len(objs)} 个，{len(classes)} 个类别:')
for c, n in classes.most_common():
    rooms = sorted({o['room'] for o in objs if o['class'] == c})
    print(f'  {c:18s} x{n}  房间: {", ".join(rooms)}')

print()
print('每个房间的物体数:')
per = defaultdict(list)
for o in objs:
    per[o['room']].append(o['class'])
for room in [r['name'] for r in d['rooms']]:
    lst = per.get(room, [])
    print(f'  {room:12s} {len(lst):2d} 个  {", ".join(lst)}')

print()
missing = [o['class'] for o in objs if not any('\u4e00' <= ch <= '\u9fff' for ch in str(o['class']))]
print(f'没有中文标签的类别: {len(set(missing))} / {len(classes)}（需要一张 中英别名 词表）')
