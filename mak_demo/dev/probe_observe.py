"""对比"贴到旁边"和"退后观察"两种站位（离线）。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from semantic_map import SemanticMap          # noqa: E402

sm = SemanticMap()
for oid in ('fridge_1', 'sofa_1', 'bed_2', 'bookshelf_2'):
    o = sm.object_by_id(oid)
    s = sm.standoff_pose(o)
    v = sm.observation_pose(o)
    rad = max(o['w'], o['d']) / 2
    print(f"{oid:14s} 半径 {rad:.2f} m")
    for tag, p in (('贴到旁边', s), ('退后观察', v)):
        if p is None:
            print(f'  {tag}：无解')
            continue
        x, y, yaw, in_room = p
        d = ((x - o['x']) ** 2 + (y - o['y']) ** 2) ** 0.5
        print(f'  {tag}：({x:+.2f}, {y:+.2f}) yaw={yaw:+.2f} '
              f'距中心 {d:.2f} m（离表面 {d - rad:.2f} m） '
              f'{"房间内" if in_room else "门口/走廊"} '
              f'地图{"空闲" if sm.is_clear(x, y) else "非空闲"}')
