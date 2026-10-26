"""语义地图：把 config/semantics.json（真值）+ data/lexicon.json（中英别名）
变成"能导航"的东西。

对外只提供四个动词：
    resolve_room(text)      中文口语 -> 房间
    resolve_object(text)    中文口语 -> 候选物体（带分数和理由）
    standoff_pose(obj)      物体 -> 机器人可站的位姿（在物体旁边、不撞、在房间里）
    describe()              一句话概览（给 LLM 当前可去的地方）

为什么 standoff 要单独算：
    物体坐标是物体中心，Nav2 直接开过去会撞。必须给一个"在它旁边"的可站姿态。
    这里用"绕物体一圈采样 16 个方向 + 地图空闲校验 + 优先朝向房间中心"的办法，
    纯几何、可复现、不需要 LLM。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT_ROOT = HERE.parent
SEMANTICS = ROBOT_ROOT / 'config' / 'semantics.json'
LEXICON = HERE / 'data' / 'lexicon.json'
MAP_PGM = ROBOT_ROOT / 'config' / 'maps' / 'dan_house_explore.pgm'
MAP_YAML = ROBOT_ROOT / 'config' / 'maps' / 'dan_house_explore.yaml'

sys.path.insert(0, str(ROBOT_ROOT / 'tools'))
from map_quality import read_pgm, occupancy  # noqa: E402

# 站姿周围必须无墙的半径。用 Nav2 的 inflation_radius(0.45) 对齐：
# 只查 0.30 m 时，冰箱/餐椅之间那种"地图上可走、实际进去出不来"的缝隙会被选中
# （实测：站位 0.30 m 内干净，但车头朝里挤进去后满速指令 + 0.02 m 位移，物理卡死）。
CLEAR_M = 0.45
# 站姿离物体中心的距离：物体半径 + 0.55 m（留出机身和一小段接近距离）
GAP_M = 0.55
# 观察站位：物体半径 + 1.40 m。
# 相机在塔前 0.70 m 高（2026-10-23 车体改造），离柜面 1.41 m 时可见高度带 [-0.03, +1.43]，
# 冰箱的下把手(0.75)和门缝(1.15)同时进画面 —— 这是"看清物体"和"贴到物体"的分界线。
# （0.60 m 高时是 [-0.13, +1.33]，也够用；0.70 m 把特征推到画面中部，余量更大）
OBSERVE_GAP_M = 1.40
# 站姿到"其他家具"包围盒的最小距离。椅子腿在 2D 雷达图上稀疏，
# 只靠地图会选中"看着空、实际撞"的缝（实测冰箱站位落进椅子间，物理卡死）。
MIN_OBJ_GAP = 0.35


class SemanticMap:
    def __init__(self):
        data = json.loads(SEMANTICS.read_text())
        self.rooms = data['rooms']
        self.objects = data['objects']
        self.lex = json.loads(LEXICON.read_text())

        arr, _w, _h = read_pgm(str(MAP_PGM))
        free, occ = occupancy(arr)
        self.occ = occ
        self.free = free
        self.res, self.origin, self.shape = self._read_yaml()

    # ---------------- 地图 / 坐标 ----------------
    def _read_yaml(self):
        import yaml
        meta = yaml.safe_load(MAP_YAML.read_text())
        return float(meta['resolution']), meta['origin'], self.occ.shape

    def world_to_cell(self, x, y):
        o_x, o_y, _ = self.origin
        c = int((x - o_x) / self.res)
        r = self.shape[0] - 1 - int((y - o_y) / self.res)
        return r, c

    def is_clear(self, x, y, radius_m=CLEAR_M):
        """站姿周围 radius_m 内有没有墙（沿半径取 12 个方位，够用且快）。"""
        r0, c0 = self.world_to_cell(x, y)
        if not (0 <= r0 < self.shape[0] and 0 <= c0 < self.shape[1]):
            return False
        if not self.free[r0, c0]:
            return False
        n = int(radius_m / self.res)
        for k in range(12):
            a = 2 * math.pi * k / 12
            rr, cc = r0 + int(n * math.sin(a)), c0 + int(n * math.cos(a))
            if not (0 <= rr < self.shape[0] and 0 <= cc < self.shape[1]):
                return False
            if self.occ[rr, cc]:
                return False
        return True

    # ---------------- 房间 ----------------
    def room_by_name(self, name):
        for r in self.rooms:
            if r['name'] == name:
                return r
        return None

    def resolve_room(self, text):
        """中文/英文房间名 -> 房间 dict。先精确匹配 label_zh，再包含匹配。"""
        t = (text or '').strip()
        if not t:
            return None
        for r in self.rooms:                      # 1) 中文标签精确
            if r['label_zh'] == t:
                return r
        for r in self.rooms:                      # 2) 中文标签包含
            if r['label_zh'] in t:
                return r
        for r in self.rooms:                      # 3) 英文 name
            if r['name'].lower() in t.lower():
                return r
        # 4) 俗称
        alias = {'卧1': 'bedroom_1', '卧2': 'bedroom_2', '主卧': 'bedroom_1',
                 '次卧': 'bedroom_2', '洗手间': 'bathroom', '厕所': 'bathroom',
                 '浴室': 'bathroom', '储物间': 'storage', '储藏室': 'storage',
                 '过道': 'corridor', '厅': 'living_room'}
        for k, v in alias.items():
            if k in t:
                return self.room_by_name(v)
        return None

    # ---------------- 物体 ----------------
    def class_zh(self, cls):
        return self.lex.get(cls, {}).get('canonical', cls)

    def class_terms(self, cls):
        e = self.lex.get(cls, {})
        return [e.get('canonical', cls)] + list(e.get('aliases', []))

    def resolve_object(self, text, room=None):
        """中文口语 -> [(obj, score, reason)]，按分数降序。

        score 组成：正名命中 3 分，别名命中 2 分，英文 class 命中 1 分；
        若给了 room 且物体在该房间，+1.5 分。
        """
        t = (text or '').strip()
        if not t:
            return []
        out = []
        for o in self.objects:
            cls = o['class']
            score, why = 0.0, ''
            if self.class_zh(cls) and self.class_zh(cls) in t:
                score, why = 3.0, f'正名「{self.class_zh(cls)}」'
            else:
                for a in self.lex.get(cls, {}).get('aliases', []):
                    if a in t:
                        score, why = 2.0, f'别名「{a}」'
                        break
            if not score and cls.replace('_', ' ') in t.lower():
                score, why = 1.0, f'英文 class「{cls}」'
            if not score:
                continue
            if room and o['room'] == room:
                score += 1.5
                why += ' + 在该房间'
            out.append((o, score, why))
        out.sort(key=lambda z: -z[1])
        return out

    def objects_in(self, room_name):
        return [o for o in self.objects if o['room'] == room_name]

    def object_by_id(self, oid):
        for o in self.objects:
            if o['id'] == oid:
                return o
        return None

    # ---------------- 可站姿态 ----------------
    def obj_clearance(self, x, y, skip_id=None):
        """到"其他所有物体"包围盒的最小距离（米）。

        为什么必须查这个：椅子腿在 0.132 m 高的 2D 雷达图上很稀疏，
        地图看着是空的，车开进去却撞上实体 —— 实测冰箱站位落在椅子缝里，
        满速指令 + 0.02 m 位移 = 物理卡死。语义地图里有真值包围盒，直接用来避。
        """
        best = 99.0
        for o in self.objects:
            if o['id'] == skip_id:
                continue
            hw = float(o.get('w', 0.4)) / 2.0
            hd = float(o.get('d', 0.4)) / 2.0
            # 忽略 yaw 的轴对齐盒，偏保守（旋转过的物体会被算得更大一点）
            dx = max(abs(x - float(o['x'])) - hw, 0.0)
            dy = max(abs(y - float(o['y'])) - hd, 0.0)
            best = min(best, math.hypot(dx, dy))
        return best

    def standoff_pose(self, obj, room=None, gap_m=GAP_M):
        """绕物体采样 16 个方向，返回第一个"地图空闲 + 离其他家具够远 + 在房间内"的位姿。

        yaw 指向物体（这样到点后车头朝物体，符合"到 X 旁边看看"的直觉）。
        gap_m 控制退后多少：GAP_M（0.55，贴到旁边）或 OBSERVE_GAP_M（1.40，看得清）。
        """
        x, y = float(obj['x']), float(obj['y'])
        rad = max(float(obj.get('w', 0.4)), float(obj.get('d', 0.4))) / 2.0
        dist = rad + gap_m
        room = room or self.room_by_name(obj['room'])

        # 优先朝向房间中心（通常就是房间里最空旷的方向）
        pref = math.atan2(room['center'][1] - y, room['center'][0] - x) if room else 0.0
        angles = sorted((2 * math.pi * k / 16 for k in range(16)),
                        key=lambda a: abs((a - pref + math.pi) % (2 * math.pi) - math.pi))

        # 逐级放宽：先要求最严（0.45 净空 + 0.35 离家具），过不了就降一档，
        # 保证贴墙的小物体（垃圾桶、边桌）也能给出位姿，而不是直接失败。
        tiers = [(CLEAR_M, MIN_OBJ_GAP), (0.40, 0.25), (0.32, 0.15), (0.30, 0.0)]
        for clear_m, obj_gap in tiers:
            fallback = None
            for a in angles:
                px, py = x + dist * math.cos(a), y + dist * math.sin(a)
                if not self.is_clear(px, py, radius_m=clear_m):
                    continue
                if obj_gap > 0 and self.obj_clearance(px, py, skip_id=obj['id']) < obj_gap:
                    continue
                in_room = True
                if room and room.get('rect'):
                    x1, y1, x2, y2 = room['rect']
                    in_room = (x1 <= px <= x2) and (y1 <= py <= y2)
                pose = (round(px, 3), round(py, 3),
                        round(math.atan2(y - py, x - px), 3), in_room)
                if in_room:
                    self.last_tier = (clear_m, obj_gap)
                    return pose
                if fallback is None:
                    fallback = pose
            if fallback is not None:
                self.last_tier = (clear_m, obj_gap)
                return fallback
        self.last_tier = None
        return None

    def observation_pose(self, obj, room=None):
        """观察站位：比站姿再退 0.85 m，让相机的可见高度带覆盖门缝和把手。

        相机 0.70 m 高时，离柜面 1.41 m 的可见带是 [-0.03, +1.43]，
        冰箱下把手(0.75) 与门缝(1.15) 同时在画面内 —— 这是 VLM 能认出物体的前提。
        用途区分：贴到物体旁边用 standoff_pose，要"看清它是什么"用这个。
        """
        return self.standoff_pose(obj, room=room, gap_m=OBSERVE_GAP_M)

    # ---------------- 概览 ----------------
    def describe(self):
        lines = [f'共 {len(self.rooms)} 个房间、{len(self.objects)} 个物体：']
        for r in self.rooms:
            n = len(self.objects_in(r['name']))
            lines.append(f'  {r["label_zh"]}（{r["name"]}，{n} 个物体）中心 '
                         f'({r["center"][0]:+.2f}, {r["center"][1]:+.2f})')
        return '\n'.join(lines)


if __name__ == '__main__':
    sm = SemanticMap()
    print(sm.describe())
