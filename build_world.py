#!/usr/bin/env python3
"""
参数化生成 dan_house.world ——  7 房间 / 18 x 14 m 公寓 (Gazebo Fortress, gz-sim 6)

运行:
    python3 build_world.py

设计原则（本版重点：把环境"特征尺寸"整体放大）
------------------------------------------------
机器人是 Lighthouse（灯塔）：底盘尺寸沿用 TurtleBot3 Waffle（外接圆半径约 0.23 m），
把所有"特征尺寸"放大，让机器人有充分余量通过：

    房子          16 x 12  ->  18 x 14 m
    走廊净宽      2.08     ->  2.88 m
    门洞          1.0      ->  1.3 m（卫生间 1.2 m）
    房间净尺寸    最小 2.58 x 5.33 m
    家具摆放      散在房间内        ->  一律贴墙，房间中央留空

生成后自动做几何自检（verify）：
  · 巡游路线每一段抽样检查到所有家具/墙的净距是否 >= 0.40 m
  · 每个门洞的进出通道是否完全无遮挡
不通过会直接报错并指出位置 —— 不用再靠仿真试错。

输出:
    worlds/dan_house.world     世界文件（--lidar 2d，单线雷达，默认）
    worlds/dan_house_3d.world  世界文件（--lidar 3d，64 线雷达）
    config/route.json          巡游路线（tour_node.py 读这个，单一数据源）
    config/semantics.json      语义标注（第二阶段语义导航的真值）

两个世界的房间/墙/家具/路线完全相同，唯一差别是 include 哪台机器人：
    2d → models/lighthouse/     （单线，1 条扫描线）
    3d → models/lighthouse_3d/  （64 线，点云）
"""
import argparse
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

_ap = argparse.ArgumentParser(description='生成 dan_house 世界（可选单线 / 64 线机器人）')
_ap.add_argument('--lidar', choices=('2d', '3d'), default='2d',
                 help='2d = models/lighthouse（单线，默认，行为与以前完全一致）；'
                      '3d = models/lighthouse_3d（64 线，写 dan_house_3d.world）')
ARGS = _ap.parse_args()
ROBOT = 'lighthouse' if ARGS.lidar == '2d' else 'lighthouse_3d'
WORLD_NAME = 'dan_house.world' if ARGS.lidar == '2d' else 'dan_house_3d.world'

OUT = os.path.join(HERE, 'worlds', WORLD_NAME)

# ---------- 低矮家具的高度策略（两版不同）----------
# 单线版（2d）：把低于雷达面 0.78 m 的家具抬到 0.86 m，否则它们既不在图上也不在代价地图里。
#             这套世界已经建好地图，**冻结不动**。
# 64 线版（3d）：用真实高度。投影高度带 [-0.75, +0.05] m 相对 base_scan(0.78)
#             = 绝对 0.03 ~ 0.83 m，本来就覆盖茶几(0.40)、浴缸(0.56)、垃圾桶(0.60)……
#             这才是"换传感器"而不是"把家具长高"。
RAISE_LOW = (ARGS.lidar == '2d')


def hh(real_height):
    """低矮家具高度：单线版抬到 0.86（雷达面 0.78 + 8 cm 余量），64 线版保留真实值。"""
    return 0.86 if RAISE_LOW else real_height


ROUTE_JSON = os.path.join(HERE, 'config', 'route.json')
SEM_JSON = os.path.join(HERE, 'config', 'semantics.json')
TEX = 'textures/'

# ============ 全局尺寸（放大后的特征尺寸） ============
XW, XE = -9.0, 9.0         # 外墙中心线 x  -> 18 m
YS, YN = -7.0, 7.0         # 外墙中心线 y  -> 14 m
T_OUT, T_IN = 0.22, 0.12   # 外墙 / 内墙厚度
H = 2.7                    # 层高
COR = 1.5                  # 走廊两侧墙中心线 y = ±1.5 -> 净宽 2.88 m
DOOR_W = 1.3               # 标准门宽（原 1.0）
DOOR_W_BATH = 1.2          # 卫生间门
DOOR_H = 2.1
CEILING = False

X = []
def add(s): X.append(s)

# 碰撞体足迹表（几何自检用） + 语义标注表（第二阶段语义导航用）
FOOT = []      # (x, y, half_x, half_y, name)
SEM = []       # {'id','class','room','x','y','yaw','w','d'}
_ids = {}
ROOM_LABELS = {
    'living': 'living_room', 'kitchen': 'kitchen', 'study': 'study',
    'bed1': 'bedroom_1', 'bed2': 'bedroom_2', 'bath': 'bathroom', 'store': 'storage',
}

def room_of(x, y):
    """根据坐标反推物体属于哪个房间（贴墙摆放会略微超出房间矩形，给 0.3 m 容差）"""
    for nm, x0, x1, y0, y1, _ in ROOMS:
        if x0 - 0.30 <= x <= x1 + 0.30 and y0 - 0.30 <= y <= y1 + 0.30:
            return ROOM_LABELS.get(nm, nm)
    return 'corridor'

def foot(cx, cy, w, d, rz, name, cls=None):
    if abs(abs(rz) - math.pi / 2) < 0.2:   # ±90° 时交换宽深
        w, d = d, w
    FOOT.append((cx, cy, w / 2.0, d / 2.0, name))
    if not name.startswith('wall_') and name != 'floor':
        _ids[name] = _ids.get(name, 0) + 1
        SEM.append({'id': f'{name}_{_ids[name]}', 'class': cls or name,
                    'room': room_of(cx, cy),
                    'x': round(cx, 3), 'y': round(cy, 3), 'yaw': round(rz, 3),
                    'w': round(w, 3), 'd': round(d, 3)})

# ============ 几何 / 材质小工具 ============
def mat_c(rgb, emissive=None):
    e = f'<emissive>{emissive}</emissive>' if emissive else ''
    return f'<material><diffuse>{rgb} 1</diffuse>{e}</material>'

def mat_t(tex, rough=0.85):
    return (f'<material><diffuse>1 1 1 1</diffuse><pbr><metal>'
            f'<albedo_map>{TEX}{tex}</albedo_map>'
            f'<metalness>0.0</metalness><roughness>{rough}</roughness></metal></pbr></material>')

def box(s):  return f'<box><size>{s}</size></box>'
def cyl(r, l): return f'<cylinder><radius>{r}</radius><length>{l}</length></cylinder>'
def sph(r):  return f'<sphere><radius>{r}</radius></sphere>'

def P(p):
    v = p.split()
    return ' '.join(v + ['0', '0', '0']) if len(v) == 3 else ' '.join(v)

def vis(n, p, g, m): return f'<visual name="{n}"><pose>{P(p)}</pose><geometry>{g}</geometry>{m}</visual>'
def col(n, p, g):    return f'<collision name="{n}"><pose>{P(p)}</pose><geometry>{g}</geometry></collision>'

_SEEN = {}
def model(n, pose, parts):
    i = _SEEN.get(n, 0) + 1
    _SEEN[n] = i
    uniq = n if i == 1 else f'{n}_{i}'
    return (f'<model name="{uniq}"><pose>{P(pose)}</pose><static>true</static>'
            f'<link name="link">{"".join(parts)}</link></model>')

WOOD  = mat_t('wood_floor.jpg'); WOODD = mat_t('wood_dark.jpg')
WALL  = mat_t('wallpaper.jpg');  TILE  = mat_t('tile.jpg')
CARP  = mat_t('carpet.jpg');     FAB   = mat_t('fabric.jpg')
MET   = mat_t('metal.jpg')
BED   = mat_t('bedding.jpg');    RUG   = mat_t('rug.jpg')
PAINT = mat_t('painting.jpg');   LEAF  = mat_t('leaf.jpg')
TERRA = mat_t('terracotta.jpg')
WHITE = mat_c('0.95 0.95 0.95'); DARKW = mat_c('0.2 0.15 0.1')
FRAME = mat_c('0.36 0.26 0.18')

# ============ 房间表（净尺寸，不含墙） ============
ROOMS = [
    ('living',   -8.89, -3.20,   1.56,  6.89,  WOOD),
    ('kitchen',  -3.08,   1.60,   1.56,  6.89,  TILE),
    ('study',     1.72,   8.89,   1.56,  6.89,  WOODD),
    ('bed1',     -8.89,  -4.00,  -6.89, -1.56,  CARP),
    ('bed2',     -3.88,   0.90,  -6.89, -1.56,  CARP),
    ('bath',      1.02,   3.60,  -6.89, -1.56,  TILE),
    ('store',     3.72,   8.89,  -6.89, -1.56,  WOODD),
]

# ============ 头部 ============
add('''<?xml version="1.0" ?>
<sdf version="1.9">
<world name="dan_house">
<physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
<scene><ambient>0.32 0.32 0.32 1</ambient></scene>
<light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction></light>''')

for name, x0, x1, y0, y1, _ in ROOMS:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    warm = '1.0 0.94 0.85' if name in ('living', 'bed1', 'bed2') else '1.0 1.0 1.0'
    add(f'<light type="point" name="light_{name}"><cast_shadows>false</cast_shadows>'
        f'<pose>{cx} {cy} {H - 0.15} 0 0 0</pose><diffuse>{warm} 1</diffuse>'
        f'<attenuation><range>15</range><constant>0.4</constant><linear>0.3</linear>'
        f'<quadratic>0.02</quadratic></attenuation></light>')
for i, lx in enumerate([-7.5, -3.5, 0.0, 3.5, 7.5]):
    add(f'<light type="point" name="light_corridor_{i}"><cast_shadows>false</cast_shadows>'
        f'<pose>{lx} 0 {H - 0.15} 0 0 0</pose><diffuse>1.0 0.98 0.92 1</diffuse>'
        f'<attenuation><range>11</range><constant>0.5</constant><linear>0.35</linear>'
        f'<quadratic>0.04</quadratic></attenuation></light>')

# 房子外面的大地面（视觉 + 兜底碰撞，在室内地板下方 12 mm）
add(model('ground_plane', '0 0 0', [
    col('c', '0 0 0', '<plane><normal>0 0 1</normal><size>90 90</size></plane>'),
    vis('v', '0 0 0', '<plane><normal>0 0 1</normal><size>90 90</size></plane>', mat_c('0.35 0.35 0.36'))]))

# ============ 地板：整块连续，绝不留缝 ============
# 教训：以前按房间一块块铺地板，墙厚度带里会留下 6~12 cm 的空槽，
#       门洞处墙是空的 → 轮子（直径 66 mm）掉进槽里卡死，odom 一路跑飞。
# 现在：一整块地板（带碰撞）铺满整个室内空间；房间材质只作为 1 mm 厚的
#       "视觉贴片"叠在上面（无碰撞、无缝隙、不影响通行）。
add(model('floor_main', '0 0 0', [
    col('c', '0 0 0.006', box(f'{XE-XW-T_OUT:.3f} {YN-YS-T_OUT:.3f} 0.012')),
    vis('v', '0 0 0.006', box(f'{XE-XW-T_OUT:.3f} {YN-YS-T_OUT:.3f} 0.012'), WOOD)]))

for name, x0, x1, y0, y1, tex in ROOMS:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    # 纯视觉贴片：厚 1 mm，从地板顶面 0.012 起，向内缩 2 cm 避免和墙贴片重叠
    g = box(f'{x1-x0-0.04:.3f} {y1-y0-0.04:.3f} 0.001')
    add(model(f'floor_tex_{name}', '0 0 0', [
        vis('v', f'{cx} {cy} 0.0125', g, tex)]))

# ============ 墙体 ============
def _gaps(a, b, doors):
    segs, cur = [], a
    for cx, w in sorted(doors):
        s, e = max(cx - w / 2, a), min(cx + w / 2, b)
        if s > cur + 1e-6:
            segs.append((cur, s))
        cur = max(cur, e)
    if cur < b - 1e-6:
        segs.append((cur, b))
    return segs

def door_frame(name, cx, cy, w, vertical):
    h, fw, fd = DOOR_H + 0.05, 0.07, T_IN + 0.04
    if vertical:
        for i, dy in enumerate((-1, 1)):
            add(model(f'{name}_p{i}', f'{cx} {cy + dy*(w/2 - fw/2)} {h/2}',
                      [vis('v', '0 0 0', box(f'{fd:.3f} {fw} {h:.2f}'), FRAME)]))
        add(model(f'{name}_top', f'{cx} {cy} {h - fw/2}',
                  [vis('v', '0 0 0', box(f'{fd:.3f} {w} {fw}'), FRAME)]))
    else:
        for i, dx in enumerate((-1, 1)):
            add(model(f'{name}_p{i}', f'{cx + dx*(w/2 - fw/2)} {cy} {h/2}',
                      [vis('v', '0 0 0', box(f'{fw} {fd:.3f} {h:.2f}'), FRAME)]))
        add(model(f'{name}_top', f'{cx} {cy} {h - fw/2}',
                  [vis('v', '0 0 0', box(f'{w} {fd:.3f} {fw}'), FRAME)]))
    lintel_h = H - DOOR_H
    if vertical:
        add(model(f'{name}_lintel', f'{cx} {cy} {DOOR_H + lintel_h/2}',
                  [vis('v', '0 0 0', box(f'{T_IN} {w} {lintel_h:.3f}'), WALL)]))
    else:
        add(model(f'{name}_lintel', f'{cx} {cy} {DOOR_H + lintel_h/2}',
                  [vis('v', '0 0 0', box(f'{w} {T_IN} {lintel_h:.3f}'), WALL)]))

def hwall(name, y, x0, x1, t, doors=()):
    for i, (a, b) in enumerate(_gaps(x0, x1, doors)):
        add(model(f'{name}_{i}', f'{(a+b)/2:.3f} {y} {H/2}',
                  [col('c', '0 0 0', box(f'{b-a:.3f} {t} {H}')),
                   vis('v', '0 0 0', box(f'{b-a:.3f} {t} {H}'), WALL)]))
        foot((a + b) / 2, y, b - a, t, 0.0, f'wall_{name}_{i}')
    for cx, w in doors:
        door_frame(f'{name}_door_{cx}', cx, y, w, vertical=False)

def vwall(name, x, y0, y1, t, doors=()):
    for i, (a, b) in enumerate(_gaps(y0, y1, doors)):
        add(model(f'{name}_{i}', f'{x} {(a+b)/2:.3f} {H/2}',
                  [col('c', '0 0 0', box(f'{t} {b-a:.3f} {H}')),
                   vis('v', '0 0 0', box(f'{t} {b-a:.3f} {H}'), WALL)]))
        foot(x, (a + b) / 2, t, b - a, 0.0, f'wall_{name}_{i}')
    for cy, w in doors:
        door_frame(f'{name}_door_{cy}', x, cy, w, vertical=True)

# ---- 门的位置（集中定义，路线也引用这些值）----
D_LIVING  = (-6.0, DOOR_W)
D_KITCHEN = (-0.8, DOOR_W)
D_STUDY   = (5.2,  DOOR_W)
D_BED1    = (-6.3, DOOR_W)
D_BED2    = (-1.5, DOOR_W)
D_BATH    = (2.3,  DOOR_W_BATH)
D_STORE   = (6.2,  DOOR_W)
D_INNER   = (3.8,  DOOR_W)     # 厨房<->书房 内门：在 x=1.66 的墙上，门洞沿 y

hwall('wall_south', YS, XW, XE, T_OUT)
hwall('wall_north', YN, XW, XE, T_OUT)
vwall('wall_west', XW, YS, YN, T_OUT)
vwall('wall_east', XE, YS, YN, T_OUT)

hwall('wall_corr_north', +COR, XW, XE, T_IN, doors=[D_LIVING, D_KITCHEN, D_STUDY])
hwall('wall_corr_south', -COR, XW, XE, T_IN, doors=[D_BED1, D_BED2, D_BATH, D_STORE])

vwall('wall_living_kitchen', -3.14, +COR, YN, T_IN)
vwall('wall_kitchen_study',  +1.66, +COR, YN, T_IN, doors=[D_INNER])
vwall('wall_bed1_bed2',      -3.94, YS, -COR, T_IN)
vwall('wall_bed2_bath',      +0.96, YS, -COR, T_IN)
vwall('wall_bath_store',     +3.66, YS, -COR, T_IN)

# ============ 窗 & 挂画（纯装饰，无碰撞） ============
def window(name, x, y, w=1.8):
    add(model(name, f'{x} {y} 1.6', [
        vis('pane', '0 0 0', box(f'{w*0.9:.2f} 0.06 1.2'), mat_c('0.55 0.72 0.9', '0.14 0.2 0.3')),
        vis('ft', '0 0 0.65', box(f'{w:.2f} 0.1 0.08'), WHITE),
        vis('fb', '0 0 -0.65', box(f'{w:.2f} 0.1 0.08'), WHITE),
        vis('fl', f'{-w/2:.2f} 0 0', box('0.1 0.1 1.2'), WHITE),
        vis('fr', f'{w/2:.2f} 0 0', box('0.1 0.1 1.2'), WHITE)]))

for i, wx in enumerate([-7.4, -4.4, -1.2, 3.4, 7.0]):
    window(f'window_n{i+1}', wx, YN - 0.05)
for i, wx in enumerate([-7.4, -5.0, -2.0, 1.5, 7.0]):
    window(f'window_s{i+1}', wx, YS + 0.05)

def painting(name, x, y, rz):
    add(model(name, f'{x} {y} 1.6 0 0 {rz}', [
        vis('frame', '0 0 0', box('0.02 0.8 0.65'), mat_c('0.15 0.12 0.1')),
        vis('canvas', '0.025 0 0', box('0.03 0.72 0.57'), PAINT)]))
painting('painting_1', XW + 0.11, 2.6, 0)
painting('painting_2', 4.0, -COR + 0.06, 3.1416)

# ================================================================
#                     家 具（一律贴墙 + 登记足迹）
# ================================================================
def sofa(cx, cy, rz=0.0):
    add(model('sofa', f'{cx} {cy} 0 0 0 {rz}', [
        col('seat_c', '0 0 0.21', box('2.0 0.85 0.42')), col('back_c', '0 0.535 0.55', box('2.0 0.22 0.6')),
        vis('base', '0 -0.05 0.16', box('2.0 0.9 0.32'), FAB),
        vis('cl', '-0.49 -0.05 0.40', box('0.92 0.8 0.16'), FAB),
        vis('cr', '0.49 -0.05 0.40', box('0.92 0.8 0.16'), FAB),
        vis('back', '0 0.52 0.55', box('2.0 0.24 0.7'), FAB),
        vis('al', '-1.12 -0.05 0.28', box('0.24 0.9 0.56'), FAB),
        vis('ar', '1.12 -0.05 0.28', box('0.24 0.9 0.56'), FAB),
        vis('l1', '-1.05 -0.45 0.035', cyl(0.03, 0.07), DARKW), vis('l2', '1.05 -0.45 0.035', cyl(0.03, 0.07), DARKW),
        vis('l3', '-1.05 0.3 0.035', cyl(0.03, 0.07), DARKW), vis('l4', '1.05 0.3 0.035', cyl(0.03, 0.07), DARKW)]))
    foot(cx, cy, 2.5, 1.0, rz, 'sofa')

def coffee_table(cx, cy, rz=0.0, h=0.40):
    # h = 桌面高度。雷达在 0.78 m 平面，低于这个面就扫不到 → 摆放时传 h>=0.85
    leg = max(0.05, h - 0.05)
    add(model('coffee_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', f'0 0 {h/2:.3f}', box(f'1.0 0.6 {h:.3f}')),
        vis('top', f'0 0 {h-0.025:.3f}', box('1.0 0.6 0.05'), WOODD),
        vis('l1', f'-0.42 -0.24 {leg/2:.3f}', cyl(0.025, leg), DARKW), vis('l2', f'0.42 -0.24 {leg/2:.3f}', cyl(0.025, leg), DARKW),
        vis('l3', f'-0.42 0.24 {leg/2:.3f}', cyl(0.025, leg), DARKW), vis('l4', f'0.42 0.24 {leg/2:.3f}', cyl(0.025, leg), DARKW)]))
    foot(cx, cy, 1.0, 0.6, rz, 'coffee_table')

def tv_set(cx, cy, rz=0.0):
    p = [col('c', '0 0 0.25', box('0.45 1.8 0.5')), vis('body', '0 0 0.25', box('0.45 1.8 0.5'), WOODD),
         vis('d1', '0.235 -0.45 0.25', box('0.02 0.86 0.44'), mat_c('0.25 0.18 0.12')),
         vis('d2', '0.235 0.45 0.25', box('0.02 0.86 0.44'), mat_c('0.25 0.18 0.12')),
         col('tv_c', '-0.18 0 0.95', box('0.07 1.5 0.9')),
         vis('tvframe', '-0.18 0 0.95', box('0.07 1.5 0.9'), mat_c('0.05 0.05 0.05')),
         vis('screen', '-0.135 0 0.95', box('0.02 1.36 0.76'),
             '<material><diffuse>1 1 1 1</diffuse><emissive>0.15 0.17 0.22 1</emissive><pbr><metal>'
             '<albedo_map>textures/screen.jpg</albedo_map><metalness>0.0</metalness><roughness>0.3</roughness></metal></pbr></material>'),
         vis('neck', '-0.18 0 0.44', box('0.06 0.25 0.12'), mat_c('0.05 0.05 0.05'))]
    add(model('tv_set', f'{cx} {cy} 0 0 0 {rz}', p))
    foot(cx, cy, 0.5, 1.85, rz, 'tv_set')

def bookshelf(cx, cy, rz=0.0, name='bookshelf'):
    bs = [col('c', '0 0 0.9', box('1.4 0.35 1.8')),
          vis('sl', '-0.68 0 0.9', box('0.04 0.35 1.8'), WOODD), vis('sr', '0.68 0 0.9', box('0.04 0.35 1.8'), WOODD),
          vis('top', '0 0 1.78', box('1.4 0.35 0.04'), WOODD), vis('bot', '0 0 0.02', box('1.4 0.35 0.04'), WOODD),
          vis('back', '0 -0.155 0.9', box('1.4 0.04 1.72'), mat_c('0.3 0.22 0.15')),
          vis('s1', '0 0.02 0.45', box('1.32 0.3 0.03'), mat_c('0.4 0.28 0.18')),
          vis('s2', '0 0.02 0.9', box('1.32 0.3 0.03'), mat_c('0.4 0.28 0.18')),
          vis('s3', '0 0.02 1.35', box('1.32 0.3 0.03'), mat_c('0.4 0.28 0.18'))]
    for i, bc in enumerate(['0.65 0.25 0.2', '0.2 0.35 0.6', '0.25 0.5 0.3', '0.8 0.7 0.2', '0.5 0.3 0.55']):
        bs.append(vis(f'b{i+1}', f'{-0.5+i*0.12} 0 0.585', box('0.07 0.24 0.24'), mat_c(bc)))
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', bs))
    foot(cx, cy, 1.4, 0.36, rz, name)

def floor_lamp(cx, cy):
    add(model('floor_lamp', f'{cx} {cy} 0', [
        col('c', '0 0 0.7', cyl(0.16, 1.4)), vis('base', '0 0 0.01', cyl(0.15, 0.02), mat_c('0.2 0.2 0.2')),
        vis('pole', '0 0 0.66', cyl(0.02, 1.28), mat_c('0.25 0.25 0.25')),
        vis('shade', '0 0 1.4', cyl(0.16, 0.26), mat_c('0.95 0.9 0.75', '0.6 0.5 0.32'))]))
    foot(cx, cy, 0.34, 0.34, 0, 'floor_lamp')

def side_table(cx, cy, rz=0.0, h=0.60):
    # h = 柜体高度（雷达 0.78 m 平面，低于就扫不到）
    add(model('side_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', f'0 0 {h/2:.3f}', box(f'0.6 0.6 {h:.3f}')), vis('body', f'0 0 {h/2:.3f}', box(f'0.6 0.6 {h:.3f}'), WOODD),
        vis('mug', f'0.1 -0.1 {h+0.045:.3f}', cyl(0.04, 0.09), WHITE)]))
    foot(cx, cy, 0.6, 0.6, rz, 'side_table')

def plant(cx, cy):
    add(model('plant', f'{cx} {cy} 0', [
        col('pot_c', '0 0 0.175', cyl(0.2, 0.35)), col('leaf_c', '0 0 0.8', cyl(0.38, 0.9)),
        vis('pot', '0 0 0.175', cyl(0.2, 0.35), TERRA), vis('soil', '0 0 0.36', cyl(0.18, 0.02), mat_c('0.2 0.14 0.1')),
        vis('trunk', '0 0 0.63', cyl(0.03, 0.55), mat_c('0.35 0.25 0.15')),
        vis('lf1', '0 0 1.05', sph(0.32), LEAF), vis('lf2', '0.18 0.1 0.9', sph(0.24), LEAF),
        vis('lf3', '-0.15 -0.1 0.95', sph(0.22), LEAF)]))
    foot(cx, cy, 0.78, 0.78, 0, 'plant')

def rug(cx, cy, w, h, rz=0.0):
    add(model('rug', f'{cx} {cy} 0.0125 0 0 {rz}', [vis('v', '0 0 0', box(f'{w} {h} 0.008'), RUG)]))

def kitchen_counter(cx, cy, rz=0.0):
    kc = [col('c', '0 0 0.45', box('2.6 0.65 0.9')), vis('cab', '0 0 0.425', box('2.6 0.65 0.85'), mat_c('0.92 0.92 0.9')),
          vis('top', '0 0 0.88', box('2.72 0.74 0.06'), mat_c('0.32 0.32 0.35')),
          vis('sink', '-0.6 -0.15 0.912', box('0.45 0.35 0.02'), mat_c('0.18 0.18 0.2')),
          vis('faucet', '-0.6 -0.28 1.0', cyl(0.015, 0.22), mat_c('0.75 0.75 0.78')),
          vis('stove', '0.6 0 0.912', box('0.55 0.45 0.02'), mat_c('0.08 0.08 0.08'))]
    add(model('kitchen_counter', f'{cx} {cy} 0 0 0 {rz}', kc))
    foot(cx, cy, 2.72, 0.74, rz, 'kitchen_counter')

def counter_side(cx, cy, rz=0.0):
    add(model('counter_side', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.45', box('0.65 1.6 0.9')), vis('cab', '0 0 0.425', box('0.65 1.6 0.85'), mat_c('0.92 0.92 0.9')),
        vis('top', '0 0 0.88', box('0.74 1.7 0.06'), mat_c('0.32 0.32 0.35'))]))
    foot(cx, cy, 0.74, 1.7, rz, 'counter_side')

def fridge(cx, cy, rz=0.0):
    add(model('fridge', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.9', box('0.75 0.75 1.8')), vis('body', '0 0 0.9', box('0.75 0.75 1.8'), MET),
        vis('split', '0 -0.38 1.15', box('0.72 0.02 0.03'), mat_c('0.4 0.4 0.42')),
        vis('h1', '0.28 -0.4 0.75', box('0.03 0.03 0.5'), mat_c('0.3 0.3 0.32')),
        vis('h2', '0.28 -0.4 1.45', box('0.03 0.03 0.4'), mat_c('0.3 0.3 0.32'))]))
    foot(cx, cy, 0.78, 0.78, rz, 'fridge')

def dining_table(cx, cy, h=0.75):
    post = max(0.08, h - 0.08)          # 从基座顶 0.03 撑到桌面底 h-0.05
    add(model('dining_table', f'{cx} {cy} 0', [
        col('top_c', f'0 0 {h-0.025:.3f}', cyl(0.55, 0.05)), col('post_c', f'0 0 {0.03+post/2:.3f}', cyl(0.06, post)),
        vis('top', f'0 0 {h-0.025:.3f}', cyl(0.55, 0.05), WOODD), vis('post', f'0 0 {0.03+post/2:.3f}', cyl(0.06, post), mat_c('0.25 0.18 0.12')),
        vis('base', '0 0 0.015', cyl(0.28, 0.03), mat_c('0.25 0.18 0.12'))]))
    foot(cx, cy, 1.1, 1.1, 0, 'dining_table')

def chair(cx, cy, rz=0.0, name='chair'):
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.475', box('0.45 0.45 0.95')), vis('seat', '0 0 0.45', box('0.45 0.45 0.04'), WOODD),
        vis('back', '0 -0.2 0.75', box('0.45 0.05 0.55'), WOODD)]))
    foot(cx, cy, 0.46, 0.46, rz, name)

def desk(cx, cy, rz=0.0, name='desk'):
    p = [col('c', '0 0 0.38', box('1.4 0.7 0.76')),
         vis('top', '0 0 0.75', box('1.45 0.72 0.05'), WOODD),
         vis('leg1', '-0.66 -0.3 0.36', box('0.12 0.12 0.72'), DARKW),
         vis('leg2', '0.66 -0.3 0.36', box('0.12 0.12 0.72'), DARKW),
         vis('leg3', '-0.66 0.3 0.36', box('0.12 0.12 0.72'), DARKW),
         vis('leg4', '0.66 0.3 0.36', box('0.12 0.12 0.72'), DARKW),
         vis('monitor', '0 0.28 1.03', box('0.06 0.62 0.38'), mat_c('0.08 0.08 0.09')),
         vis('mstand', '0 0.28 0.83', box('0.06 0.06 0.12'), mat_c('0.3 0.3 0.32')),
         vis('keyb', '0 -0.05 0.785', box('0.42 0.14 0.02'), mat_c('0.15 0.15 0.17'))]
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', p))
    foot(cx, cy, 1.45, 0.72, rz, name)

def bed(cx, cy, rz=0.0):
    add(model('bed', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.225', box('1.5 2.0 0.45')), vis('frame', '0 0 0.11', box('1.6 2.1 0.22'), WOODD),
        vis('mattress', '0 0 0.33', box('1.5 2.0 0.22'), BED), vis('head', '0 -1.0 0.6', box('1.6 0.1 0.9'), WOODD),
        vis('p1', '-0.4 -0.75 0.48', box('0.55 0.35 0.09'), WHITE),
        vis('p2', '0.4 -0.75 0.48', box('0.55 0.35 0.09'), WHITE)]))
    foot(cx, cy, 1.6, 2.1, rz, 'bed')

def nightstand(cx, cy, rz=0.0, h=0.50):
    # h = 柜体高度（灯座、灯杆、灯罩都跟着往上挪）。雷达 0.78 m 平面，柜体要够高。
    add(model('nightstand', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', f'0 0 {h/2:.3f}', box(f'0.45 0.45 {h:.3f}')), vis('body', f'0 0 {h/2:.3f}', box(f'0.45 0.45 {h:.3f}'), WOODD),
        vis('drawer', f'0.235 0 {h-0.15:.3f}', box('0.02 0.4 0.18'), mat_c('0.3 0.22 0.15')),
        vis('lb', f'0 0 {h+0.01:.3f}', cyl(0.06, 0.02), mat_c('0.3 0.3 0.3')),
        vis('lp', f'0 0 {h+0.11:.3f}', cyl(0.015, 0.18), mat_c('0.3 0.3 0.3')),
        vis('ls', f'0 0 {h+0.25:.3f}', cyl(0.09, 0.12), mat_c('0.95 0.85 0.6', '0.55 0.45 0.28'))]))
    foot(cx, cy, 0.48, 0.48, rz, 'nightstand')

def wardrobe(cx, cy, rz=0.0):
    add(model('wardrobe', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.95', box('1.0 0.6 1.9')), vis('body', '0 0 0.95', box('1.0 0.6 1.9'), WOODD),
        vis('d1', '-0.245 -0.31 0.95', box('0.48 0.02 1.8'), mat_c('0.42 0.32 0.22')),
        vis('d2', '0.245 -0.31 0.95', box('0.48 0.02 1.8'), mat_c('0.45 0.35 0.25')),
        vis('h1', '-0.05 -0.33 1.0', box('0.03 0.03 0.12'), mat_c('0.7 0.7 0.72')),
        vis('h2', '0.05 -0.33 1.0', box('0.03 0.03 0.12'), mat_c('0.7 0.7 0.72'))]))
    foot(cx, cy, 1.0, 0.62, rz, 'wardrobe')

def toilet(cx, cy, rz=0.0, h=0.78):
    # h = 水箱顶面高度。水箱顶面必须高过雷达平面（0.78），否则马桶整件看不见。
    tank_h = max(0.2, h - 0.4)
    add(model('toilet', f'{cx} {cy} 0 0 0 {rz}', [
        col('bowl_c', '0 0.12 0.2', box('0.4 0.6 0.4')),
        vis('base', '0 0.12 0.2', box('0.36 0.55 0.38'), WHITE),
        vis('seat', '0 0.12 0.4', cyl(0.2, 0.06), mat_c('0.92 0.92 0.94')),
        col('tank_c', f'0 0.44 {0.4+tank_h/2:.3f}', box(f'0.42 0.2 {tank_h:.3f}')),
        vis('tank', f'0 0.44 {0.4+tank_h/2:.3f}', box(f'0.42 0.2 {tank_h:.3f}'), WHITE)]))
    foot(cx, cy, 0.44, 0.66, rz, 'toilet')

def sink_basin(cx, cy, rz=0.0):
    add(model('sink_basin', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.42', box('0.62 0.46 0.84')),
        vis('cab', '0 0 0.4', box('0.6 0.44 0.78'), mat_c('0.9 0.9 0.9')),
        vis('top', '0 0 0.83', box('0.66 0.5 0.06'), mat_c('0.95 0.95 0.97')),
        vis('basin', '0 -0.05 0.86', cyl(0.17, 0.04), mat_c('0.85 0.87 0.9')),
        vis('tap', '0 0.16 0.95', cyl(0.015, 0.18), mat_c('0.75 0.75 0.78')),
        vis('mirror', '0 0.22 1.5', box('0.62 0.02 0.7'), mat_c('0.7 0.8 0.85', '0.2 0.22 0.24'))]))
    foot(cx, cy, 0.66, 0.5, rz, 'sink_basin')

def bathtub(cx, cy, rz=0.0, w=0.9, l=2.2, h=0.56):
    add(model('bathtub', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', f'0 0 {h/2:.3f}', box(f'{w} {l} {h:.3f}')),
        vis('body', f'0 0 {h/2:.3f}', box(f'{w} {l} {h:.3f}'), mat_c('0.97 0.97 0.98')),
        vis('inner', f'0 0 {h-0.06:.3f}', box(f'{w-0.16:.2f} {l-0.3:.2f} 0.06'), mat_c('0.82 0.86 0.9')),
        vis('tap', f'0 {l/2-0.2:.2f} {h+0.06:.3f}', cyl(0.02, 0.14), mat_c('0.75 0.75 0.78'))]))
    foot(cx, cy, w, l, rz, 'bathtub')

def washer(cx, cy, rz=0.0):
    add(model('washer', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.42', box('0.6 0.6 0.84')),
        vis('body', '0 0 0.42', box('0.6 0.6 0.84'), mat_c('0.93 0.93 0.95')),
        vis('door', '0 -0.31 0.45', cyl(0.2, 0.04), mat_c('0.55 0.62 0.68')),
        vis('panel', '0 -0.31 0.78', box('0.5 0.03 0.1'), mat_c('0.2 0.2 0.22'))]))
    foot(cx, cy, 0.62, 0.62, rz, 'washer')

def shelf_unit(cx, cy, rz=0.0, w=1.2, d=0.5, h=1.9, name='shelf_unit'):
    p = [col('c', f'0 0 {h/2}', box(f'{w} {d} {h}')),
         vis('back', f'0 {-d/2+0.02:.3f} {h/2}', box(f'{w} 0.04 {h}'), MET)]
    for side, sx in (('l', -w/2 + 0.03), ('r', w/2 - 0.03)):
        p.append(vis(f's{side}', f'{sx:.3f} 0 {h/2}', box('0.06 0.5 ' + f'{h}'), MET))
    for i in range(1, 4):
        p.append(vis(f'shelf{i}', f'0 0 {i*h/4:.3f}', box(f'{w-0.1:.2f} {d-0.06:.2f} 0.04'), mat_c('0.55 0.56 0.58')))
    for i, bc in enumerate(['0.55 0.4 0.25', '0.3 0.35 0.5', '0.6 0.55 0.3']):
        p.append(vis(f'box{i+1}', f'{-0.3+0.3*i:.2f} 0 {0.55+i*0.47:.2f}', box('0.34 0.3 0.28'), mat_c(bc)))
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', p))
    foot(cx, cy, w, d, rz, name)

def crate(cx, cy, s=0.7, h=0.7, col_rgb='0.55 0.4 0.22'):
    add(model('crate', f'{cx} {cy} 0', [
        col('c', f'0 0 {h/2}', box(f'{s} {s} {h}')),
        vis('v', f'0 0 {h/2}', box(f'{s} {s} {h}'), mat_c(col_rgb)),
        vis('band', f'0 0 {h*0.75:.2f}', box(f'{s+0.01} {s+0.01} 0.06'), mat_c('0.3 0.3 0.32'))]))
    foot(cx, cy, s, s, 0, 'crate')

def pillar(cx, cy, s=0.5):
    add(model('pillar', f'{cx} {cy} 0', [
        col('c', f'0 0 {H/2}', box(f'{s} {s} {H}')),
        vis('v', f'0 0 {H/2}', box(f'{s} {s} {H}'), mat_c('0.9 0.88 0.84'))]))
    foot(cx, cy, s, s, 0, 'pillar')

def console_table(cx, cy, rz=0.0):
    add(model('console_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.4', box('1.0 0.32 0.8')),
        vis('top', '0 0 0.78', box('1.05 0.34 0.05'), WOODD),
        vis('leg1', '-0.45 -0.12 0.38', box('0.08 0.08 0.76'), DARKW),
        vis('leg2', '0.45 -0.12 0.38', box('0.08 0.08 0.76'), DARKW),
        vis('vase', '0.3 0 0.88', cyl(0.07, 0.16), TERRA)]))
    foot(cx, cy, 1.05, 0.34, rz, 'console_table')

def cart(cx, cy, rz=0.0):
    add(model('cart', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.45', box('0.55 0.4 0.9')),
        vis('body', '0 0 0.45', box('0.55 0.4 0.9'), mat_c('0.75 0.78 0.8')),
        vis('tray', '0 0 0.92', box('0.58 0.43 0.03'), mat_c('0.35 0.37 0.4'))]))
    foot(cx, cy, 0.58, 0.43, rz, 'cart')

# ---- 语义锚点：颜色鲜明、形状简单，方便 VLM / 视觉分割定位 ----
def trash_bin(cx, cy, rgb, name='trash_bin', h=0.60):
    add(model(name, f'{cx} {cy} 0', [
        col('c', f'0 0 {h/2:.3f}', cyl(0.19, h)),
        vis('body', f'0 0 {h/2:.3f}', cyl(0.19, h), mat_c(rgb)),
        vis('rim', f'0 0 {h+0.01:.3f}', cyl(0.2, 0.04), mat_c('0.15 0.15 0.15'))]))
    foot(cx, cy, 0.4, 0.4, 0, name, cls='trash_bin')

def fire_extinguisher(cx, cy, h=0.52):
    add(model('fire_extinguisher', f'{cx} {cy} 0', [
        col('c', f'0 0 {h/2:.3f}', cyl(0.12, h)),
        vis('body', f'0 0 {h/2:.3f}', cyl(0.12, h), mat_c('0.85 0.1 0.1')),
        vis('neck', f'0 0 {h+0.04:.3f}', cyl(0.04, 0.1), mat_c('0.2 0.2 0.2')),
        vis('handle', f'0.06 0 {h+0.08:.3f}', box('0.14 0.03 0.03'), mat_c('0.15 0.15 0.15'))]))
    foot(cx, cy, 0.26, 0.26, 0, 'fire_extinguisher', cls='fire_extinguisher')

# ================================================================
#              按房间摆放（一律贴墙，中央留空）
# ================================================================
# 客厅 x[-8.89,-3.20] y[1.56,6.89]，门 x=-6.0
# 说明：低矮家具统一走 hh(真实高度)：
#       单线版 → 0.86（雷达面 0.78 + 8 cm，否则扫不到）
#       64 线版 → 真实高度（投影高度带覆盖 0.03~0.83 m，本来就看得见）
sofa(-8.40, 4.30, rz=1.5708)
coffee_table(-7.35, 4.30, rz=1.5708, h=hh(0.40))
rug(-6.60, 4.20, 2.6, 2.0)
tv_set(-6.00, 6.55, rz=-1.5708)
bookshelf(-3.55, 3.00, rz=1.5708)
plant(-8.45, 6.40)
side_table(-8.50, 2.00, h=hh(0.60))
floor_lamp(-3.75, 6.20)

# 厨房 x[-3.08,1.60] y[1.56,6.89]，门 x=-0.8
kitchen_counter(-1.60, 6.50)
fridge(-2.66, 5.20, rz=1.5708)
counter_side(1.22, 5.60)              # 贴东墙（避开内门通道 y 3.15~4.45）
dining_table(-2.20, 3.40, h=hh(0.75))
chair(-2.20, 2.70)
chair(-2.20, 4.10)

# 书房 x[1.72,8.89] y[1.56,6.89]，门 x=5.2，内门 y=3.8
bookshelf(2.80, 6.71)
bookshelf(4.40, 6.71)
bookshelf(6.00, 6.71)
desk(8.50, 4.60, rz=-1.5708)
desk(8.50, 2.90, rz=-1.5708)
chair(7.85, 4.60, rz=1.5708)
chair(7.85, 2.90, rz=1.5708)
pillar(7.50, 5.60)
shelf_unit(2.50, 2.60, name='rack')
plant(8.45, 1.95)

# 卧室1 x[-8.89,-4.00] y[-6.89,-1.56]，门 x=-6.3
bed(-8.15, -3.90, rz=-1.5708)
nightstand(-8.45, -2.55, h=hh(0.50))
wardrobe(-5.60, -6.55)
rug(-6.40, -3.90, 2.2, 1.6)
plant(-4.40, -1.95)

# 卧室2 x[-3.88,0.90] y[-6.89,-1.56]，门 x=-1.5
bed(-1.50, -5.85)
nightstand(-2.65, -6.50, h=hh(0.50))
wardrobe(0.45, -5.90, rz=1.5708)
desk(-3.50, -3.00, rz=1.5708)
rug(-1.50, -3.30, 1.8, 1.4)

# 卫生间 x[1.02,3.60] y[-6.89,-1.56]，门 x=2.3
bathtub(3.15, -4.80, h=hh(0.56))       # 长边沿 y（rz=0），靠东墙；转 90° 会横穿房间
toilet(1.45, -6.40, rz=3.1416, h=hh(0.78))
sink_basin(1.35, -5.20, rz=1.5708)
washer(1.30, -2.60)

# 储藏间 x[3.72,8.89] y[-6.89,-1.56]，门 x=6.2
shelf_unit(4.40, -6.55)
shelf_unit(5.70, -6.55)
shelf_unit(7.00, -6.55)
shelf_unit(8.55, -3.40, rz=1.5708, name='shelf_east')
crate(4.20, -3.00, 0.9, 0.85)
crate(7.60, -2.60, 0.7, hh(0.70), '0.35 0.45 0.3')
crate(5.00, -2.10, 0.6, hh(0.60), '0.4 0.4 0.42')

# 走廊 y[-1.44,1.44]（净宽 2.88 m）
console_table(-4.70, 1.24)            # 贴北墙（避开客厅门洞 x -6.55~-5.45）
cart(3.90, -1.20, rz=3.1416)          # 贴南墙（避开卫生间门洞 x 1.2~2.6）
plant(8.30, 1.05)

# ---- 语义锚点（第二阶段语义导航的目标物：颜色鲜明、形状简单）----
trash_bin(-2.85, 2.00, '0.85 0.12 0.12', 'trash_bin_red', h=hh(0.60))     # 厨房，红色
trash_bin(1.95, 1.75, '0.12 0.25 0.85', 'trash_bin_blue', h=hh(0.60))    # 书房，蓝色
trash_bin(8.55, -1.95, '0.90 0.80 0.10', 'trash_bin_yellow', h=hh(0.60))  # 储藏间，黄色
fire_extinguisher(-8.35, 1.10, h=hh(0.52))                                # 走廊西端，灭火器

# ============ 走廊地标：给 scan matching 提供横向（沿走廊方向）特征 ============
# 为什么必须加：
#   走廊两侧是两面平行的长墙，沿走廊方向 (x) 没有任何可辨识特征。
#   相关性扫描匹配在这一维上是退化的 —— 车沿走廊滑一点点，匹配得分几乎不变，
#   SLAM 找不到唯一解，误差就累积成地图扭曲/重影（corridor aliasing）。
#   房间里家具多，所以房间从不扭曲；出问题的地方永远是走廊。
#
# 两条硬约束（踩过的坑）：
#   1) 雷达装在 0.78 m 高（2026-10-23 塔身改造前是 0.132 m），只能看见"穿过这个高度
#      的实体"。所以地标必须**落地**，且顶面高度要留足余量到 0.86 m 以上
#      （0.78 上只露 2~5 cm 的物件，激光截面太薄，匹配噪声大）。
#      原来按 0.132 m 平面挑的矮件（stool 0.45/0.50、umbrella_stand 0.55、
#      shoe_cabinet 0.70）在 0.78 平面上会**整个消失**，已换成同位置的高版本：
#        stool → 高脚凳 0.90/0.92、umbrella_stand → 落地伞架 1.10、
#        shoe_cabinet → 0.86（顶面 0.89）
#      挂画 / 门牌 / 壁灯 / 挂钟虽然够高，但贴在墙上、进深几乎为零，
#      对 2D 激光轮廓贡献很小，仍然不用。
#   2) 不能挡路。走廊净宽 2.88 m（y ∈ [-1.44, 1.44]），地标贴墙、
#      进深 ≤ 0.30 m、中心 y = ±1.29 → 中央仍留 2.28 m 通行（机器人宽 0.42 m）。
#   全部避开 8 个门洞（门中心 ±1.0 m 内不放），交错布置、间距 1.7~2.2 m，
#   外形各不相同（方/圆/高/矮/长条）—— 每一段走廊的激光轮廓都不同，匹配才有唯一解。

def _lam(name, cx, cy, parts, w, d, cls):
    add(model(name, f'{cx} {cy} 0', parts))
    foot(cx, cy, w, d, 0.0, name, cls=cls)

def shoe_cabinet(cx, cy, w=0.72, d=0.30, h=0.78):
    _lam('shoe_cabinet', cx, cy, [
        col('c', f'0 0 {h/2}', box(f'{w} {d} {h}')),
        vis('body', f'0 0 {h/2}', box(f'{w} {d} {h}'), WOODD),
        vis('top', f'0 0 {h+0.015}', box(f'{w+0.04} {d+0.04} 0.03'), FRAME),
    ], w, d, 'shoe_cabinet')

def potted_plant(cx, cy, r=0.19, ph=0.42):
    _lam('potted_plant', cx, cy, [
        col('c_base', f'0 0 {ph/2}', cyl(r, ph)),
        vis('v_base', f'0 0 {ph/2}', cyl(r, ph), TERRA),
        vis('v_leaf', f'0 0 {ph+0.30}', cyl(r * 1.5, 0.60), LEAF),
    ], 2 * r, 2 * r, 'potted_plant')

def stool(cx, cy, r=0.17, h=0.45):
    _lam('stool', cx, cy, [
        col('c', f'0 0 {h/2}', cyl(r, h)),
        vis('seat', f'0 0 {h-0.02}', cyl(r, 0.04), FAB),
        vis('leg', f'0 0 {h/2-0.02}', cyl(r * 0.35, h - 0.04), MET),
    ], 2 * r, 2 * r, 'stool')

def umbrella_stand(cx, cy, r=0.13, h=0.55):
    _lam('umbrella_stand', cx, cy, [
        col('c', f'0 0 {h/2}', cyl(r, h)),
        vis('body', f'0 0 {h/2}', cyl(r, h), MET),
    ], 2 * r, 2 * r, 'umbrella_stand')

def lam_console_table(cx, cy, w=0.90, d=0.28, h=0.80):
    _lam('console_table', cx, cy, [
        col('c', f'0 0 {h/2}', box(f'{w} {d} {h}')),
        vis('body', f'0 0 {h/2}', box(f'{w} {d} {h}'), WOODD),
        vis('top', f'0 0 {h+0.02}', box(f'{w+0.05} {d+0.05} 0.04'), FRAME),
    ], w, d, 'console_table')

def lam_bookshelf(cx, cy, w=0.80, d=0.26, h=1.20):
    _lam('bookshelf', cx, cy, [
        col('c', f'0 0 {h/2}', box(f'{w} {d} {h}')),
        vis('body', f'0 0 {h/2}', box(f'{w} {d} {h}'), WOODD),
        vis('shelf1', '0 0 0.45', box(f'{w-0.06} {d-0.04} 0.03'), WHITE),
        vis('shelf2', '0 0 0.85', box(f'{w-0.06} {d-0.04} 0.03'), WHITE),
    ], w, d, 'bookshelf')

_Y_N, _Y_S = 1.29, -1.29          # 贴墙（墙内表面 ±1.44，进深/2 后正好贴合）
shoe_cabinet(-8.00, _Y_N, h=0.86)                # 北墙西端
potted_plant(-7.80, _Y_S)                        # 南墙西端
stool(-4.60, _Y_N, h=0.90)                       # 高脚凳（原 0.45，低于 0.78 平面）
umbrella_stand(-5.00, _Y_S, h=1.10)              # 落地伞架（原 0.55，低于 0.78 平面）
lam_console_table(-3.00, _Y_N)                   # 长条台面 → 明显的纵向基准
stool(-3.40, _Y_S, r=0.15, h=0.92)               # 高脚凳（原 0.50）
lam_bookshelf(1.00, _Y_N)                        # 东段高书架
potted_plant(0.20, _Y_S, r=0.22, ph=0.38)
shoe_cabinet(2.60, _Y_N, w=0.60, d=0.28, h=0.86)  # 原 0.70 → 顶面 0.73，0.78 平面上会消失
umbrella_stand(4.40, _Y_S, h=1.10)               # 原 0.55
lam_console_table(7.00, _Y_N, w=0.80, d=0.26, h=0.86)   # 原 0.80 → 顶面 0.84，余量太薄
potted_plant(7.40, _Y_S, r=0.20)

# ================================================================
#                        巡 游 路 线
# ================================================================
ROUTE = [
    # ---- 走廊西段 ----
    (-6.0,  0.00, 'drive', '走廊西段'),
    (-8.2,  0.00, 'spin',  '走廊西端：扫西墙'),

    # ---- 客厅：两个观察点（门口内侧 + 深处）----
    (-6.0,  0.00, 'drive', '客厅门口'),
    (-6.0,  2.80, 'spin',  '客厅：门口内侧'),
    (-6.0,  4.60, 'spin',  '客厅：深处（沙发/电视）'),
    (-6.0,  0.00, 'drive', '退出客厅'),

    # ---- 卧室1 ----
    (-6.3,  0.00, 'drive', '卧室1门口'),
    (-6.3, -2.60, 'spin',  '卧室1：门口内侧'),
    (-6.3, -4.40, 'spin',  '卧室1：深处（床/衣柜）'),
    (-6.3,  0.00, 'drive', '退出卧室1'),

    # ---- 卧室2 ----
    (-1.5,  0.00, 'drive', '卧室2门口'),
    (-1.5, -2.60, 'spin',  '卧室2：门口内侧'),
    (-1.5, -4.20, 'spin',  '卧室2：深处'),
    (-1.5,  0.00, 'drive', '退出卧室2'),

    # ---- 卫生间 ----
    (2.2,   0.00, 'drive', '卫生间门口'),
    (2.2,  -2.40, 'spin',  '卫生间：门口内侧'),
    (2.0,  -4.00, 'spin',  '卫生间：深处（浴缸/洗手台）'),
    (2.2,   0.00, 'drive', '退出卫生间'),

    # ---- 储藏间 ----
    (6.2,   0.00, 'drive', '储藏间门口'),
    (6.2,  -2.60, 'spin',  '储藏间：门口内侧'),
    (6.2,  -4.40, 'spin',  '储藏间：深处（货架）'),
    (6.2,   0.00, 'drive', '退出储藏间'),

    # ---- 书房（两个观察点 + 穿内门）----
    (5.2,   0.00, 'drive', '书房门口'),
    (5.2,   2.80, 'spin',  '书房：门口内侧'),
    (4.6,   5.00, 'spin',  '书房：深处（书桌/书架）'),
    (3.20,  3.80, 'drive', '书房西侧'),
    (2.00,  3.80, 'spin',  '准备穿内门'),
    (0.60,  3.80, 'drive', '穿过内门进入厨房'),

    # ---- 厨房（从内门进来，两个观察点）----
    (0.30,  4.30, 'spin',  '厨房：东侧（操作台）'),
    (-0.80, 2.60, 'spin',  '厨房：中部（餐桌）'),
    (-0.80, 0.00, 'drive', '从厨房门退回走廊'),

    # ---- 收尾 ----
    (0.00,  0.00, 'spin',  '回到起点，收尾扫一圈'),
]

# ============ 机器人（Lighthouse 灯塔 / Lighthouse-3D）============
# 用绝对 file:// 路径（由脚本自己算，不写死用户路径）：最稳，不依赖 GZ_SIM_RESOURCE_PATH。
# SDF 内部的网格用 model://<ROBOT>/meshes/...，那个靠 launch 里设置的
# GZ_SIM_RESOURCE_PATH 解析（见 dan_house.launch.py）。
# --lidar 3d 时只换这一处 include，世界几何一个字节都不变（可用 diff 证明）。
add(f'''<include>
  <name>{ROBOT}</name>
  <uri>file://{os.path.join(HERE, 'models', ROBOT, 'model.sdf')}</uri>
  <pose>0 0 0.015 0 0 0</pose>
</include>
</world>
</sdf>''')

# ================================================================
#                        几 何 自 检
# ================================================================
MIN_CLEAR = 0.40     # 路线上每点要求的最小净距（机器人外接圆 0.23 + 余量）

def dist_point_box(px, py, bx, by, hx, hy):
    dx = max(abs(px - bx) - hx, 0.0)
    dy = max(abs(py - by) - hy, 0.0)
    return math.hypot(dx, dy)

def verify():
    print('—' * 64)
    print('几何自检')
    bad = []
    worst = (9.9, '-', '-')
    for i in range(len(ROUTE) - 1):
        ax, ay = ROUTE[i][0], ROUTE[i][1]
        bx, by = ROUTE[i + 1][0], ROUTE[i + 1][1]
        L = math.hypot(bx - ax, by - ay)
        n = max(int(L / 0.05), 1)
        for k in range(n + 1):
            t = k / n
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            for (fx, fy, hx, hy, nm) in FOOT:
                d = dist_point_box(px, py, fx, fy, hx, hy)
                if d < worst[0]:
                    worst = (d, nm, f'({px:.2f},{py:.2f})')
                if d < MIN_CLEAR:
                    bad.append(f'路线段 {i+1}->{i+2} 点 ({px:.2f},{py:.2f}) '
                               f'距 {nm} 仅 {d*100:.0f} cm')
    print(f'  路线 {len(ROUTE)} 航点 / {len(ROUTE)-1} 段，'
          f'全程最小净距 {worst[0]*100:.0f} cm（{worst[1]} @ {worst[2]}）'
          f'，要求 >= {MIN_CLEAR*100:.0f} cm')

    # 门洞通道：门中线两侧各 0.55 m，从走廊中心向房间内 2.4 m
    for label, (dx, dw), sgn in [('客厅', D_LIVING, +1), ('厨房', D_KITCHEN, +1),
                                 ('书房', D_STUDY, +1), ('卧室1', D_BED1, -1),
                                 ('卧室2', D_BED2, -1), ('卫生间', D_BATH, -1),
                                 ('储藏间', D_STORE, -1)]:
        for v in (0.0, 0.6, 1.2, 1.8, 2.4):
            for xoff in (-0.55, 0.0, 0.55):
                px, py = dx + xoff, sgn * v
                for (fx, fy, hx, hy, nm) in FOOT:
                    if dist_point_box(px, py, fx, fy, hx, hy) < 0.10:
                        bad.append(f'{label}门通道 ({px:.2f},{py:.2f}) 被 {nm} 占用')
    if bad:
        print(f'\n❌ 自检失败，{len(bad)} 处不达标：')
        for m in bad[:20]:
            print('   ', m)
        raise SystemExit(1)
    print('  ✅ 巡游路线 + 7 个门洞通道 全部达标')

# ============ 写出 ============
os.makedirs(os.path.dirname(OUT), exist_ok=True)
os.makedirs(os.path.dirname(ROUTE_JSON), exist_ok=True)
with open(OUT, 'w') as f:
    f.write('\n'.join(X))
with open(ROUTE_JSON, 'w') as f:
    json.dump({'waypoints': ROUTE, 'note': '由 build_world.py 生成，勿手改'}, f,
              ensure_ascii=False, indent=1)

n_models = '\n'.join(X).count('<model name=')
print(f'✅ 已生成 {OUT}')
print(f'   机器人 {ROBOT}（雷达 {"单线 LaserScan" if ARGS.lidar == "2d" else "64 线 PointCloud2"}）')
print(f'   房子 {XE-XW:.1f} x {YN-YS:.1f} m | 房间 {len(ROOMS)} 个 | 静态模型 {n_models} 个')
print(f'   走廊净宽 {2*COR-T_IN:.2f} m | 门洞 {DOOR_W} m（卫生间 {DOOR_W_BATH}）| 层高 {H} m')
for n, x0, x1, y0, y1, _ in ROOMS:
    print(f'   {n:8s} 净尺寸 {x1-x0:.2f} x {y1-y0:.2f} m')
verify()

# ============ 语义地图标注（第二阶段语义导航的真值） ============
ROOM_SEM = [
    ('living',  'living_room', '客厅',   D_LIVING,  +1),
    ('kitchen', 'kitchen',     '厨房',   D_KITCHEN, +1),
    ('study',   'study',       '书房',   D_STUDY,   +1),
    ('bed1',    'bedroom_1',   '卧室1',  D_BED1,    -1),
    ('bed2',    'bedroom_2',   '卧室2',  D_BED2,    -1),
    ('bath',    'bathroom',    '卫生间', D_BATH,    -1),
    ('store',   'storage',     '储藏间', D_STORE,   -1),
]
rmap = {r[0]: r for r in ROOMS}
rooms_sem = []
for key, en, zh, (dx, dw), sgn in ROOM_SEM:
    _, x0, x1, y0, y1, _ = rmap[key]
    rooms_sem.append({
        'name': en, 'label_zh': zh,
        'rect': [x0, y0, x1, y1],
        'center': [round((x0 + x1) / 2, 3), round((y0 + y1) / 2, 3)],
        'door_center': [dx, round(sgn * COR, 3)],
        'door_width': dw,
    })
rooms_sem.append({'name': 'corridor', 'label_zh': '走廊',
                  'rect': [-8.89, -1.44, 8.89, 1.44],
                  'center': [0.0, 0.0], 'door_center': None, 'door_width': None})

# 物体之间的最小间距（越大越利于视觉分割/语义识别）
gap = (9.9, '', '')
for i in range(len(SEM)):
    for j in range(i + 1, len(SEM)):
        a, b = SEM[i], SEM[j]
        d = math.hypot(a['x'] - b['x'], a['y'] - b['y']) \
            - max(a['w'], a['d']) / 2 - max(b['w'], b['d']) / 2
        if d < gap[0]:
            gap = (d, a['id'], b['id'])

with open(SEM_JSON, 'w') as f:
    json.dump({
        'frame': 'odom/map（机器人出生点为原点，朝向 +x）',
        'note': '由 build_world.py 自动生成，供第二阶段语义导航做真值/评测',
        'rooms': rooms_sem,
        'objects': SEM,
    }, f, ensure_ascii=False, indent=1)

print(f'✅ 语义标注已写入 {SEM_JSON}')
print(f'   {len(rooms_sem)} 个房间 / {len(SEM)} 个带类别的物体')
print(f'   物体间最小间距 {gap[0]*100:.0f} cm（{gap[1]} <-> {gap[2]}）')
from collections import Counter
cnt = Counter(o['class'] for o in SEM)
print('   类别统计: ' + ', '.join(f'{k}x{v}' for k, v in sorted(cnt.items())))
print(f'✅ 路线已写入 {ROUTE_JSON}')
