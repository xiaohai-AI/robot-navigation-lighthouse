#!/usr/bin/env python3
"""
参数化生成 dan_house.world  ——  7 房间 / 16 x 12 m 公寓 (Gazebo Fortress, gz-sim 6, SDF 1.9)

运行:
    python3 build_world.py          # 覆盖写入 worlds/dan_house.world

设计说明
--------
整栋房子 16 m (x) x 12 m (y)，中间一条 2.08 m 宽的东西向走廊，南北各一排房间：

    y=+6  ┌──────────┬────────┬──────────────────┐
          │ 客厅      │ 厨房    │      书房         │
    y=+1.1├──────────┴────────┴──────────────────┤   <- 北侧走廊墙 (3 个门)
          │        走廊 (2.08 m 宽，全长 15.8 m)     │
    y=-1.1├──────────┬────────┬────────┬──────────┤   <- 南侧走廊墙 (4 个门)
          │ 卧室 1    │ 卧室 2  │ 卫生间 │  储藏间   │
    y=-6  └──────────┴────────┴────────┴──────────┘

房间净尺寸都 >= 2.4 x 4.7 m（原来 3 室只有 ~2 x 2.4 m），门洞 1.0 m，
门洞前 1.3 m 保持净空 —— 0.31 m 的 TB3 不会再擦墙。
厨房 <-> 书房之间还有一道内门，形成环路，方便测试 SLAM 回环。

坐标约定：所有墙用"中心线"定义，地板/家具用房间净尺寸。
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'worlds', 'dan_house.world')
TEX = 'textures/'

# ============ 全局尺寸 ============
XW, XE = -8.0, 8.0        # 外墙中心线 x
YS, YN = -6.0, 6.0        # 外墙中心线 y
T_OUT, T_IN = 0.22, 0.12  # 外墙 / 内墙厚度
H = 2.6                   # 层高
COR = 1.1                 # 走廊两侧墙的中心线 y = ±COR
DOOR_W = 1.0              # 标准门宽
DOOR_H = 2.05             # 门洞高
CEILING = False           # True 会挡住俯瞰视角，需要录视频时可以打开

X = []
def add(s): X.append(s)

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
    """pose 字符串：3 个值补 0 0 0，6 个值原样"""
    v = p.split()
    return ' '.join(v + ['0', '0', '0']) if len(v) == 3 else ' '.join(v)

def vis(n, p, g, m): return f'<visual name="{n}"><pose>{P(p)}</pose><geometry>{g}</geometry>{m}</visual>'
def col(n, p, g):    return f'<collision name="{n}"><pose>{P(p)}</pose><geometry>{g}</geometry></collision>'

_SEEN = {}
def model(n, pose, parts):
    """同一 world 里模型名必须唯一，所以这里自动加 _2 _3 后缀"""
    i = _SEEN.get(n, 0) + 1
    _SEEN[n] = i
    uniq = n if i == 1 else f'{n}_{i}'
    return (f'<model name="{uniq}"><pose>{P(pose)}</pose><static>true</static>'
            f'<link name="link">{"".join(parts)}</link></model>')

WOOD  = mat_t('wood_floor.jpg'); WOODD = mat_t('wood_dark.jpg')
WALL  = mat_t('wallpaper.jpg');  TILE  = mat_t('tile.jpg')
CARP  = mat_t('carpet.jpg');     FAB   = mat_t('fabric.jpg')
MET   = mat_t('metal.jpg');      SCR   = mat_t('screen.jpg')
BED   = mat_t('bedding.jpg');    RUG   = mat_t('rug.jpg')
PAINT = mat_t('painting.jpg');   LEAF  = mat_t('leaf.jpg')
TERRA = mat_t('terracotta.jpg')
WHITE = mat_c('0.95 0.95 0.95'); DARKW = mat_c('0.2 0.15 0.1')
FRAME = mat_c('0.36 0.26 0.18')

# ============ 房间表（净尺寸，不含墙） ============
# x0/x1/y0/y1 = 墙面内侧坐标；这些值由墙的中心线 ± 半厚算出来，改墙就要同步改这里
ROOMS = [
    # name          x0      x1      y0      y1     floor
    ('living',   -7.89, -2.86,   1.16,  5.89,  WOOD),
    ('kitchen',  -2.74,   1.34,   1.16,  5.89,  TILE),
    ('study',     1.46,   7.89,   1.16,  5.89,  WOODD),
    ('bed1',     -7.89,  -3.66,  -5.89, -1.16,  CARP),
    ('bed2',     -3.54,   0.54,  -5.89, -1.16,  CARP),
    ('bath',      0.66,   3.14,  -5.89, -1.16,  TILE),
    ('store',     3.26,   7.89,  -5.89, -1.16,  WOODD),
]
ROOM = {r[0]: r for r in ROOMS}

# ============ 头部：物理 / 插件 / 灯光 ============
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

# 每个房间一个顶灯 + 走廊每 3 m 一个
# 性能要点：点光源的 <cast_shadows> 在 ogre2 里每盏都要渲 6 个面的 shadow map，
# 12 盏影子灯能把 GUI 帧率吃光（实测 RTF 掉到 0.15~0.5，/scan 从 10 Hz 变 5 Hz）。
# 所以只有太阳投影，室内点光源一律不投影 —— 视觉上差别很小，性能差很多。
for name, x0, x1, y0, y1, _ in ROOMS:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    warm = '1.0 0.94 0.85' if name in ('living', 'bed1', 'bed2') else '1.0 1.0 1.0'
    add(f'<light type="point" name="light_{name}"><cast_shadows>false</cast_shadows>'
        f'<pose>{cx} {cy} {H - 0.15} 0 0 0</pose><diffuse>{warm} 1</diffuse>'
        f'<attenuation><range>14</range><constant>0.4</constant><linear>0.3</linear>'
        f'<quadratic>0.02</quadratic></attenuation></light>')
for i, lx in enumerate([-6.5, -3.2, 0.0, 3.2, 6.5]):
    add(f'<light type="point" name="light_corridor_{i}"><cast_shadows>false</cast_shadows>'
        f'<pose>{lx} 0 {H - 0.15} 0 0 0</pose><diffuse>1.0 0.98 0.92 1</diffuse>'
        f'<attenuation><range>10</range><constant>0.5</constant><linear>0.35</linear>'
        f'<quadratic>0.04</quadratic></attenuation></light>')

# ============ 地面 & 地板 ============
add(model('ground_plane', '0 0 0', [
    col('c', '0 0 0', '<plane><normal>0 0 1</normal><size>80 80</size></plane>'),
    vis('v', '0 0 0', '<plane><normal>0 0 1</normal><size>80 80</size></plane>', mat_c('0.35 0.35 0.36'))]))

for name, x0, x1, y0, y1, tex in ROOMS:
    g = box(f'{x1-x0:.3f} {y1-y0:.3f} 0.012')
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    # 注意：地板必须有 collision！
    # 只给 visual 的话轮子会踩在 z=0 的 ground_plane 上，
    # 而地板顶面在 z=0.012 -> 视觉上每只轮子都"陷进地板"12 mm（TB3 轮半径才 33 mm）。
    add(model(f'floor_{name}', '0 0 0', [
        col('c', f'{cx} {cy} 0.006', g),
        vis('v', f'{cx} {cy} 0.006', g, tex)]))
# 走廊地板
add(model('floor_corridor', '0 0 0', [
    col('c', '0 0 0.006', box(f'{XE-XW-T_OUT:.3f} {2*COR-T_IN:.3f} 0.012')),
    vis('v', '0 0 0.006', box(f'{XE-XW-T_OUT:.3f} {2*COR-T_IN:.3f} 0.012'), WOOD)]))

if CEILING:
    add(model('ceiling', '0 0 0', [
        vis('v', f'0 0 {H+0.05}', box(f'{XE-XW+0.3:.2f} {YN-YS+0.3:.2f} 0.1'), mat_c('0.96 0.96 0.95'))]))

# ============ 墙体生成 ============
def _gaps(a, b, doors):
    """在 [a,b] 区间挖掉门洞，返回剩余的墙段"""
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
    """门框（2 根立柱 + 1 根横梁），纯装饰，不参与碰撞"""
    h, fw, fd = DOOR_H + 0.05, 0.07, T_IN + 0.04
    if vertical:  # 墙沿 y 走向
        for i, dy in enumerate((-1, 1)):
            add(model(f'{name}_p{i}', f'{cx} {cy + dy*(w/2 - fw/2)} {h/2}',
                      [vis('v', '0 0 0', box(f'{fd:.3f} {fw} {h:.2f}'), FRAME)]))
        add(model(f'{name}_top', f'{cx} {cy} {h - fw/2}',
                  [vis('v', '0 0 0', box(f'{fd:.3f} {w} {fw}'), FRAME)]))
    else:         # 墙沿 x 走向
        for i, dx in enumerate((-1, 1)):
            add(model(f'{name}_p{i}', f'{cx + dx*(w/2 - fw/2)} {cy} {h/2}',
                      [vis('v', '0 0 0', box(f'{fw} {fd:.3f} {h:.2f}'), FRAME)]))
        add(model(f'{name}_top', f'{cx} {cy} {h - fw/2}',
                  [vis('v', '0 0 0', box(f'{w} {fd:.3f} {fw}'), FRAME)]))
    # 门楣（门洞上方的过梁，让门洞看起来像门）
    lintel_h = H - DOOR_H
    if vertical:
        add(model(f'{name}_lintel', f'{cx} {cy} {DOOR_H + lintel_h/2}',
                  [vis('v', '0 0 0', box(f'{T_IN} {w} {lintel_h:.3f}'), WALL)]))
    else:
        add(model(f'{name}_lintel', f'{cx} {cy} {DOOR_H + lintel_h/2}',
                  [vis('v', '0 0 0', box(f'{w} {T_IN} {lintel_h:.3f}'), WALL)]))

def hwall(name, y, x0, x1, t, doors=()):
    """沿 x 走向的墙，门洞位于 x 轴"""
    for i, (a, b) in enumerate(_gaps(x0, x1, doors)):
        add(model(f'{name}_{i}', f'{(a+b)/2:.3f} {y} {H/2}',
                  [col('c', '0 0 0', box(f'{b-a:.3f} {t} {H}')),
                   vis('v', '0 0 0', box(f'{b-a:.3f} {t} {H}'), WALL)]))
    for cx, w in doors:
        door_frame(f'{name}_door_{cx}', cx, y, w, vertical=False)

def vwall(name, x, y0, y1, t, doors=()):
    """沿 y 走向的墙，门洞位于 y 轴"""
    for i, (a, b) in enumerate(_gaps(y0, y1, doors)):
        add(model(f'{name}_{i}', f'{x} {(a+b)/2:.3f} {H/2}',
                  [col('c', '0 0 0', box(f'{t} {b-a:.3f} {H}')),
                   vis('v', '0 0 0', box(f'{t} {b-a:.3f} {H}'), WALL)]))
    for cy, w in doors:
        door_frame(f'{name}_door_{cy}', x, cy, w, vertical=True)

# ---- 外墙 ----
hwall('wall_south', YS, XW, XE, T_OUT)
hwall('wall_north', YN, XW, XE, T_OUT)
vwall('wall_west', XW, YS, YN, T_OUT)
vwall('wall_east', XE, YS, YN, T_OUT)

# ---- 走廊两侧内墙（7 个房间门） ----
hwall('wall_corr_north', +COR, XW, XE, T_IN,
      doors=[(-5.4, DOOR_W),   # 客厅
             (-0.7, DOOR_W),   # 厨房
             (4.6, DOOR_W)])   # 书房
hwall('wall_corr_south', -COR, XW, XE, T_IN,
      doors=[(-5.6, DOOR_W),   # 卧室 1
             (-1.6, DOOR_W),   # 卧室 2
             (1.9, 0.95),      # 卫生间（窄一点）
             (5.6, DOOR_W)])   # 储藏间

# ---- 房间之间的纵向隔墙 ----
vwall('wall_living_kitchen', -2.8, +COR, YN, T_IN)                     # 客厅 | 厨房
vwall('wall_kitchen_study',  +1.4, +COR, YN, T_IN, doors=[(3.5, DOOR_W)])  # 厨房 | 书房（内门 -> 环路）
vwall('wall_bed1_bed2',      -3.6, YS, -COR, T_IN)                     # 卧室 1 | 卧室 2
vwall('wall_bed2_bath',      +0.6, YS, -COR, T_IN)                     # 卧室 2 | 卫生间
vwall('wall_bath_store',     +3.2, YS, -COR, T_IN)                     # 卫生间 | 储藏间

# ============ 窗 & 挂画 ============
def window(name, x, y, rot_z=0.0, w=1.6):
    add(model(name, f'{x} {y} 1.55 0 0 {rot_z}', [
        vis('pane', '0 0 0', box(f'{w*0.9:.2f} 0.06 1.15'), mat_c('0.55 0.72 0.9', '0.14 0.2 0.3')),
        vis('ft', '0 0 0.62', box(f'{w:.2f} 0.1 0.08'), WHITE),
        vis('fb', '0 0 -0.62', box(f'{w:.2f} 0.1 0.08'), WHITE),
        vis('fl', f'{-w/2:.2f} 0 0', box('0.1 0.1 1.15'), WHITE),
        vis('fr', f'{w/2:.2f} 0 0', box('0.1 0.1 1.15'), WHITE)]))

for i, wx in enumerate([-6.4, -4.2, -1.4, 3.4, 6.0]):
    window(f'window_n{i+1}', wx, YN - 0.05)
for i, wx in enumerate([-6.6, -5.0, -2.6, 1.9, 6.4]):
    window(f'window_s{i+1}', wx, YS + 0.05)

def painting(name, x, y, rot_z):
    add(model(name, f'{x} {y} 1.55 0 0 {rot_z}', [
        vis('frame', '0 0 0', box('0.02 0.78 0.63'), mat_c('0.15 0.12 0.1')),
        vis('canvas', '0.025 0 0', box('0.03 0.7 0.55'), PAINT)]))

painting('painting_1', XW + 0.11, 1.9, 0)          # 客厅西墙
painting('painting_2', 3.9, -COR + 0.06, 3.1416)   # 走廊南墙（卧室2 门东侧）
painting('painting_3', -2.75, YN - 0.06, 3.1416)   # 厨房北墙

# ================================================================
#                          家 具
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

def coffee_table(cx, cy, rz=0.0):
    add(model('coffee_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.2', box('1.0 0.6 0.4')),
        vis('top', '0 0 0.375', box('1.0 0.6 0.05'), WOODD),
        vis('l1', '-0.42 -0.24 0.175', cyl(0.025, 0.35), DARKW), vis('l2', '0.42 -0.24 0.175', cyl(0.025, 0.35), DARKW),
        vis('l3', '-0.42 0.24 0.175', cyl(0.025, 0.35), DARKW), vis('l4', '0.42 0.24 0.175', cyl(0.025, 0.35), DARKW),
        vis('b1', '0.15 0.1 0.415', box('0.22 0.16 0.03'), mat_c('0.6 0.2 0.2')),
        vis('b2', '0.14 0.11 0.445', box('0.2 0.14 0.03'), mat_c('0.2 0.3 0.6'))]))

def tv_set(cx, cy, rz=0.0):
    """电视柜 + 电视，屏幕朝局部 +x 方向"""
    p = [col('c', '0 0 0.25', box('0.45 1.8 0.5')), vis('body', '0 0 0.25', box('0.45 1.8 0.5'), WOODD),
         vis('d1', '0.235 -0.45 0.25', box('0.02 0.86 0.44'), mat_c('0.25 0.18 0.12')),
         vis('d2', '0.235 0.45 0.25', box('0.02 0.86 0.44'), mat_c('0.25 0.18 0.12'))]
    for i, (nx, ny) in enumerate([(0.25, -0.06), (0.25, 0.06)]):
        p.append(vis(f'h{i+1}', f'{nx} {ny} 0.25', box('0.03 0.03 0.12'), mat_c('0.7 0.7 0.72')))
    p += [col('tv_c', '-0.18 0 0.95', box('0.07 1.5 0.9')),
          vis('tvframe', '-0.18 0 0.95', box('0.07 1.5 0.9'), mat_c('0.05 0.05 0.05')),
          vis('screen', '-0.135 0 0.95', box('0.02 1.36 0.76'),
              '<material><diffuse>1 1 1 1</diffuse><emissive>0.15 0.17 0.22 1</emissive><pbr><metal>'
              '<albedo_map>textures/screen.jpg</albedo_map><metalness>0.0</metalness><roughness>0.3</roughness></metal></pbr></material>'),
          vis('neck', '-0.18 0 0.44', box('0.06 0.25 0.12'), mat_c('0.05 0.05 0.05'))]
    add(model('tv_set', f'{cx} {cy} 0 0 0 {rz}', p))

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
    for i, bc in enumerate(['0.3 0.3 0.35', '0.6 0.4 0.25', '0.2 0.5 0.5']):
        bs.append(vis(f'c{i+1}', f'{-0.35+i*0.13} 0 1.035', box('0.07 0.24 0.24'), mat_c(bc)))
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', bs))

def floor_lamp(cx, cy):
    add(model('floor_lamp', f'{cx} {cy} 0', [
        col('c', '0 0 0.7', cyl(0.03, 1.4)), vis('base', '0 0 0.01', cyl(0.15, 0.02), mat_c('0.2 0.2 0.2')),
        vis('pole', '0 0 0.66', cyl(0.02, 1.28), mat_c('0.25 0.25 0.25')),
        vis('shade', '0 0 1.4', cyl(0.16, 0.26), mat_c('0.95 0.9 0.75', '0.6 0.5 0.32'))]))

def side_table(cx, cy, rz=0.0):
    add(model('side_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.3', box('0.6 0.6 0.6')), vis('body', '0 0 0.3', box('0.6 0.6 0.6'), WOODD),
        vis('mug', '0.1 -0.1 0.645', cyl(0.04, 0.09), WHITE)]))

def plant(cx, cy):
    add(model('plant', f'{cx} {cy} 0', [
        col('pot_c', '0 0 0.175', cyl(0.18, 0.35)), col('leaf_c', '0 0 0.8', cyl(0.35, 0.9)),
        vis('pot', '0 0 0.175', cyl(0.18, 0.35), TERRA), vis('soil', '0 0 0.36', cyl(0.16, 0.02), mat_c('0.2 0.14 0.1')),
        vis('trunk', '0 0 0.63', cyl(0.03, 0.55), mat_c('0.35 0.25 0.15')),
        vis('lf1', '0 0 1.05', sph(0.3), LEAF), vis('lf2', '0.18 0.1 0.9', sph(0.22), LEAF),
        vis('lf3', '-0.15 -0.1 0.95', sph(0.2), LEAF)]))

def rug(cx, cy, w, h, rz=0.0):
    # 地毯薄薄一层贴在地板顶面(0.012)上，别做成 2 cm 厚的大块——否则轮子会"陷进地毯"
    add(model('rug', f'{cx} {cy} 0.0125 0 0 {rz}', [vis('v', '0 0 0', box(f'{w} {h} 0.008'), RUG)]))

def kitchen_counter(cx, cy, rz=0.0):
    kc = [col('c', '0 0 0.45', box('2.6 0.65 0.9')), vis('cab', '0 0 0.425', box('2.6 0.65 0.85'), mat_c('0.92 0.92 0.9')),
          vis('top', '0 0 0.88', box('2.72 0.74 0.06'), mat_c('0.32 0.32 0.35')),
          vis('sink', '-0.6 -0.15 0.912', box('0.45 0.35 0.02'), mat_c('0.18 0.18 0.2')),
          vis('faucet', '-0.6 -0.28 1.0', cyl(0.015, 0.22), mat_c('0.75 0.75 0.78')),
          vis('stove', '0.6 0 0.912', box('0.55 0.45 0.02'), mat_c('0.08 0.08 0.08'))]
    for i, (bx, by) in enumerate([(0.45, 0.12), (0.75, 0.12), (0.45, -0.12), (0.75, -0.12)]):
        kc.append(vis(f'burn{i+1}', f'{bx} {by} 0.925', cyl(0.07, 0.012), mat_c('0.15 0.15 0.15')))
    for i, hx in enumerate([-1.0, -0.35, 0.35, 1.0]):
        kc.append(vis(f'h{i+1}', f'{hx} -0.345 0.7', box('0.18 0.03 0.03'), mat_c('0.7 0.7 0.72')))
    add(model('kitchen_counter', f'{cx} {cy} 0 0 0 {rz}', kc))

def counter_side(cx, cy, rz=0.0):
    add(model('counter_side', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.45', box('0.65 1.6 0.9')), vis('cab', '0 0 0.425', box('0.65 1.6 0.85'), mat_c('0.92 0.92 0.9')),
        vis('top', '0 0 0.88', box('0.74 1.7 0.06'), mat_c('0.32 0.32 0.35'))]))

def fridge(cx, cy, rz=0.0):
    add(model('fridge', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.9', box('0.75 0.75 1.8')), vis('body', '0 0 0.9', box('0.75 0.75 1.8'), MET),
        vis('split', '0 -0.38 1.15', box('0.72 0.02 0.03'), mat_c('0.4 0.4 0.42')),
        vis('h1', '0.28 -0.4 0.75', box('0.03 0.03 0.5'), mat_c('0.3 0.3 0.32')),
        vis('h2', '0.28 -0.4 1.45', box('0.03 0.03 0.4'), mat_c('0.3 0.3 0.32'))]))

def dining_table(cx, cy):
    add(model('dining_table', f'{cx} {cy} 0', [
        col('top_c', '0 0 0.725', cyl(0.55, 0.05)), col('post_c', '0 0 0.35', cyl(0.06, 0.68)),
        vis('top', '0 0 0.725', cyl(0.55, 0.05), WOODD), vis('post', '0 0 0.35', cyl(0.06, 0.68), mat_c('0.25 0.18 0.12')),
        vis('base', '0 0 0.015', cyl(0.28, 0.03), mat_c('0.25 0.18 0.12'))]))

def chair(cx, cy, rz=0.0, name='chair'):
    p = [col('c', '0 0 0.475', box('0.45 0.45 0.95')), vis('seat', '0 0 0.45', box('0.45 0.45 0.04'), WOODD),
         vis('back', '0 -0.2 0.75', box('0.45 0.05 0.55'), WOODD)]
    for i, (lx, ly) in enumerate([(-0.19, -0.19), (0.19, -0.19), (-0.19, 0.19), (0.19, 0.19)]):
        p.append(vis(f'l{i+1}', f'{lx} {ly} 0.215', cyl(0.02, 0.43), DARKW))
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', p))

def desk(cx, cy, rz=0.0, name='desk'):
    p = [col('c', '0 0 0.38', box('1.4 0.7 0.76')),
         vis('top', '0 0 0.75', box('1.45 0.72 0.05'), WOODD),
         vis('leg1', '-0.66 -0.3 0.36', box('0.06 0.06 0.72'), DARKW),
         vis('leg2', '0.66 -0.3 0.36', box('0.06 0.06 0.72'), DARKW),
         vis('leg3', '-0.66 0.3 0.36', box('0.06 0.06 0.72'), DARKW),
         vis('leg4', '0.66 0.3 0.36', box('0.06 0.06 0.72'), DARKW),
         vis('monitor', '0 0.28 1.03', box('0.06 0.62 0.38'), mat_c('0.08 0.08 0.09')),
         vis('mstand', '0 0.28 0.83', box('0.06 0.06 0.12'), mat_c('0.3 0.3 0.32')),
         vis('keyb', '0 -0.05 0.785', box('0.42 0.14 0.02'), mat_c('0.15 0.15 0.17')),
         vis('mug', '0.42 0.1 0.815', cyl(0.04, 0.09), WHITE)]
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', p))

def bed(cx, cy, rz=0.0):
    add(model('bed', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.225', box('1.5 2.0 0.45')), vis('frame', '0 0 0.11', box('1.6 2.1 0.22'), WOODD),
        vis('mattress', '0 0 0.33', box('1.5 2.0 0.22'), BED), vis('head', '0 -1.0 0.6', box('1.6 0.1 0.9'), WOODD),
        vis('p1', '-0.4 -0.75 0.48', box('0.55 0.35 0.09'), WHITE),
        vis('p2', '0.4 -0.75 0.48', box('0.55 0.35 0.09'), WHITE)]))

def nightstand(cx, cy, rz=0.0):
    add(model('nightstand', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.25', box('0.45 0.45 0.5')), vis('body', '0 0 0.25', box('0.45 0.45 0.5'), WOODD),
        vis('drawer', '0.235 0 0.35', box('0.02 0.4 0.18'), mat_c('0.3 0.22 0.15')),
        vis('lb', '0 0 0.51', cyl(0.06, 0.02), mat_c('0.3 0.3 0.3')),
        vis('lp', '0 0 0.61', cyl(0.015, 0.18), mat_c('0.3 0.3 0.3')),
        vis('ls', '0 0 0.75', cyl(0.09, 0.12), mat_c('0.95 0.85 0.6', '0.55 0.45 0.28'))]))

def wardrobe(cx, cy, rz=0.0):
    add(model('wardrobe', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.95', box('1.0 0.6 1.9')), vis('body', '0 0 0.95', box('1.0 0.6 1.9'), WOODD),
        vis('d1', '-0.245 -0.31 0.95', box('0.48 0.02 1.8'), mat_c('0.42 0.32 0.22')),
        vis('d2', '0.245 -0.31 0.95', box('0.48 0.02 1.8'), mat_c('0.45 0.35 0.25')),
        vis('h1', '-0.05 -0.33 1.0', box('0.03 0.03 0.12'), mat_c('0.7 0.7 0.72')),
        vis('h2', '0.05 -0.33 1.0', box('0.03 0.03 0.12'), mat_c('0.7 0.7 0.72'))]))

# ---- 新增：卫浴 / 储藏 / 书房的构件 ----
def toilet(cx, cy, rz=0.0):
    """水箱在局部 +y，坐便朝 -y"""
    add(model('toilet', f'{cx} {cy} 0 0 0 {rz}', [
        col('bowl_c', '0 0.12 0.2', box('0.38 0.55 0.4')),
        vis('base', '0 0.12 0.2', box('0.34 0.5 0.38'), WHITE),
        vis('seat', '0 0.12 0.4', cyl(0.19, 0.06), mat_c('0.92 0.92 0.94')),
        col('tank_c', '0 0.42 0.5', box('0.4 0.18 0.55')),
        vis('tank', '0 0.42 0.5', box('0.4 0.18 0.55'), WHITE)]))

def sink_basin(cx, cy, rz=0.0):
    """镜子/靠墙在局部 +y"""
    add(model('sink_basin', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.42', box('0.6 0.45 0.84')),
        vis('cab', '0 0 0.4', box('0.58 0.43 0.78'), mat_c('0.9 0.9 0.9')),
        vis('top', '0 0 0.83', box('0.64 0.48 0.06'), mat_c('0.95 0.95 0.97')),
        vis('basin', '0 -0.05 0.86', cyl(0.16, 0.04), mat_c('0.85 0.87 0.9')),
        vis('tap', '0 0.16 0.95', cyl(0.015, 0.18), mat_c('0.75 0.75 0.78')),
        vis('mirror', '0 0.22 1.5', box('0.6 0.02 0.7'), mat_c('0.7 0.8 0.85', '0.2 0.22 0.24'))]))

def bathtub(cx, cy, rz=0.0, w=0.9, l=2.2):
    add(model('bathtub', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.28', box(f'{w} {l} 0.56')),
        vis('body', '0 0 0.28', box(f'{w} {l} 0.56'), mat_c('0.97 0.97 0.98')),
        vis('inner', '0 0 0.5', box(f'{w-0.16:.2f} {l-0.3:.2f} 0.06'), mat_c('0.82 0.86 0.9')),
        vis('tap', f'0 {l/2-0.2:.2f} 0.62', cyl(0.02, 0.14), mat_c('0.75 0.75 0.78'))]))

def washer(cx, cy, rz=0.0):
    add(model('washer', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.42', box('0.6 0.6 0.84')),
        vis('body', '0 0 0.42', box('0.6 0.6 0.84'), mat_c('0.93 0.93 0.95')),
        vis('door', '0 -0.31 0.45', cyl(0.19, 0.04), mat_c('0.55 0.62 0.68')),
        vis('panel', '0 -0.31 0.78', box('0.5 0.03 0.1'), mat_c('0.2 0.2 0.22'))]))

def shelf_unit(cx, cy, rz=0.0, w=1.2, d=0.5, h=1.9, name='shelf_unit'):
    p = [col('c', f'0 0 {h/2}', box(f'{w} {d} {h}')),
         vis('back', f'0 {-d/2+0.02:.3f} {h/2}', box(f'{w} {0.04} {h}'), MET)]
    for side, sx in (('l', -w/2 + 0.03), ('r', w/2 - 0.03)):
        p.append(vis(f's{side}', f'{sx:.3f} 0 {h/2}', box('0.06 0.5 ' + f'{h}'), MET))
    for i in range(1, 4):
        p.append(vis(f'shelf{i}', f'0 0 {i*h/4:.3f}', box(f'{w-0.1:.2f} {d-0.06:.2f} 0.04'), mat_c('0.55 0.56 0.58')))
    for i, bc in enumerate(['0.55 0.4 0.25', '0.3 0.35 0.5', '0.6 0.55 0.3']):
        p.append(vis(f'box{i+1}', f'{-0.3+0.3*i:.2f} 0 {0.55+i*0.47:.2f}', box('0.34 0.3 0.28'), mat_c(bc)))
    add(model(name, f'{cx} {cy} 0 0 0 {rz}', p))

def crate(cx, cy, s=0.6, h=0.6, col_rgb='0.55 0.4 0.22'):
    add(model('crate', f'{cx} {cy} 0', [
        col('c', f'0 0 {h/2}', box(f'{s} {s} {h}')),
        vis('v', f'0 0 {h/2}', box(f'{s} {s} {h}'), mat_c(col_rgb)),
        vis('band', f'0 0 {h*0.75:.2f}', box(f'{s+0.01} {s+0.01} 0.06'), mat_c('0.3 0.3 0.32'))]))

def pillar(cx, cy, s=0.4):
    add(model('pillar', f'{cx} {cy} 0', [
        col('c', f'0 0 {H/2}', box(f'{s} {s} {H}')),
        vis('v', f'0 0 {H/2}', box(f'{s} {s} {H}'), mat_c('0.9 0.88 0.84'))]))

def console_table(cx, cy, rz=0.0):
    add(model('console_table', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.4', box('1.0 0.32 0.8')),
        vis('top', '0 0 0.78', box('1.05 0.34 0.05'), WOODD),
        vis('leg1', '-0.45 -0.12 0.38', box('0.05 0.05 0.76'), DARKW),
        vis('leg2', '0.45 -0.12 0.38', box('0.05 0.05 0.76'), DARKW),
        vis('vase', '0.3 0 0.88', cyl(0.07, 0.16), TERRA)]))

def cart(cx, cy, rz=0.0):
    add(model('cart', f'{cx} {cy} 0 0 0 {rz}', [
        col('c', '0 0 0.45', box('0.55 0.4 0.9')),
        vis('body', '0 0 0.45', box('0.55 0.4 0.9'), mat_c('0.75 0.78 0.8')),
        vis('tray', '0 0 0.92', box('0.58 0.43 0.03'), mat_c('0.35 0.37 0.4')),
        vis('w1', '-0.2 -0.22 0.03', cyl(0.03, 0.03), mat_c('0.15 0.15 0.15')),
        vis('w2', '0.2 -0.22 0.03', cyl(0.03, 0.03), mat_c('0.15 0.15 0.15'))]))

# ================================================================
#                     按房间摆放家具（留出 1 m 门洞通道）
# ================================================================
# ---- 客厅 x[-7.89,-2.86] y[1.16,5.89]，门 x=-5.4 ----
sofa(-7.25, 3.45, rz=1.5708)                 # 靠西墙，背朝 -x
coffee_table(-6.05, 3.45, rz=1.5708)
rug(-6.1, 3.4, 1.9, 2.6)
tv_set(-4.3, 5.665, rz=-1.5708)              # 靠北墙，屏幕朝 -y
bookshelf(-3.04, 4.2, rz=1.5708)              # 靠东墙
floor_lamp(-3.35, 2.0)
side_table(-7.45, 1.85)
plant(-7.5, 5.45)

# ---- 厨房 x[-2.74,1.34] y[1.16,5.89]，门 x=-0.7 ----
kitchen_counter(-1.35, 5.565)                 # 靠北墙
fridge(-2.365, 4.5, rz=1.5708)                # 靠西墙
counter_side(1.015, 2.0)                      # 靠东墙
dining_table(-0.9, 3.6)
chair(-0.9, 2.8)
chair(-0.9, 4.4, rz=3.1416)
chair(0.1, 3.6, rz=1.5708)

# ---- 书房 x[1.46,7.89] y[1.16,5.89]，门 x=4.6 + 西侧内门 y=3.5 ----
bookshelf(2.3, 5.715)
bookshelf(3.9, 5.715)
bookshelf(5.5, 5.715)
desk(7.54, 4.3, rz=-1.5708)
desk(7.54, 2.4, rz=-1.5708)
chair(7.0, 4.3, rz=1.5708)
chair(7.0, 2.4, rz=1.5708)
pillar(6.2, 3.95)
shelf_unit(2.2, 2.4, name='rack')            # 服务器/文件柜
plant(7.45, 1.55)

# ---- 卧室 1 x[-7.89,-3.66] y[-5.89,-1.16]，门 x=-5.6 ----
bed(-6.72, -3.5, rz=-1.5708)                  # 床头靠西墙
nightstand(-7.45, -2.3)
wardrobe(-4.6, -5.59)                         # 靠南墙
rug(-5.2, -3.4, 2.2, 1.6)
plant(-4.1, -1.65)

# ---- 卧室 2 x[-3.54,0.54] y[-5.89,-1.16]，门 x=-1.6 ----
bed(-0.85, -3.5, rz=1.5708)                   # 床头靠东墙
nightstand(0.2, -2.3)
wardrobe(-2.0, -5.59)                         # 靠南墙
desk(-3.19, -2.3, rz=1.5708)                  # 靠西墙
rug(-1.2, -3.4, 1.8, 1.4)

# ---- 卫生间 x[0.66,3.14] y[-5.89,-1.16]，门 x=1.9 ----
bathtub(2.62, -3.6)
toilet(0.95, -5.5, rz=3.1416)
sink_basin(0.95, -4.2, rz=1.5708)
washer(0.95, -2.75)

# ---- 储藏间 x[3.26,7.89] y[-5.89,-1.16]，门 x=5.6 ----
shelf_unit(4.1, -5.64)
shelf_unit(5.4, -5.64)
shelf_unit(6.7, -5.64)
shelf_unit(7.64, -3.4, rz=1.5708, name='shelf_east')
crate(5.0, -3.2, 0.9, 0.85)
crate(6.2, -2.6, 0.6, 0.6, '0.35 0.45 0.3')
crate(4.0, -2.1, 0.55, 0.55, '0.4 0.4 0.42')

# ---- 走廊 x[-7.89,7.89] y[-1.04,1.04] ----
console_table(-4.5, -0.88)                    # 靠南墙（窄，留 1.7 m 通道）
cart(2.6, 0.84, rz=3.1416)                    # 靠北墙
plant(7.4, 0.6)

# ================================================================
#                       机 器 人
# ================================================================
add('''<include>
  <name>tb3</name>
  <uri>file:///home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot/models/turtlebot3_waffle/model.sdf</uri>
  <!-- 出生点 z = 地板顶面 (0.012) + 3 mm 余量；抬高是为了让轮子一开始就别穿模，
       落下来之后轮子正好停在地板顶面上 -->
  <pose>0 0 0.015 0 0 0</pose>
</include>
</world>
</sdf>''')

# ============ 写出 ============
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w') as f:
    f.write('\n'.join(X))

n_tex = '\n'.join(X).count('albedo_map')
n_models = '\n'.join(X).count('<model name=')
print(f'✅ 已生成 {OUT}')
print(f'   尺寸 {XE-XW:.1f} x {YN-YS:.1f} m | 房间 {len(ROOMS)} 个 | 静态模型 {n_models} 个 | PBR 纹理 {n_tex} 处')
print(f'   门洞 {DOOR_W} m | 走廊净宽 {2*COR-T_IN:.2f} m | 层高 {H} m')
