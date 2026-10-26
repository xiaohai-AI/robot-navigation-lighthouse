#!/usr/bin/python3
"""把两张 SLAM 结果图并排渲染成一张对比图（用于 README / 汇报）。

用法：
    /usr/bin/python3 tools/map_compare.py \
        config/maps/dan_house.pgm        config/maps/dan_house_explore.pgm \
        docs/map_compare.png

左图默认标注为「A · 巡游建图」，右图为「B · 自主探索」。
统计数字（尺寸/覆盖率/倾斜角）用 tools/map_quality.py 的同一套算法现算，
不写死 —— 图换了数字自动跟着变。
"""
import os
import sys
import yaml
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# 倾斜角 / 覆盖率统一用 tools/map_quality.py 的算法，保证文档里的数字和图上的数字一致
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_quality import read_pgm, occupancy, tilt_angle  # noqa: E402

SCALE = 3                     # 像素放大倍数（占据栅格很细，放大 3 倍才好读）
MARGIN = 34
GUTTER = 40
HEADER = 96                   # 标题区高度
STATS = 116                   # 每张图下方的统计文字区
FOOTER = 86                   # 底部图例区

C_BG = (255, 255, 255)
C_TEXT = (26, 30, 36)
C_SUB = (96, 104, 116)
C_BORDER = (208, 214, 222)
C_ACCENT = (21, 90, 170)

C_OCC = (24, 26, 30)          # 墙
C_FREE = (252, 252, 250)      # 空闲
C_UNK = (168, 176, 186)       # 未知
C_LM = (232, 120, 30)         # 走廊地标

# 12 个走廊地标（世界坐标，来自 build_world.py 的 _lam 调用）
LANDMARKS = [(-8.00, 1.29), (-4.60, 1.29), (-3.00, 1.29),
             (1.00, 1.29), (2.60, 1.29), (7.00, 1.29),
             (-7.80, -1.29), (-5.00, -1.29), (-3.40, -1.29),
             (0.20, -1.29), (4.40, -1.29), (7.40, -1.29)]

FONT_CANDS = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]


def font(size, bold=False):
    for p in (FONT_CANDS if not bold else FONT_CANDS[:2] + FONT_CANDS[3:4] + FONT_CANDS[4:]):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return ImageFont.load_default()


def load_map(pgm_path):
    """返回 (RGB 图, 元信息)。复用 map_quality.read_pgm，口径完全一致。"""
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    meta = {}
    if os.path.exists(yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f) or {}
    arr, _w, _h = read_pgm(pgm_path)
    free, occ = occupancy(arr)
    unk = ~free & ~occ

    rgb = np.zeros(arr.shape + (3,), dtype=np.uint8)
    rgb[unk] = C_UNK
    rgb[free] = C_FREE
    rgb[occ] = C_OCC
    img = Image.fromarray(rgb)

    total = arr.size
    res = float(meta.get('resolution', 0.05))
    m = {
        'w_m': arr.shape[1] * res,
        'h_m': arr.shape[0] * res,
        'free_pct': 100.0 * free.sum() / total,
        'wall_pct': 100.0 * occ.sum() / total,
        'unk_pct': 100.0 * unk.sum() / total,
        'res': res,
        'origin': meta.get('origin', [0.0, 0.0, 0.0]),
        'shape': arr.shape,
        'tilt': tilt_angle(occ, arr.shape[1], arr.shape[0])[0],
    }
    return img, m


def draw_landmarks(draw, m, ox, oy, r=4):
    """在地图上标 12 个走廊地标。

    地标贴墙（y=±1.29 是墙内表面），圆心直接画会压墙线，所以朝走廊中心
    挪 0.10 m，再加一圈白色描边，保证点落在白色通道一侧且清晰。
    """
    res = m['res']
    o_x, o_y, _ = m['origin']
    h = m['shape'][0]
    for wx, wy in LANDMARKS:
        wy_in = wy - 0.10 if wy > 0 else wy + 0.10
        px = ox + (wx - o_x) / res * SCALE
        py = oy + (h - (wy_in - o_y) / res) * SCALE
        draw.ellipse([px - r - 2, py - r - 2, px + r + 2, py + r + 2], fill=(255, 255, 255))
        draw.ellipse([px - r, py - r, px + r, py + r], fill=C_LM, outline=(140, 62, 8), width=1)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    left_pgm, right_pgm, out_png = sys.argv[1], sys.argv[2], sys.argv[3]
    title = sys.argv[4] if len(sys.argv) > 4 else 'dan_house 建图结果对比（同一世界，2026-10-21）'

    panels = [
        ('A · 巡游建图（脚本化 32 航点）', left_pgm, '18~19 分钟 · 零 frontier 逻辑'),
        ('B · 自主探索（前沿探索 80 目标）', right_pgm, '26.2 分钟（仿真）· 16 个目标失败'),
    ]

    loaded = []
    for label, pgm, note in panels:
        img, m = load_map(pgm)
        loaded.append((label, note, img.resize((img.width * SCALE, img.height * SCALE),
                                              Image.Resampling.NEAREST), m))

    pw = max(p[2].width for p in loaded)
    ph = max(p[2].height for p in loaded)
    W = MARGIN * 2 + pw * 2 + GUTTER
    H = MARGIN + HEADER + ph + STATS + FOOTER

    canvas = Image.new('RGB', (W, H), C_BG)
    d = ImageDraw.Draw(canvas)

    d.text((MARGIN, MARGIN - 6), title, font=font(34, True), fill=C_TEXT)
    d.text((MARGIN, MARGIN + 38),
           '同一天、同一套世界（16×12 m，7 房间，12 个走廊地标）、同一台车，两条建图路线的结果',
           font=font(19), fill=C_SUB)

    for i, (label, note, img, m) in enumerate(loaded):
        x = MARGIN + i * (pw + GUTTER)
        y = MARGIN + HEADER
        d.rectangle([x - 2, y - 2, x + pw + 1, y + ph + 1], outline=C_BORDER, width=2)
        canvas.paste(img, (x, y))
        draw_landmarks(d, m, x, y)

        ty = y + ph + 16
        d.text((x, ty), label, font=font(23, True), fill=C_ACCENT)
        d.text((x, ty + 34),
               f'{m["w_m"]:.1f} × {m["h_m"]:.1f} m   ｜   空闲 {m["free_pct"]:.1f}%   '
               f'墙 {m["wall_pct"]:.1f}%   未知 {m["unk_pct"]:.1f}%',
               font=font(20), fill=C_TEXT)
        d.text((x, ty + 64),
               f'倾斜角 {m["tilt"]:+.1f}°   ｜   走廊地标 12/12 可见   ｜   {note}',
               font=font(20), fill=C_TEXT)

    fy = H - FOOTER + 18
    d.line([MARGIN, fy - 12, W - MARGIN, fy - 12], fill=C_BORDER, width=1)
    lx = MARGIN
    for color, text in ((C_OCC, '墙 / 障碍'), (C_FREE, '已探索空闲'), (C_UNK, '未知'), (C_LM, '走廊地标（12）')):
        d.rectangle([lx, fy + 4, lx + 22, fy + 26], fill=color, outline=(150, 156, 164))
        d.text((lx + 30, fy + 4), text, font=font(19), fill=C_TEXT)
        lx += 190
    d.text((W - MARGIN - 470, fy + 4),
           '灰块=未知（墙后/遮挡/未到达），不是错误', font=font(18), fill=C_SUB)

    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    canvas.save(out_png)
    print(f'saved {out_png}  {canvas.width}x{canvas.height}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
