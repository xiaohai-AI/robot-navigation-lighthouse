#!/usr/bin/env python3
"""量化 SLAM 地图倾斜角（投影法，对二值图可靠）

原理：真实世界墙是轴对齐的。把占据栅格按角度 θ 旋转后，
      若墙正好落在行/列上，"按行求和 + 按列求和"会非常尖锐（少数行/列特别大）。
      用 sum(row_sum²) + sum(col_sum²) 当锐度指标，扫描 -45..45°，峰值即倾斜角。
      峰值接近 0° = 端正。

用法: python3 map_quality.py <map.pgm>

也作为库使用（tools/map_compare.py 就是这么用的，保证两处数字一致）：
    from map_quality import read_pgm, tilt_angle, size_stats
"""
import sys

import numpy as np

RES = 0.05          # 地图分辨率（m/格），与 config/maps/*.yaml 一致


def read_pgm(path):
    """读 P5 二进制 PGM，返回 (arr[h,w] uint8, w, h)。"""
    with open(path, 'rb') as f:
        raw = f.read()
    if not raw.startswith(b'P5'):
        raise SystemExit('需要 P5 二进制 PGM')
    # 跳过 magic/注释，读 w h maxval
    tok, i = [], 2
    while len(tok) < 3:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            i = raw.index(b'\n', i) + 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        tok.append(int(raw[i:j]))
        i = j
    w, h, _ = tok
    i += 1
    return np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w), w, h


def occupancy(arr):
    """PGM 值 → (free, occ) 布尔掩码。250 以上=空闲，50 以下=墙，其余=未知。"""
    return arr >= 250, arr <= 50


def tilt_angle(occ, w, h):
    """用旋转投影锐度找倾斜角，返回 (best_deg, angles, scores)。

    occ 为布尔占据掩码。1° 步长扫 -45..45°，峰值附近做抛物线插值。
    """
    occ_f = occ.astype(np.float32)
    cy, cx = h / 2.0, w / 2.0
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    dy, dx = yy - cy, xx - cx

    def sharpness(theta_deg):
        t = np.deg2rad(theta_deg)
        c, s = np.cos(t), np.sin(t)
        xr = (c * dx - s * dy + cx).astype(np.int32)
        yr = (s * dx + c * dy + cy).astype(np.int32)
        ok = (xr >= 0) & (xr < w) & (yr >= 0) & (yr < h)
        rot = np.zeros((h, w), np.float32)
        rot[yr[ok], xr[ok]] = occ_f[yy[ok].astype(np.int32), xx[ok].astype(np.int32)]
        rs = rot.sum(1)
        cs = rot.sum(0)
        return float((rs ** 2).sum() + (cs ** 2).sum())

    angles = np.arange(-45, 45.5, 1.0)
    scores = np.array([sharpness(a) for a in angles])
    k = int(scores.argmax())
    best = float(angles[k])
    if 0 < k < len(angles) - 1:
        y0, y1, y2 = scores[k - 1], scores[k], scores[k + 1]
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) > 1e-9:
            best += 0.5 * (y0 - y2) / denom
    return best, angles, scores


def size_stats(free, occ, w, h):
    """覆盖率与已建范围（米）。"""
    total = w * h
    ys, xs = np.nonzero(free | occ)
    return {
        'w_m': w * RES,
        'h_m': h * RES,
        'free_pct': 100.0 * free.sum() / total,
        'wall_pct': 100.0 * occ.sum() / total,
        'unk_pct': 100.0 * (~free & ~occ).sum() / total,
        'ext_w': (xs.max() - xs.min() + 1) * RES,
        'ext_h': (ys.max() - ys.min() + 1) * RES,
    }


def main(path):
    img, w, h = read_pgm(path)
    free, occ = occupancy(img)
    st = size_stats(free, occ, w, h)
    unk = ~free & ~occ

    print(f'尺寸     : {w} x {h} 格  ({st["w_m"]:.1f} x {st["h_m"]:.1f} m)')
    print(f'空闲 {st["free_pct"]:.1f}%  墙 {st["wall_pct"]:.1f}%  '
          f'未知 {100 * unk.sum() / (w * h):.1f}%')
    print(f'已建范围 : {st["ext_w"]:.1f} x {st["ext_h"]:.1f} m   （房子 16.0 x 12.0 m）')

    best, angles, scores = tilt_angle(occ, w, h)
    print(f'\n倾斜角   : {best:+.1f}°   ' +
          ('✅ 端正（< 5°）' if abs(best) < 5 else '❌ 有明显倾斜'))
    print('（正数 = 地图逆时针偏了；负数 = 顺时针偏了）')
    print('\n锐度曲线（每 5°）：')
    for a, sc in zip(angles[::5], scores[::5]):
        bar = '#' * int(40 * (sc - scores.min()) / (scores.max() - scores.min() + 1e-9))
        print(f'  {a:+5.0f}° {bar}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('用法: python3 map_quality.py <map.pgm>')
    main(sys.argv[1])
