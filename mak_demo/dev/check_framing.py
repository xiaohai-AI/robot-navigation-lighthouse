"""相机取景几何：在不同距离/FOV 下，画面能框住多高的一段。

用来判断"到点能不能认出物体"到底是距离问题还是 FOV 问题 —— 纯计算，不跑仿真。

跑法：.venv/bin/python dev/check_framing.py
"""
import math

# 当前相机（models/lighthouse/model.sdf）：camera_link z=0.70，传感器 pose 再 +0.013
CAM_H = 0.70 + 0.013
HFOV = 1.204                   # rad（69°）
W, H = 320, 240
VFOV = 2 * math.atan(math.tan(HFOV / 2) * H / W)

# 冰箱（build_world.py: fridge()）
FRIDGE_W = 0.78
FRIDGE_H = 1.80
HANDLES = (0.75, 1.45)         # 两个把手高度
SPLIT_Z = 1.15                 # 门缝高度

ROWS = [
    ('到点站姿 0.94 m（离柜面 0.55）', 0.55),
    ('退到 1.5 m（离柜面 1.11）', 1.11),
    ('退到 1.8 m（离柜面 1.41）', 1.41),
    ('退到 2.5 m（离柜面 2.11）', 2.11),
]


def framing(dist, hfov=HFOV):
    vfov = 2 * math.atan(math.tan(hfov / 2) * H / W)
    w_vis = 2 * dist * math.tan(hfov / 2)
    h_vis = 2 * dist * math.tan(vfov / 2)
    return w_vis, h_vis, (CAM_H - h_vis / 2, CAM_H + h_vis / 2)


def main():
    print(f'相机高度 {CAM_H:.3f} m　HFOV {math.degrees(HFOV):.1f}°　'
          f'VFOV {math.degrees(VFOV):.1f}°　{W}x{H}')
    print(f'冰箱 {FRIDGE_W}x{FRIDGE_H} m，把手 z={HANDLES[0]}/{HANDLES[1]}，门缝 z={SPLIT_Z}')
    print()
    print(f'{"场景":32s} {"横宽":>6s} {"竖高":>6s} {"可见高度带":>16s}  特征')
    for label, d in ROWS:
        w, h, (lo, hi) = framing(d)
        feats = []
        for z, name in ((HANDLES[0], '下手柄'), (SPLIT_Z, '门缝'), (HANDLES[1], '上手柄')):
            if lo <= z <= hi:
                feats.append(name)
        print(f'{label:32s} {w:5.2f}m {h:5.2f}m '
              f'[{lo:+.2f}, {hi:+.2f}] m  {"、".join(feats) or "无（只看到面板下部）"}')

    print()
    print('假设把 HFOV 放宽到 100°/120°（到点 0.55 m 处）：')
    for deg in (100, 120):
        w, h, (lo, hi) = framing(0.55, math.radians(deg))
        feats = [n for z, n in ((HANDLES[0], '下手柄'), (SPLIT_Z, '门缝'),
                                (HANDLES[1], '上手柄')) if lo <= z <= hi]
        print(f'  HFOV {deg}°：横宽 {w:.2f} m（冰箱 {FRIDGE_W} m '
              f'{"← 能看到左右边缘" if w > FRIDGE_W * 1.1 else "← 仍被填满"}），'
              f'可见带 [{lo:+.2f}, {hi:+.2f}] m，特征：{"、".join(feats) or "无"}')

    print()
    print('结论：')
    w094, h094, (lo094, hi094) = framing(0.55)
    w141, h141, (lo141, hi141) = framing(1.41)
    print(f'  · 到点 0.94 m：只看到 {w094:.2f}×{h094:.2f} m 的一小块面板 → 认不出')
    print(f'  · 退到 1.8 m：看到 {w141:.2f}×{h141:.2f} m，高度带覆盖到 {hi141:.2f} m '
          f'{"（含门缝/把手）" if hi141 >= SPLIT_Z else ""}')


if __name__ == '__main__':
    main()
