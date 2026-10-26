"""相机高度 × 观察距离 的取景矩阵：找出"能同时框到冰箱门缝和把手"的最小改动。

冰箱可视特征（build_world.py）：
    下把手 z=0.75　门缝 z=1.15　上把手 z=1.45　总高 1.80
只有这些特征进画面，VLM 才可能把它和"一堵白墙/一扇门"区分开。

跑法：.venv/bin/python dev/check_camera_height.py
"""
import math

HFOV = 1.204
W, H = 320, 240
VFOV = 2 * math.atan(math.tan(HFOV / 2) * H / W)
FEATS = ((0.75, '下把手'), (1.15, '门缝'), (1.45, '上把手'))
FRIDGE_H = 1.80

HEIGHTS = (0.117, 0.30, 0.45, 0.60, 0.80)
# 相机到冰箱正面的距离
DISTS = (0.55, 1.00, 1.41, 2.11)


def band(cam_h, dist, hfov=HFOV):
    vfov = 2 * math.atan(math.tan(hfov / 2) * H / W)
    h_vis = 2 * dist * math.tan(vfov / 2)
    return cam_h - h_vis / 2, cam_h + h_vis / 2, h_vis


def main():
    print(f'HFOV {math.degrees(HFOV):.0f}°　VFOV {math.degrees(VFOV):.0f}°　{W}x{H}　'
          f'冰箱特征：' + '、'.join(f'{n} {z}m' for z, n in FEATS))
    print()
    header = '相机高'.ljust(8) + ''.join(f'离柜面 {d:.2f} m'.ljust(26) for d in DISTS)
    print(header)
    print('-' * len(header))
    for h in HEIGHTS:
        row = f'{h:.3f} m'.ljust(8)
        for d in DISTS:
            lo, hi, _ = band(h, d)
            got = [n for z, n in FEATS if lo <= z <= hi]
            tag = f'{len(got)}/3'
            row += f'[{lo:+.2f},{hi:+.2f}] {tag}'.ljust(26)
        print(row)

    print()
    print('判据：至少 2/3 特征进画面（门缝 + 一个把手）才可能被认出。')
    print('候选方案：')
    for h in (0.45, 0.60, 0.80):
        for d in (1.00, 1.41):
            lo, hi, _ = band(h, d)
            got = [n for z, n in FEATS if lo <= z <= hi]
            if len(got) >= 2:
                print(f'  ✅ 相机 {h:.2f} m + 离柜面 {d:.2f} m → 可见 {got}')
    print()
    print('注意：抬高相机只改传感器位姿，不影响雷达/地图/碰撞体；需要重启仿真才生效。')


if __name__ == '__main__':
    main()
