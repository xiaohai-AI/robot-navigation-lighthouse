#!/usr/bin/python3
"""逐像素时域抖动图：抓一串连续帧，算每个像素在时间上的抖动强度。

为什么：单帧看不出来，而"跳动的黑影"本质是**同一位置的像素在帧间反复变化**。
把每个像素的时间标准差画成一张图，黑影自己会亮出来。

输出：
  · 控制台：整幅平均抖动、最抖的 6 个 16x12 网格块（位置 + 强度 + 平均亮度）
  · logs/flicker_map_*.png   抖动热力图（亮 = 抖）
  · logs/flicker_pair_*.png  差异最大的两帧并排（左 A / 右 B）

跑法（仿真 + rosbridge 起来后）：
    .venv/bin/python dev/probe_flicker_map.py          # 抓 20 帧
    .venv/bin/python dev/probe_flicker_map.py 30       # 抓 30 帧
"""
from __future__ import annotations

import base64
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from rosbridge_link import RosbridgeLink          # noqa: E402

TOPIC = '/camera/image_raw'
TYPE = 'sensor_msgs/Image'
LOG_DIR = HERE.parent / 'logs'


def to_bytes(data) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, list):
        return bytes(b & 0xFF for b in data)
    if isinstance(data, dict):
        return to_bytes(data.get('data', []))
    return base64.b64decode(data)


def unpack(msg: dict):
    """→ (w, h, luma: list[int] 长度 w*h)"""
    w, h = int(msg['width']), int(msg['height'])
    step = int(msg.get('step') or w * 3)
    rgb = to_bytes(msg['data'])
    luma = [0] * (w * h)
    for y in range(h):
        row = y * step
        base = y * w
        for x in range(w):
            i = row + x * 3
            luma[base + x] = (rgb[i] * 299 + rgb[i + 1] * 587 + rgb[i + 2] * 114) // 1000
    return w, h, luma


def main() -> int:
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    try:
        link = RosbridgeLink()
    except Exception as e:
        print(f'❌ 连不上 rosbridge：{e}')
        return 3
    frames = []
    link.subscribe(TOPIC, TYPE, lambda m: frames.append((time.time(), m)))
    print(f'订阅 {TOPIC}，收集 {want} 帧 …')
    t0 = time.time()
    while len(frames) < want and time.time() - t0 < 25.0:
        time.sleep(0.03)
    link.close()
    if len(frames) < 3:
        print(f'❌ 只收到 {len(frames)} 帧（仿真在跑吗？相机话题在吗？）')
        return 1

    w, h, _ = unpack(frames[0][1])
    n = len(frames)
    px = [unpack(m)[2] for _, m in frames]
    span = frames[-1][0] - frames[0][0]
    print(f'收到 {n} 帧 / {span:.2f} s = {n/max(span,1e-6):.1f} fps，分辨率 {w}x{h}')

    # 每像素时间均值与标准差
    means = [0.0] * (w * h)
    for f in px:
        for i, v in enumerate(f):
            means[i] += v
    means = [m / n for m in means]

    stds = [0.0] * (w * h)
    for f in px:
        for i, v in enumerate(f):
            d = v - means[i]
            stds[i] += d * d
    stds = [(s / n) ** 0.5 for s in stds]

    overall = sum(stds) / len(stds)
    print(f'整幅平均抖动（时间 std）= {overall:.2f} 灰阶')

    # 16x12 网格汇总
    GX, GY = 16, 12
    cells = []
    for gy in range(GY):
        for gx in range(GX):
            x0, x1 = gx * w // GX, (gx + 1) * w // GX
            y0, y1 = gy * h // GY, (gy + 1) * h // GY
            s = m = 0.0
            cnt = 0
            for y in range(y0, y1):
                for x in range(x0, x1):
                    i = y * w + x
                    s += stds[i]
                    m += means[i]
                    cnt += 1
            cells.append((s / cnt, m / cnt, gx, gy, x0, y0, x1, y1))
    cells.sort(reverse=True)
    print()
    print('最抖的 6 个网格块（抖动 / 该块平均亮度 / 位置）:')
    for s, m, gx, gy, x0, y0, x1, y1 in cells[:6]:
        print(f'  抖动 {s:6.2f}  亮度 {m:6.1f}  格({gx:>2},{gy:>2}) 像素 x{x0}-{x1} y{y0}-{y1}')

    # 找差异最大的相邻两帧
    best = (0, 0, 0.0)
    for k in range(n - 1):
        a, b = px[k], px[k + 1]
        d = sum(abs(a[i] - b[i]) for i in range(0, w * h, 7)) / (w * h / 7)
        if d > best[2]:
            best = (k, k + 1, d)
    print(f'\n相邻帧最大平均差：帧{best[0]}→帧{best[1]} = {best[2]:.2f} 灰阶')

    # 存图
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime('%H%M%S')
    try:
        from PIL import Image

        # 热力图
        mx = max(stds) or 1.0
        heat = bytearray()
        for i in range(w * h):
            v = int(255 * min(1.0, stds[i] / mx))
            heat += bytes((v, v, v))
        Image.frombytes('RGB', (w, h), bytes(heat)).save(LOG_DIR / f'flicker_map_{ts}.png')

        # 并排两帧
        def frame_rgb(msg):
            rgb = to_bytes(msg['data'])
            return rgb

        a = frame_rgb(frames[best[0]][1])
        b = frame_rgb(frames[best[1]][1])
        pair = bytearray()
        for y in range(h):
            row = y * w * 3
            pair += a[row:row + w * 3]
            pair += b[row:row + w * 3]
        Image.frombytes('RGB', (w * 2, h), bytes(pair)).save(LOG_DIR / f'flicker_pair_{ts}.png')
        print(f'已存：{LOG_DIR / f"flicker_map_{ts}.png"}')
        print(f'已存：{LOG_DIR / f"flicker_pair_{ts}.png"}')
    except Exception as e:
        print(f'（存图失败：{e}）')

    print()
    if overall > 4.0:
        print('❌ 整幅抖动明显 —— 画面在逐帧变化（真·抖动）')
        return 1
    print('✅ 帧间稳定，没有整幅抖动')
    return 0


if __name__ == '__main__':
    sys.exit(main())
