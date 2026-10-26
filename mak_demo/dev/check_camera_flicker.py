#!/usr/bin/python3
"""量化"相机画面里的跳动黑影"：连续抓 N 帧，统计每帧暗像素占比的抖动。

为什么需要这个：黑影是"自家外壳卡在 near clip 平面上"导致的逐帧裁剪抖动，
肉眼能看见但很难说清严重程度。这个脚本给两个数字：
  · dark%  每帧里亮度 < 40 的像素占比（自家机身/黑块会显著抬高它）
  · std    这些 dark% 的帧间标准差 —— 就是"跳"的量化指标
开阔地面看出去，正常应该 dark% 接近 0、std < 1%。

跑法（仿真 + rosbridge 起来后）：
    .venv/bin/python dev/check_camera_flicker.py            # 默认 16 帧
    .venv/bin/python dev/check_camera_flicker.py 24         # 抓 24 帧

判读：
    std > 2%    → 仍在跳，另有遮挡源（或相机正对着深色家具，换个朝向再测）
    dark% > 20% → 画面被实心黑块挡住（贴脸遮挡，不是抖动）
    两者都小    → 修好了

2026-10-23：修 camera_visual/sensor 几何前，传感器在自己外壳内部，典型表现就是
dark% 在 0~15% 之间来回跳。
"""
from __future__ import annotations

import base64
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from rosbridge_link import RosbridgeLink          # noqa: E402

TOPIC = '/camera/image_raw'
TYPE = 'sensor_msgs/Image'
LOG_DIR = HERE.parent / 'logs'
DARK = 40          # 亮度阈值 0~255


def to_bytes(data) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, list):
        return bytes(b & 0xFF for b in data)
    if isinstance(data, dict):
        return to_bytes(data.get('data', []))
    return base64.b64decode(data)


def frame_stats(msg: dict) -> tuple[float, tuple[int, int, int, int], bytes]:
    """返回 (暗像素占比%, 暗区包围盒, rgb字节)"""
    w, h = int(msg['width']), int(msg['height'])
    step = int(msg.get('step') or w * 3)
    enc = msg.get('encoding', 'rgb8')
    rgb = to_bytes(msg['data'])
    if enc.startswith('bgr'):
        # BGR → RGB（只对亮度统计影响很小，但存图需要正确）
        ba = bytearray(rgb)
        ba[0::3], ba[2::3] = rgb[2::3], rgb[0::3]
        rgb = bytes(ba)

    dark = 0
    x0, y0, x1, y1 = w, h, -1, -1
    for y in range(h):
        row = y * step
        for x in range(w):
            i = row + x * 3
            if i + 2 >= len(rgb):
                continue
            luma = (rgb[i] * 299 + rgb[i + 1] * 587 + rgb[i + 2] * 114) // 1000
            if luma < DARK:
                dark += 1
                if x < x0:
                    x0 = x
                if y < y0:
                    y0 = y
                if x > x1:
                    x1 = x
                if y > y1:
                    y1 = y
    pct = 100.0 * dark / max(1, w * h)
    box = (x0, y0, x1, y1) if x1 >= 0 else (-1, -1, -1, -1)
    return pct, box, rgb


def save_png(msg: dict, rgb: bytes, out: Path) -> None:
    w, h = int(msg['width']), int(msg['height'])
    try:
        from PIL import Image
        Image.frombytes('RGB', (w, h), rgb).save(out)
        print(f'   最差帧已存：{out}')
    except Exception as e:
        print(f'   （存图失败：{e}）')


def selftest() -> int:
    """离线自检：造一张 320x240 灰底 + 左下 40x40 黑块的假帧，验证统计算法。"""
    w, h = 320, 240
    px = bytearray()
    for y in range(h):
        for x in range(w):
            black = (x < 40 and y > h - 41)
            v = 0 if black else 180
            px += bytes((v, v, v))
    msg = {'width': w, 'height': h, 'step': w * 3, 'encoding': 'rgb8',
           'data': base64.b64encode(bytes(px)).decode()}
    pct, box, rgb = frame_stats(msg)
    expect = 100.0 * 40 * 40 / (w * h)
    ok_pct = abs(pct - expect) < 0.05
    ok_box = box == (0, h - 40, 39, h - 1)
    ok_len = len(rgb) == w * h * 3
    print(f'自检 320x240 黑块 40x40：dark%={pct:.3f}（期望 {expect:.3f}）bbox={box}')
    print(f'  占比 {"✅" if ok_pct else "❌"}  包围盒 {"✅" if ok_box else "❌"}  解包长度 {"✅" if ok_len else "❌"}')
    return 0 if (ok_pct and ok_box and ok_len) else 1


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        return selftest()

    want = int(sys.argv[1]) if len(sys.argv) > 1 else 16
    try:
        link = RosbridgeLink()
    except Exception as e:
        print(f'❌ 连不上 rosbridge：{e}')
        print('   仿真和 rosbridge 起来后再跑；纯算法自检用：dev/check_camera_flicker.py --selftest')
        return 3
    frames = []
    link.subscribe(TOPIC, TYPE, lambda m: frames.append(m))

    print(f'订阅 {TOPIC}，收集 {want} 帧 …')
    t0 = time.time()
    while len(frames) < want and time.time() - t0 < 20.0:
        time.sleep(0.05)
    link.close()

    if not frames:
        print('❌ 一帧都没收到。检查：仿真在跑？ros_gz_bridge 起来？rosbridge 同一 DOMAIN_ID？')
        return 1
    if len(frames) < 3:
        print(f'⚠️ 只收到 {len(frames)} 帧，统计意义有限')

    print('=' * 66)
    print(f'{"帧":>3}  {"dark%":>7}  {"暗区包围盒":>20}   {"耗时":>6}')
    pcts = []
    worst: None | tuple[float, dict, bytes] = None
    for i, m in enumerate(frames):
        pct, box, rgb = frame_stats(m)
        pcts.append(pct)
        if worst is None or pct > worst[0]:
            worst = (pct, m, rgb)
        print(f'{i:>3}  {pct:>6.2f}%  {str(box):>20}')

    std = statistics.pstdev(pcts) if len(pcts) > 1 else 0.0
    rng = max(pcts) - min(pcts)
    print('-' * 66)
    print(f'均值 dark%  = {statistics.mean(pcts):.2f}%')
    print(f'帧间 std    = {std:.2f}%   （抖动指标）')
    print(f'极差        = {rng:.2f}%')
    print('-' * 66)

    if std > 2.0 or rng > 5.0:
        print('❌ 仍在跳：还有一种逐帧变化的遮挡（换个朝向再测一次排除深色家具）')
        verdict = 1
    elif statistics.mean(pcts) > 20.0:
        print('⚠️ 不跳但被挡：画面里有实心暗块，检查是否有几何贴在镜头前')
        verdict = 2
    else:
        print('✅ 没有抖动黑块')
        verdict = 0

    assert worst is not None
    worst_pct, worst_msg, worst_rgb = worst
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    out = LOG_DIR / f'flicker_worst_{time.strftime("%H%M%S")}.png'
    save_png(worst_msg, worst_rgb, out)
    return verdict


if __name__ == '__main__':
    sys.exit(main())
