"""相机链路体检：帧率 / 分辨率 / rosbridge 上的实际带宽 / 能否解出图。

为什么先测这个：VLM 方案的第一道门槛不是模型，是**带宽**。
sensor_msgs/Image 里是 uint8[]，经 rosbridge 走 JSON 时会变 base64（体积 ×1.33）。
320x240x3 = 230 KB → base64 约 307 KB；10 Hz 就是 3 MB/s。
先量出来，才知道要不要降分辨率、转 JPEG、或者干脆不走 rosbridge。

跑法：
    .venv/bin/python dev/check_camera.py            # 默认收 4 秒
    .venv/bin/python dev/check_camera.py 8          # 收 8 秒
输出：
    终端统计 + logs/camera_<时间>.png（能直接用眼睛看的一帧）
"""
from __future__ import annotations

import base64
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from rosbridge_link import RosbridgeLink          # noqa: E402

TOPIC = '/camera/image_raw'
TYPE = 'sensor_msgs/Image'


def to_bytes(data) -> bytes:
    """rosbridge 对 uint8[] 的三种编码都接住：base64 串 / int 列表 / {'data': [...]}。"""
    if isinstance(data, bytes):
        return data
    if isinstance(data, list):
        return bytes(b & 0xFF for b in data)
    if isinstance(data, dict):
        return to_bytes(data.get('data', []))
    return base64.b64decode(data)


def save_png(msg: dict, rgb: bytes, out: Path) -> bool:
    """把一帧存成 PNG（需要 PIL；没有就存 PPM）。"""
    w, h = int(msg['width']), int(msg['height'])
    enc = msg.get('encoding', 'rgb8')
    try:
        from PIL import Image
        mode = 'RGB' if enc.startswith('rgb') else ('L' if enc in ('mono8',) else 'RGB')
        img = Image.frombytes(mode, (w, h), rgb)
        img.save(out)
        return True
    except Exception:
        ppm = out.with_suffix('.ppm')
        ppm.write_bytes(f'P6\n{w} {h}\n255\n'.encode() + rgb)
        print(f'（没装 PIL，已存 {ppm}）')
        return False


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    link = RosbridgeLink()
    frames = []

    def on_img(msg):
        frames.append((time.time(), msg))

    link.subscribe(TOPIC, TYPE, on_img)
    print(f'订阅 {TOPIC} {secs:.0f} s …')
    t0 = time.time()
    while time.time() - t0 < secs:
        time.sleep(0.05)
    link.close()

    print('=' * 70)
    if not frames:
        print(f'❌ {secs:.0f} s 内一帧都没收到')
        print('   检查：仿真在跑吗？ros_gz_bridge 起来了吗？rosbridge 连的是同一个 ROS_DOMAIN_ID 吗？')
        return 1

    n = len(frames)
    span = frames[-1][0] - frames[0][0]
    fps = (n - 1) / span if span > 0 else 0.0
    m = frames[-1][1]
    w, h = int(m['width']), int(m['height'])
    enc = m.get('encoding')
    step = int(m.get('step', w * 3))

    # 单帧在线路上有多大：取最后一条消息的原始 payload 表示
    raw_payload = m['data']
    if isinstance(raw_payload, str):
        wire = len(raw_payload)
        rep = 'base64 字符串'
    elif isinstance(raw_payload, list):
        wire = len(repr(raw_payload))
        rep = 'int 列表（JSON 数字数组，最贵）'
    else:
        wire = len(str(raw_payload))
        rep = 'dict/Buffer'
    pixel = len(to_bytes(raw_payload))

    print(f'收到 {n} 帧 / {span:.2f} s  →  {fps:.2f} fps')
    print(f'分辨率 {w}x{h}  encoding {enc}  step {step}')
    print(f'像素数据 {pixel / 1024:.1f} KB/帧')
    print(f'线路表示：{rep}，{wire / 1024:.1f} KB/帧')
    print(f'→ 实际带宽约 {wire * fps / 1024 / 1024:.2f} MB/s')
    print(f'→ 假设 10 Hz 满速，线路要 {wire * 10 / 1024 / 1024:.2f} MB/s')

    # 解一帧存下来，顺便验证字节序/行 padding
    rgb = to_bytes(m['data'])
    need = w * h * (3 if enc.startswith('rgb') or enc.startswith('bgr') else 1)
    print(f'解出 {len(rgb)} 字节（按 {enc} 应为 {need}）'
          f'  {"✅ 尺寸吻合" if len(rgb) >= need else "⚠️ 偏小，可能被截断"}')

    out = HERE / 'logs' / f"camera_{time.strftime('%H%M%S')}.png"
    out.parent.mkdir(exist_ok=True)
    ok = save_png(m, rgb, out)
    print(f'帧已存：{out if ok else out.with_suffix(".ppm")}')

    print()
    print('判读：')
    if isinstance(raw_payload, list):
        print('  ❌ rosbridge 把 uint8[] 发成了 JSON 数字数组 —— 体积约是 base64 的 3~4 倍，')
        print('     1 帧就能吃掉几百 KB，10 Hz 下 10+ MB/s。必须换方案（见下面建议）。')
    elif wire * fps / 1024 / 1024 > 2.0:
        print('  ⚠️ 带宽偏高，实时 VLM 会拖慢导航循环；建议降分辨率或转 JPEG。')
    else:
        print('  ✅ 带宽可接受。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
