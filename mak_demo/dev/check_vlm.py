"""VLM 链路体检：抓一帧 → 问本地 qwen3-vl → 打印回答与延迟。

为什么单独测：
  ① 确认相机帧能变成 LLM 的输入（base64 → /api/chat 的 images 字段）；
  ② 量出本机 VLM 的实际延迟（决定"到点后视觉确认"能否放进导航循环）；
  ③ 用真实回答说明当前相机几何**能**和**不能**看什么 —— 不要靠猜。

跑法：
    .venv/bin/python dev/check_vlm.py                 # 用当前视角抓帧，问默认问题
    .venv/bin/python dev/check_vlm.py "画面前方 1 米内有什么？"
    .venv/bin/python dev/check_vlm.py "能看到冰箱吗？" qwen3-vl:8b
"""
from __future__ import annotations

import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from rosbridge_link import RosbridgeLink                     # noqa: E402

OLLAMA = 'http://127.0.0.1:11434/api/chat'
DEFAULT_Q = ('这是家用机器人的前置相机画面（相机高约 0.12 m，正对前方）。'
             '请只描述你真正看到的：1) 画面里有哪些物体；2) 最近的东西离镜头大概多远；'
             '3) 能不能判断出这是一台冰箱？如果不能，说明为什么（比如只看到一片平面）。')


def to_bytes(data) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, list):
        return bytes(b & 0xFF for b in data)
    if isinstance(data, dict):
        return to_bytes(data.get('data', []))
    return base64.b64decode(data)


def grab_frame(timeout=8.0):
    """抓一帧，返回 PNG 字节和元信息。"""
    from PIL import Image

    link = RosbridgeLink()
    got = []

    def on_img(m):
        got.append(m)

    link.subscribe('/camera/image_raw', 'sensor_msgs/Image', on_img)
    t0 = time.time()
    while time.time() - t0 < timeout and not got:
        time.sleep(0.05)
    link.close()
    if not got:
        raise RuntimeError('抓帧超时：仿真/桥接在跑吗？')

    m = got[-1]
    w, h = int(m['width']), int(m['height'])
    rgb = to_bytes(m['data'])
    img = Image.frombytes('RGB', (w, h), rgb[:w * h * 3])
    buf = Path(HERE / 'logs' / f"vlm_{time.strftime('%H%M%S')}.png")
    buf.parent.mkdir(exist_ok=True)
    img.save(buf)
    return buf.read_bytes(), buf, (w, h)


def ask_vlm(png: bytes, question: str, model: str):
    b64 = base64.b64encode(png).decode()
    body = json.dumps({
        'model': model,
        'stream': False,
        'options': {'temperature': 0},
        'messages': [{'role': 'user', 'content': question, 'images': [b64]}],
    }).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={'Content-Type': 'application/json'})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=180) as r:
        data = json.loads(r.read())
    return data.get('message', {}).get('content', ''), time.time() - t0


def main():
    question = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_Q
    model = sys.argv[2] if len(sys.argv) > 2 else 'qwen3-vl:8b'

    png, path, size = grab_frame()
    print('=' * 70)
    print(f'抓帧：{path.name}  {size[0]}x{size[1]}  PNG {len(png) / 1024:.1f} KB'
          f'  （原图 base64 会更大，VLM 输入按 PNG 就够）')
    print(f'模型：{model}')
    print(f'问题：{question}')
    print('=' * 70)

    answer, el = ask_vlm(png, question, model)
    print(answer.strip())
    print()
    print(f'VLM 耗时 {el:.1f}s')
    print(f'判读：{"✅ 可用于到点确认" if el < 30 else "⚠️ 太慢，不能放进导航循环"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
