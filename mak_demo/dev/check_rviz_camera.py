"""RViz Camera display 的三个前置条件体检。

RViz 的 Camera 显示要同时拿到三样东西，任何一样不对就报错：
    ① Image 消息（/camera/image_raw）
    ② CameraInfo 消息（/camera/camera_info），且 width/height 与 Image 一致
    ③ 图像 header.frame_id 在 TF 树里能连到固定坐标系（map）

跑法：.venv/bin/python dev/check_rviz_camera.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from rosbridge_link import RosbridgeLink          # noqa: E402

img = {}
info = {}
link = RosbridgeLink()


def on_img(m):
    img.update(m)


def on_info(m):
    info.update(m)


link.subscribe('/camera/image_raw', 'sensor_msgs/Image', on_img)
link.subscribe('/camera/camera_info', 'sensor_msgs/CameraInfo', on_info)
t0 = time.time()
while time.time() - t0 < 8.0 and not (img and info):
    time.sleep(0.05)
link.close()

print('=' * 72)
if not img:
    print('❌ 8 s 内没收到 /camera/image_raw')
else:
    h = img.get('header', {})
    print(f"Image      {img.get('width')}x{img.get('height')}  "
          f"encoding={img.get('encoding')}  step={img.get('step')}")
    print(f"            frame_id = {h.get('frame_id')!r}")
    stamp = h.get('stamp', {})
    print(f"            stamp = {stamp.get('sec')}.{stamp.get('nanosec', 0):09d}")
    print(f"            seq/其他 = is_bigendian={img.get('is_bigendian')}")

if not info:
    print('❌ 8 s 内没收到 /camera/camera_info  ← RViz Camera display 会因此报错')
else:
    h = info.get('header', {})
    print(f"CameraInfo {info.get('width')}x{info.get('height')}  "
          f"distortion={info.get('distortion_model')}")
    print(f"            frame_id = {h.get('frame_id')!r}")
    K = info.get('k') or []
    print(f"            K = {[round(v, 1) for v in K]}")
    fov_x = None
    if len(K) == 9 and K[0]:
        import math
        fov_x = 2 * math.degrees(math.atan(info['width'] / (2 * K[0])))
        print(f"            由 K[0]={K[0]:.1f} 反推水平 FOV = {fov_x:.1f}°")

print('=' * 72)
if img and info:
    ok = (img.get('width') == info.get('width')
          and img.get('height') == info.get('height'))
    print(f"尺寸一致性：Image {img.get('width')}x{img.get('height')} vs "
          f"CameraInfo {info.get('width')}x{info.get('height')} → "
          f"{'✅ 一致' if ok else '❌ 不一致（RViz 会报错）'}")
    fi, fc = img.get('header', {}).get('frame_id'), info.get('header', {}).get('frame_id')
    print(f"frame_id 一致性：Image {fi!r} vs CameraInfo {fc!r} → "
          f"{'✅ 相同' if fi == fc else '❌ 不同'}")
