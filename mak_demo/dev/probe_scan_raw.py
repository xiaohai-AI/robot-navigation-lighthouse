"""诊断：/scan 原始数据到底长什么样 + VLM 这次为什么给出空答案。

两个怀疑：
  ① 雷达 0.13 m 是真的（前方贴着柱子/墙垛），但也有可能是解码/角度映射错了
  ② 视觉返回了「空答案」——提示词/解析出问题，或者模型真的没按格式答

跑法：.venv/bin/python dev/probe_scan_raw.py
"""
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import passability                                # noqa: E402
import vision                                     # noqa: E402
from rosbridge_link import RosbridgeLink          # noqa: E402

link = RosbridgeLink()

# ---------- ① 拿一帧原始 /scan ----------
got = []
link.subscribe('/scan', 'sensor_msgs/LaserScan', got.append)
t0 = time.time()
while time.time() - t0 < 5 and not got:
    time.sleep(0.02)

print('=' * 74)
if not got:
    print('❌ 5 s 内没收到 /scan')
    link.close()
    sys.exit(1)

m = got[-1]
print('【原始消息字段】')
for k in ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max',
          'scan_time', 'time_increment'):
    print(f'  {k:16s} = {m.get(k)}')
r_raw = m.get('ranges')
print(f'  ranges            : {type(r_raw).__name__}, len={len(r_raw) if hasattr(r_raw, "__len__") else "?"}')

rs = passability.to_floats(r_raw)
print(f'\n【解码后】{len(rs)} 个点')
print(f'  前 6 个: {[round(v, 3) for v in rs[:6]]}')
finite = [v for v in rs if v == v and abs(v) != float('inf') and v > 0]
print(f'  有效点 {len(finite)} 个，最小 {min(finite):.3f}，最大 {max(finite):.3f}')
near = sorted((v, i) for i, v in enumerate(rs)
              if v == v and abs(v) != float('inf') and 0 < v < 0.5)
print(f'  < 0.50 m 的点 {len(near)} 个：')
inc = float(m.get('angle_increment', 0)) or (2 * math.pi / len(rs))
a0 = float(m.get('angle_min', -math.pi))
for v, i in near[:8]:
    print(f'     idx {i:4d}  r={v:.3f} m  角度={(a0 + i * inc) * 180 / math.pi:+7.1f}°'
          f'  （车体系：0=正前，正=左）')

meas = passability.forward_clearance(m)
print(f'\n【前向扇区】{json.dumps(meas, ensure_ascii=False)}')
print(f'  {passability.describe(meas)}')

# ---------- ② 当前位姿 ----------
pose = []
link.subscribe('/agent/nav_status', 'std_msgs/String', pose.append)
t0 = time.time()
while time.time() - t0 < 3 and not pose:
    time.sleep(0.05)
if pose:
    print(f'\n【当前位姿】{pose[-1].get("data")[:200]}')

# ---------- ③ VLM 原始输出 ----------
print('\n' + '=' * 74)
print('【视觉：原始输出】')
png, path = vision.capture_frame()
r = vision.ask_vlm_detailed(png, vision.build_pass_prompt())
ans, why = vision.parse_answer(r['content'])
print(f'  帧 {path.name}  {len(png) / 1024:.1f} KB   耗时 {r["elapsed"]:.1f}s')
print(f'  done_reason={r["done_reason"]}  eval={r["eval_count"]}  '
      f'thinking={len(r["thinking"])} 字  truncated={r.get("truncated")}  '
      f'salvaged={r.get("salvaged", False)}')
print(f'  原始返回：{r["content"]!r}')
print(f'  解析结果：答案={ans!r}  依据={why!r}')

link.close()
