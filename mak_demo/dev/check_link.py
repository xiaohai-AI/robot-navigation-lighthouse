"""步骤 1 验收：agent 侧能不能通过 rosbridge 看见车。

检查三件事：
    1. rosbridge 连得上，能列出话题
    2. /agent/pose 有数据（说明 agent_bridge 活着、TF map->base_link 通）
    3. /agent/nav_status 有数据（状态机在线，当前应为 idle）

跑法（mak_demo 目录下）：
    .venv/bin/python dev/check_link.py
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rosbridge_link import RosbridgeError, RosbridgeLink  # noqa: E402

POSE_TYPE = 'geometry_msgs/PoseStamped'
STATUS_TYPE = 'std_msgs/String'

try:
    link = RosbridgeLink()
except RosbridgeError as e:
    print(f'❌ {e}')
    sys.exit(1)

print('✅ 1) rosbridge 已连接 ws://127.0.0.1:9090')

topics = link.topics()
print(f'✅ 2) 话题数 {len(topics)}，其中 agent 相关:',
      [t for t in topics if t.startswith('/agent')] or '（一个都没有 → agent_bridge 没起）')

poses = []
statuses = []


def on_pose(m):
    poses.append(m)


def on_status(m):
    statuses.append(m)


link.subscribe('/agent/pose', POSE_TYPE, on_pose)
link.subscribe('/agent/nav_status', STATUS_TYPE, on_status)

print('⏳ 收 5 秒数据…')
time.sleep(5.0)

if poses:
    p = poses[-1]['pose']
    print(f'✅ 3) /agent/pose 收到 {len(poses)} 帧，车在 map 系 '
          f'({p["position"]["x"]:+.2f}, {p["position"]["y"]:+.2f})')
else:
    print('❌ /agent/pose 没有数据 → agent_bridge 没起，或 Nav2/SLAM 没给 map->base_link')

if statuses:
    print(f'✅ 4) /agent/nav_status 收到 {len(statuses)} 条，最新：')
    print('   ', statuses[-1]['data'][:200])
else:
    print('❌ /agent/nav_status 没有数据')

link.close()
ok = bool(poses) and bool(statuses)
print()
print('步骤 1 通过 ✅' if ok else '步骤 1 未通过 ❌（按上面提示逐条修）')
sys.exit(0 if ok else 1)
