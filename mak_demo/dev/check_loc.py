"""定位预检：/odom（仿真真值）与 TF map->base_link 是否一致。

语义导航开车前先跑这个 —— 差超过 0.3 m 就别开车，先解决定位。
每个数都是错的（/odom 对、TF 不对）时，规划器会按错的位姿规划、车在门口反复恢复动作，
而 Nav2 日志里看不出任何异常 —— 所以要在这里量，而不是去读 Nav2。

（历史注：曾记录过一次 map->odom 偏差 3.25 m 的实例，后来确认是人为设置仿真造成的，
  不是系统缺陷。这个预检本身仍然值得每次都跑。）

跑法：.venv/bin/python dev/check_loc.py
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rosbridge_link import RosbridgeLink  # noqa: E402

ODOM_T = 'nav_msgs/Odometry'
POSE_T = 'geometry_msgs/PoseStamped'
STR_T = 'std_msgs/String'

link = RosbridgeLink()
odom, tpose, status = {}, {}, {}


def yaw_of(q):
    return math.atan2(2 * (q.get('w', 1) * q.get('z', 0) + q.get('x', 0) * q.get('y', 0)),
                      1 - 2 * (q.get('y', 0) ** 2 + q.get('z', 0) ** 2))


link.subscribe('/odom', ODOM_T, lambda m: odom.update(m))
link.subscribe('/agent/pose', POSE_T, lambda m: tpose.update(m))
link.subscribe('/agent/nav_status', STR_T,
               lambda m: status.update(json.loads(m.get('data', '{}'))))

print('⏳ 收 3 秒数据…')
time.sleep(3.0)

if not odom or not tpose:
    print(f'❌ 缺数据：/odom {"有" if odom else "无"}，/agent/pose {"有" if tpose else "无"}')
    sys.exit(1)

o = odom['pose']['pose']
t = tpose['pose']
ox, oy = o['position']['x'], o['position']['y']
tx, ty = t['position']['x'], t['position']['y']
d = math.hypot(ox - tx, oy - ty)
dyaw = abs(math.degrees(yaw_of(o['orientation']) - yaw_of(t['orientation'])))
dyaw = min(dyaw, 360 - dyaw)

print(f'/odom        (真值)  : ({ox:+.2f}, {oy:+.2f})  yaw {math.degrees(yaw_of(o["orientation"])):+.1f}°')
print(f'map->base    (TF)    : ({tx:+.2f}, {ty:+.2f})  yaw {math.degrees(yaw_of(t["orientation"])):+.1f}°')
print(f'偏差                 : {d:.2f} m / {dyaw:.1f}°')
print(f'状态机               : {status.get("state")}  goal={status.get("goal")}')
print()

if d < 0.30 and dyaw < 15:
    print('✅ 定位一致，可以开车')
    ok = True
else:
    print('❌ 定位不一致 —— 别开车。多半是 loc:=amcl 在这张图上跳了，'
          '重启时加 loc:=truth')
    ok = False
link.close()
sys.exit(0 if ok else 1)
