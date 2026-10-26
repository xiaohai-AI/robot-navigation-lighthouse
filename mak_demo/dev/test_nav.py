"""步骤 2 验收：发一个真实目标，看车动起来。"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nav_client import NavClient          # noqa: E402
from rosbridge_link import RosbridgeLink  # noqa: E402

TARGET = (3.0, 0.0)          # 走廊东侧，从出生点(0,0)直线过去 3 m

link = RosbridgeLink()
nav = NavClient(link)

if not nav.wait_ready():
    print('❌ 收不到 /agent/nav_status，agent_bridge 起了吗？')
    sys.exit(1)

print('起点位姿:', nav.pose())
print(f'发出目标 {TARGET} …')
t0 = time.time()
state, st = nav.go_to(*TARGET)
print()
print(f'结果: {state}   耗时 {time.time()-t0:.1f}s')
print(f'终点位姿: {st.get("pose")}')
print(f'剩余距离: {st.get("distance_remaining")}   恢复次数: {st.get("recoveries")}')
link.close()
sys.exit(0 if state == 'succeeded' else 1)
