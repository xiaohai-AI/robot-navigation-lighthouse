#!/bin/bash
# preflight.sh —— 启动仿真/导航前先跑这个，检查"有没有残留实例"
#   用法: bash preflight.sh
#
# 判据：
#   1) 关键话题的发布者必须各为 1
#   2) 只能有一套 gz 仿真
#   3) 不允许有"孤儿 gz"：进程还在、但 bridge 已死 → 话题全无、白吃 CPU
#      （Ctrl+C 杀 launch 时，gz server 经常活下来；它是物理引擎，占 CPU 和显存）
source /opt/ros/humble/setup.bash

ok=1
no_data=0
echo "=============================================="
echo " 启动前自检"
echo "=============================================="

# ---- 1) gz 仿真实例（区分 server/gui 与 fork 出来的包装进程）----
mapfile -t gz_pids < <(pgrep -f "ign-gazebo-server|ign gazebo -s|ign gazebo -r" 2>/dev/null | sort -u)
n_gz=${#gz_pids[@]}
min_age=999999
other_part=0
echo "[1] Gazebo 进程数: $n_gz"
for p in "${gz_pids[@]}"; do
  age=$(ps -o etimes= -p "$p" 2>/dev/null | tr -d ' ')
  part=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep '^IGN_PARTITION=' | cut -d= -f2)
  dom=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep '^ROS_DOMAIN_ID=' | cut -d= -f2)
  [ -z "$age" ] && continue
  [ "$age" -lt "$min_age" ] && min_age=$age
  # 非默认分区 = 测试实例（很可能不是你要用的那个）
  [ -n "$part" ] && [ "$part" != "dan" ] && other_part=1
  printf "      pid=%-7s 已运行 %6ss  分区=%-8s 域=%-3s\n" \
         "$p" "$age" "${part:-<默认>}" "${dom:-0}"
done
if [ "$n_gz" -eq 0 ]; then
  echo "      （没在跑，OK）"
elif [ "$other_part" -eq 1 ]; then
  echo "      ⚠️  有进程跑在非默认分区（测试残留），它不影响你的默认域，但占 CPU"
fi

# ---- 2) 关键话题的发布者数量 ----
check_pub () {
  local topic=$1 expect=$2
  local c
  c=$(ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count/{print $3}')
  if [ -z "$c" ] || [ "$c" -eq 0 ]; then
    echo "[2] $topic: 无数据"
    no_data=1
    return
  fi
  printf "[2] %-14s 发布者=%s   " "$topic" "$c"
  if [ "$c" -eq "$expect" ]; then echo "OK"; else echo "❌ 应该是 $expect 个"; ok=0; fi
}

check_pub /odom 1
check_pub /scan 1
check_pub /clock 1
n_cmd=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')
echo "[2] /cmd_vel      发布者=${n_cmd:-?}   （>4 说明有残留的巡游/遥控节点）"

# ---- 3) 重复节点名 ----
dups=$(ros2 node list 2>/dev/null | sort | uniq -d)
if [ -n "$dups" ]; then
  echo "[3] ❌ 存在重名节点（说明有两套实例）："
  echo "$dups" | sed 's/^/      /'
  ok=0
else
  echo "[3] 节点名无重复   OK"
fi

# ---- 结论 ----
echo "=============================================="
if [ "$no_data" -eq 1 ] && [ "$n_gz" -eq 0 ]; then
  echo " ⚪ 仿真还没启动（无 gz 进程、话题无数据）"
  echo "    可以启动：ros2 launch ./dan_house.launch.py headless:=true slam:=true tour:=true rviz:=true"
elif [ "$no_data" -eq 1 ] && [ "$min_age" -le 45 ]; then
  echo " ⏳ gz 刚起来（${min_age}s），bridge/slam 还在加载，等 20~30 s 再跑一次本脚本"
elif [ "$no_data" -eq 1 ]; then
  echo " 🧨 疑似孤儿 gz：进程已运行 ${min_age}s，但 /clock /odom /scan 全无数据"
  echo "    → bridge 早就死了，剩下这个物理引擎在空转（白吃 CPU）。清掉再启动："
  echo
  echo "    kill ${gz_pids[*]}"
  echo
  echo "    # 若上面杀不掉，再用："
  echo "    pkill -f '[i]gn-gazebo-server'"
elif [ "$ok" -eq 1 ]; then
  echo " ✅ 一套仿真在跑且话题正常：ros2 launch 的其余部分可以直接用，别再起第二套"
else
  echo " ❌ 先清理，再启动："
  echo "    # 首选：在跑着仿真的那个终端按 Ctrl+C（等 3 秒，确保子进程都退）"
  echo "    # 终端已关就只能结束进程："
  echo "    pkill -f '[i]gn gazebo'; pkill -f '[p]arameter_bridge'"
  echo "    pkill -f '[o]dom_tf_node'; pkill -f '[t]our_node'; pkill -f '[f]rontier_explore'"
  echo "    # 然后重跑本脚本，看到全 OK 再启动"
fi
echo "=============================================="
