# STATUS —— dan_house 机器人当前状态

最后更新：2026-10-24（64 线雷达第 5 步完成，2A/2B 已在 3D 图上复验）

---

## 一句话

单线版和 64 线版**并存**：同一套 launch、同一套 Nav2 参数、同一套智能体代码，
用 `lidar:=2d|3d` 切换。64 线版能让 0.40~0.70 m 高的矮家具进地图/代价地图；
单线版整套（模型 / 世界 / `tour_node.py` / 历史地图）一个字节没动。

---

## 三条最常用的命令

```bash
cd ~/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash

# 1) 巡游建图（单线，默认；约 7 分钟仿真时间）
ros2 launch ./dan_house.launch.py headless:=true slam:=true tour:=true rviz:=true

# 2) 自主探索（换 64 线；推荐 --stall-secs 由 launch 外单独跑节点时设 18）
ros2 launch ./dan_house.launch.py headless:=true lidar:=3d explore:=true rviz:=true \
    explore_max_goals:=40 explore_max_minutes:=25 \
    explore_map_prefix:=dan_house_lighthouse3d_explore5

# 3) 语义导航（3D 图 + 真值定位 + rosbridge + 桥接）
ros2 launch ./dan_house.launch.py lidar:=3d nav:=true loc:=truth headless:=true \
    map:=config/maps/dan_house_lighthouse3d.yaml
PATH=/usr/bin:$PATH ros2 launch rosbridge_server rosbridge_websocket_launch.xml
/usr/bin/python3 agent_bridge.py
cd mak_demo && .venv/bin/python agent_nav.py "带我去冰箱看看"
```

长任务进度条（纯只读，不影响任何进程）：

```bash
python3 tools/explore_progress.py --watch --prefix dan_house_lighthouse3d_explore5 \
    --log <launch 日志> --max-goals 40 --max-minutes 25 \
    --fallback-prefix dan_house_lighthouse3d
```

---

## 地图清单

```
单线（冻结）
  config/maps/dan_house_lighthouse.pgm/.yaml              巡游图   内部未知 5.3%
  config/maps/dan_house_lighthouse_explore.pgm/.yaml      探索图   内部未知 6.0%
  config/maps/preserved/                                  更早的历史图全在

64 线
  config/maps/dan_house_lighthouse3d.pgm/.yaml            ← 正式图，内部未知 7.3%
  config/maps/dan_house_lighthouse3d_explore{,2,3,4}*     各轮原始产物 + 检查点
  config/maps/preserved/dan_house_lighthouse3d_explore*_* 归档
```

---

## 验收状态

| 项 | 状态 | 证据 |
|---|---|---|
| 阶段 1（建图 + Nav2） | ✅ | README §7 / §16；巡游图 359×278、倾斜 −0.1° |
| 阶段 2A（智能体引导导航） | ✅ | `docs/phase2A_results.md` §2（7 项在线 + 6 项离线） |
| 阶段 2B（视觉确认 / 可通行性） | ✅ | `docs/phase2B_vision.md` §7、§12.5（19/19、3/3） |
| 阶段 3（64 线改造） | ✅ | `docs/phase3_lidar3d.md` §9~§11 |
| 3D 图上复验 2A/2B | ✅ 2026-10-24 | README §17.5；`mak_demo/` 一行未改 |
| 独立复现（Dan 手敲全部在线验收） | ✅ 2026-10-24 晚 | 全部通过；命令见 README §17.5 / STATUS「三条最常用的命令」 |

---

## 已知未完成（都不是阻塞项）

```
· 中途插话改主意（抢占模式）✅ 已于 2026-10-25 实现并在线验收 7/7（见 ⑩~⑫）
· AMCL 在探索图上会跳 124° → 仿真统一用 loc:=truth（注：那次跳变已确认是人为设置仿真所致，
  不是系统缺陷，见 STATUS ⑮；loc:=truth 仍是仿真推荐做法）
· EKF（C 模式）长跑仍会漂（7.1 m / map→odom −37°）
· 放射状喷雾（未命中射线被涂成 free）仍在
· 卡死看门狗已 A/B 验证：18 s 无收益（卡死 4 vs 2、覆盖更差 56.5 vs 45.0 m²）→ 保持 12 s；
  下一步该治的是目标可达性（贴墙/贴死角剔除），不是计时器
· 剩余内部未知 13.5 m² 主要是家具背后阴影口袋，每补一轮 40 目标只再换 ~1 m²
```

---

## 下一步候选（2026-10-24 夜，按 收益÷工作量 排序）

> ① 已于 2026-10-25 实施并实测，结论见本节末尾「① 实测结果」。

```
① 目标可达性预检  ← 已实施（见下）
   选目标时在 /map 上判断"目标格与车是否同一 free 连通块"，
   再加"目标周围 0.5 m 内 free ≥0.3 m²"；不满足直接跳过不发车。
   约 40 行，可离线用现有地图验证，不占仿真。收益：同配额多建 10~20% 面积，
   并消掉"走到死角卡 12 秒再拉黑"这一类浪费（实测占失败 8~22%）。

② 中途插话抢占（阶段 2 唯一没做的交互能力）  ← 已于 10-25 实施，见 ⑩~⑫
   nav_client.go_to() 加 cancel_if + 后台读 stdin，取消后等 /cmd_vel 归零再发新目标。

③ 真机定位（loc:=amcl 分支从未验收过）  ← 唯一的真机前置条件，124° 那条已剔除
   现在全部验收都用 loc:=truth；loc:=amcl 自 10-22 之后没再跑过，参数也没调。
   真机要动之前必须先把这条验通：map→base_link 与 /odom 在长距离行驶中保持在 0.3 m 内。

④ 真 3D 障碍（costmap voxel_layer 直接吃 /scan/points）
   现在 64 线被压回一条 2D 线；悬空物/桌底场景才需要，当前世界收益有限。
```

### ① 实测结果（2026-10-25）：预检按设计工作，但**不是**卡死的解

```
代码   frontier_explore.py 发目标前先向 compute_path_to_pose 要一条路径
       · 算不出路径 → 临时黑名单 30 s（不发车、不永久拉黑边界）
       · 直线很近但要绕很远（dist<3 m 且 plen>3·dist+1.5）→ 判定"在另一个封闭区域"，跳过 60 s
       · 连续 3 次超时 → 本轮自动关闭预检，回到旧行为
       参数：--plan-check / --no-plan-check / --plan-timeout 1.5

三组同条件（空地图起步、25 目标 / 15 分钟、12 s 看门狗、净空 0.40）
                    A 基线(无预检)    C 预检开
  到达                   22             18
  卡死                    2              6
  内部未知              45.0 m²        56.5 m²
  预检统计                —            通过 25 / 拦下 1 / 超时 1
```

**为什么反而更差**：卡死根本不是"规划器没路"。C 组 6 次卡死，控制器日志全是同一条：

```
controller_server: Failed to make progress    → abort → 清 local_costmap → 立刻重试 → 再失败
```

即：**全局路径有，但 DWB 在局部找不到无碰撞轨迹**（64 线让代价地图里多了低矮家具的边缘，
窄处的可行轨迹被吃光）。explore4 那轮里这条也出现了 31 次。

一个有价值的副产品：预检把"路径长度/直线距离"变成了可用信号。

```
                 到达组    卡死组
平均绕路比        1.12      2.42
最极端                     直线 1.5 m → 路径 10.6 m（绕 7 倍，目标在另一个封闭区域）
```

**结论**：预检保留（便宜、能防"跨封闭区域"目标和"永久拉黑好边界"），但它治不了主因。
下一步应该动**局部控制器**，不是继续加目标筛选：
```
a) 受困时的恢复动作从"原地转"换成"后退"（behavior_server backup），让车离开窄处再试
b) 调 progress_checker（movement_time_allowance / required_movement_radius）
c) 换控制器（RPP / MPPI），DWB 在窄通道本来就偏弱
```

### ② 恢复动作改造（a）实测结果（2026-10-25）

```
新增  config/nav_to_pose_explore.xml    探索专用 BT：恢复链 BackUp 排第一（0.35 m @ 0.06 m/s）
新增  config/nav2_params_explore.yaml   仅 explore:=true 生效：进度判定 0.5/10 s → 0.25/6 s
      + default_nav_to_pose_bt_xml 指向上面那个 BT
launch  nav_live 改用 NAV2_PARAMS_EXPLORE（2A/2B 验收仍用 nav2_params*.yaml，不受影响）
验证  ros2 param get /bt_navigator default_nav_to_pose_bt_xml → 指向我们的文件 ✅
      behavior_server: Running backup → backup completed successfully  ×3（每次约 7 s）✅

三组同条件（空地图、25 目标 / 15 分钟、净空 0.40）
                      A 基线(12 s,旧 BT)   C 预检(12 s,旧 BT)   D 预检(25 s,后退 BT)
  到达                      22                  18                  18
  卡死                       2                   6                   6
  仿真用时                 7.6 min             8.6 min            12.3 min
  内部未知                45.0 m²             59.7 m²             37.6 m²  ← 覆盖最好
```

结论：**后退动作确实跑了，地图也更完整（37.6 m² 未知，三组最好），但卡死次数没降**，代价是慢 44%。

卡死点的几何测量（`probe_stall_clearance.py`，对保存的地图量距离）：
```
D 组 6 次卡死时车的位置：到最近障碍 0.49~0.90 m（中位 0.90），到未知 0.54~1.00 m
→ 不在膨胀区里、也不贴未知边界：不是"撞到了"，也不是"未知当障碍"
→ 25 s 内只挪 0.02~0.19 m：DWB 在开阔处就是不给速度
```

所以真正的失败点在 **DWB 的轨迹生成层**（不是目标、不是规划、不是碰撞几何）。
下一步要用 Nav2 自己的 DEBUG 日志（`No valid trajectories out of N` / oscillation 判决）
复现一次卡死，读到 DWB 的拒绝理由再改参数，别再盲调。

### ③ DEBUG 复现：卡死的判决书拿到了（2026-10-25）

工具：`probe_eval.py` 订阅 DWB 的 `/evaluation`（`dwb_msgs/LocalPlanEvaluation`，
含**所有**被打分轨迹的逐 critic 原始分）+ `probe_eval_compare.py` / `probe_eval_forward.py`。

卡死瞬间（419 条候选，**没有一条被判 invalid**）：

```
被选中的慢轨迹：vx=0.000  wz=-0.800   总分 63.40
最好的快轨迹  ：vx=0.103  wz=-0.800   总分 72.70    ← 更快反而更贵

critic           慢轨迹raw   快轨迹raw       差
BaseObstacle        0.00      195.00    +195.00   ← 局部代价地图"前方有障碍"
PathDist            0.00        5.00      +5.00
PathAlign           2.00        6.00      +4.00
```

结论：**这不是"没路走"，是"打分上站着不动更便宜"**。

- `BaseObstacle.raw = 195` 在 `inflation_radius 0.45 / cost_scaling 3.0` 下
  对应"离障碍约 0.30 m"。局部代价地图认为前方 30 cm 有东西。
- 两条链路的层配置完全对称（都吃 `/scan`、marking+clearing、范围一致），
  所以不是"局部看见全局看不见"，而是**规划器可以穿过膨胀格（只加代价），
  控制器却为此付出 195 的分**。门洞/家具旁唯一可行的那条路，规划器照走，DWB 不跟。

副产品（很重要）：**"爬行"不是偶发**。H 组 1400 条 evaluation 里有 **833 条**
最优轨迹 vx<0.02（占 58%）；只是大部分几秒后自己恢复，只有超过看门狗阈值的那几次
才被记成"卡死"。所以"卡死 2~3 次/15 目标"是冰山一角，实际一半的控制周期都在纠结。

修法候选（按改动大小，都需要用"爬行占比"这个新指标验收）：
```
a) 局部膨胀层瘦身：inflation_radius 0.45 → 0.32（代价带变薄，前进不再被重罚）
b) 降 BaseObstacle 权重：0.02 → 0.005（敢往前走，但可能更贴障碍）
c) 让规划器也把高代价格当障碍：换 SmacPlanner2D / 调 cost_penalty（根治"规划走的路控制器不敢走"）
d) 换控制器：RPP / MPPI（DWB 的"站立最优"是它自己的代价函数特性）
```

### ④ c 方案：规划器换 SmacPlanner2D（2026-10-25，已生效）

```
config/nav2_params_explore.yaml
  planner_plugins: ["GridBased"]
  GridBased.plugin: nav2_smac_planner/SmacPlanner2D   ← 原 nav2_navfn_planner/NavfnPlanner
    tolerance 0.20 / allow_unknown true / downsample_costmap false
    max_iterations 1000000 / max_on_approx 0
  只影响 explore:=true；nav2_params.yaml 与 nav2_params_nav.yaml 未动（2A/2B 验收不受影响）
启动已验证：planner_server: Configured plugin GridBased of type SmacPlanner2D ✅
规划失败：0 次（15 目标全程）
```

为什么对症（离线可算的几何）：
```
1.0 m 门洞中线离门框 0.5 m  > inflation_radius 0.45  → 中线代价是 0（本来是免费的）
切弯切到离门框 0.32 m          → 代价 = 253·exp(-3·(0.32-0.23)) ≈ 193（实测 BaseObstacle 195）
NavFn 用势场，只要 <253 就敢穿 → 规划器抄近道 → DWB 吃到 193 的罚分 → 不肯走
SmacPlanner2D 是 A*，逐格累加代价 → 主动贴便宜的中线走
```

实测对比（同世界、同 15 目标、同看门狗 25 s，`probe_eval_share.py` 三分类）：

```
                  总周期   发呆      原地转    行驶        内部未知(15 目标)   仿真用时
NavFn  (H 组)     2339   26.4%    18.9%   54.7%      93.2 m² (49.8%)    4.6 min
Smac   (I 组)     4114   20.1%     9.3%   70.6%      72.0 m² (38.6%)    6.2 min
```

**结论：c 有效。** 同样 15 个目标，未知区域少 21.2 m²，行驶占比 +15.9 个点，
发呆/原地打转的时间几乎减半。卡死次数仍是 3/15，但因为车真的在跑，
同样的配额推进了明显更多的面积。代价是每目标慢 ~35%（因为它在真的走，而不是原地耗）。

下一步候选：
```
① 用 Smac 跑 40 目标 / 25 min，看能不能低于现役 3D 图的 13.5 m² 未知（换图需 Dan 拍板）
② 剩下的 20% 发呆主要由 PathAlign 主导（4.5×），可试降 PathAlign.scale 32 → 18
③ 把 Smac 也用在 tour/验收参数里（当前刻意不动，避免影响 2A/2B 回归）
```

### ⑤ 40 目标干净对照：SmacPlanner2D **反而更差**，已改回 NavFn（2026-10-25）

两组 40 目标、同世界、同参数文件（除规划器）、同看门狗 25 s：

```
                  到达      卡死    内部未知      发呆    原地转    行驶    仿真用时
NavFn (K 组)      35/40    4     18.0 m²      8.3%    9.4%   82.3%   21.9 min
Smac  (J 组)      28/40   11     36.2 m²     25.3%   11.2%   63.5%   22.2 min
```

差距**全部集中在书房**（x≈+4, y≈+3 那格）：Smac 16.4 m² vs NavFn 2.1 m²、现役图 1.7 m²。
其余 7 格两者相差都在 1 m² 内。也就是说：Smac 在东侧走廊/书房门口反复卡死（11 次里 4 次在
x>+3），失败又永久拉黑目标点（本轮 26 次拉黑）→ 书房再也回不去。

**结论：c 方案是退步，已把 `nav2_params_explore.yaml` 改回 NavFn。**
`config/nav2_params_explore_navfn.yaml` 与它除注释外完全相同，作为对照臂留档；
`explore_params:=` 开关保留，以后换规划器只需换一行。

重要教训：**15 目标的 A/B 不可信**。15 目标时 Smac 赢 21.2 m²（72.0 vs 93.2），
40 目标时输 18.2 m²（36.2 vs 18.0）—— 前一个结论是噪声。

顺带测到：D/I 改造（先后退 BT + 0.25/6 进度判定 + 25 s 看门狗）确实把卡死从
explore4 的 9/40 降到 4/40，但**地图没更好**（18.0 vs 13.5 m²）——后退重试在消耗时间，
不是在做覆盖。现役最优图仍是 explore4 那张（13.5 m²）。

未解问题（下一轮候选）：
```
① explore4 是 goal-clearance 0.50 + 旧 BT + 12 s 跑出来的 13.5 m²，与当前配置差 3 个变量
   → 要复现"最优图"的配置，得把这 3 个变量拆开测（每轮 22 min）
② 书房为什么容易卡：Smac 4/11、NavFn 也有 2 次在 x>+3 → 东侧走廊几何本身有问题，
   值得单独用一条直线验收（像当初灭火器那样）
```

### ⑥ 书房之谜：门洞无罪，问题在"目标点被摆在税区里"（2026-10-25）

先排除几何（离线 `probe_door_bottleneck.py`，逐门算"从走廊过去最窄处半宽"）：

```
客厅 0.55 m  厨房 0.55 m  书房 0.55 m  卧室1 0.55 m
卧室2 0.55 m 卫生间 0.50 m 储藏间 0.55 m      全部 ≥ inflation_radius 0.45 → 代价 0
```
**书房门本身完全没问题**，door 也没有被家具堵住。所以"东侧走廊几何有问题"这个假设被证伪。

真正相关的是**目标点自身的净空**（`probe_goal_bottleneck.py`，量每个卡死目标）：

```
Smac 11 次卡死里 5 次：目标点净空 0.35~0.40 m → 从走廊过去的瓶颈也是 0.35~0.40 → 代价 132~177
explore_goal_clearance 默认值 = 0.40    inflation_radius = 0.45
→ 0.40 < 0.45：到这类目标必须整条路走在"膨胀税区"里 → DWB 站着扭
```

验证（`explore_goal_clearance:=0.55`，40 目标，其余不变）：

```
                      到达    卡死   内部未知     发呆    行驶    仿真用时
clearance 0.40 (K)   35/40    4    18.0 m²    8.3%   82.3%   21.9 min
clearance 0.55 (L)   35/40    3    27.2 m²    7.7%   83.6%   15.3 min
```

**是权衡不是免费午餐**：0.55 更快、卡死更少，但覆盖更差（27.2 vs 18.0）——
因为它会跳过"贴着家具/角落"的那些目标（多出来的未知集中在厨房北侧 9.8 m²）。
0.40 = 敢往深处钻但容易卡；0.55 = 跑得快但只在宽敞处转。

跨组总览（都是 40 目标）：
```
现役图 explore4 (clearance 0.50, 旧 BT, 12 s)   13.5 m² unknown
navfn40      (0.40, 新 BT, 25 s)               18.0 m²  21.9 min
gc55         (0.55, 新 BT, 25 s)               27.2 m²  15.3 min
smac40       (0.40, 新 BT, 25 s, Smac)         36.2 m²  22.2 min
```
**差距（13.5~27.2）远大于任何单变量改动的效果** → 结论：这个任务上单次 40 目标的 A/B 不可靠，
要判断哪个配置更好必须每臂跑 ≥3 次取中位数（约 4.5 小时仿真）。

现状：默认仍是 clearance 0.40 + NavFn + 新 BT；现役图仍是 explore4 的 13.5 m²（未被超过）。
新增可复用工具：`probe_door_bottleneck.py`（每道门最宽通道）、
`probe_goal_bottleneck.py`（目标点净空 + 走廊→目标瓶颈 + 对应 Nav2 代价）。

### ⑦ 可重复实验驱动 `tools/explore_ab.py`（2026-10-25）

背景：⑥ 发现同配置两次 40 目标能差 9~18 m²，单次结果不可作为结论。
这个脚本把"多臂 × 多次 + 自动统计"做成一条命令。

```bash
# 完整实验：净空 0.40 / 0.50 / 0.55 各 3 次，每次 40 目标（约 3.3 小时）
/usr/bin/python3 tools/explore_ab.py

# 冒烟（先验证管道，5 分钟）
/usr/bin/python3 tools/explore_ab.py --arms 0.40 --reps 1 --goals 6 --minutes 5 --tag smoke2

# 只看已有结果（中位数汇总）
/usr/bin/python3 tools/explore_ab.py --report-only
```

每轮自动做：独立 ROS_DOMAIN_ID/IGN_PARTITION（不碰你手动的仿真）→ 起 eval 探针 →
起探索（独立地图前缀，绝不覆盖现役图）→ 轮询到"探索结束"或超时 → 收尾（SIGTERM 会话组，
再按 domain+partition 双条件清残留）→ 自算 8 个指标写 `experiments/ab_explore.csv`。
断点续跑：CSV 里已有的 (tag, 臂, 第几次) 自动跳过。

三道防废数据的保险（冒烟时实测踩到第一条）：
```
① 开机自检   210 s 内必须看到"前沿探索启动"，且不能出现
             "Failed to bring up all requested nodes"（Nav2 生命周期竞态，实见过一次）
② 0 目标作废 一轮要是没发出任何目标，标 no_goals，不进汇总
③ 自动重试   无效轮换一个 domain 重试（--retries 默认 2）
```

指标口径与既有探针一致（已验证：现役图 13.5 / navfn40 18.0 / gc55 27.2 / smac40 36.2 m²）。

### ⑧ 3×3 重复实验：净空这个旋钮**测不出效果**（2026-10-25，tag 1025_1647）

九轮全部跑完（净空 0.40 / 0.50 / 0.55 各 3 次，每次 40 目标、看门狗 25 s、NavFn、同一世界）：

```
臂       n   未知 m² 中位 [极差]      到达 中位 [极差]   卡死 中位 [极差]   仿真 min 中位 [极差]   发呆% 中位 [极差]
0.40     3   20.2 [17.9-31.1]        33 [30-34]        6 [5-8]          18.9 [16.9-21.3]      11.3 [9.2-16.8]
0.50     3   21.2 [20.3-22.2]        36 [32-36]        3 [2-7]          16.8 [16.2-17.1]       9.6 [5.1-16.3]
0.55     3   23.1 [19.3-33.1]        33 [30-35]        6 [4-8]          16.6 [15.6-18.8]      10.6 [9.4-20.4]
```

**判决：组间差异小于组内噪声，净空取值在这个任务上不是覆盖率旋钮。**
```
指标        组间(中位差)   组内(极差)   结论
未知 m²        2.9         13.8      差异无法与噪声区分
到达            3.0          5.0      同上
卡死            3.0          5.0      同上
仿真 min        2.3          4.4      同上
发呆 %          1.7         11.2      同上
```

因此更正 ⑥ 的结论：`0.55 → 27.2 m² vs 0.40 → 18.0 m²` 那次"0.55 更差"是**单次抽样**，同一个 0.55 臂
这次跑出 19.3 / 23.1 / 33.1 —— 极差 13.8 m²。0.50 看起来卡死更少（中位 3 vs 6），但极差 2~7，
同样是噪声范围内。**结论：净空只需守住 ≥ inflation_radius（0.45）这条安全底线，
具体取 0.40/0.50/0.55 不必再调。**

真正的瓶颈是**预算**：九轮全部落在 17.9~33.1 m²，都差于现役图 13.5 m² —— 因为 40 目标在这层楼
只有半程量（技能记录：~80 目标 / ~26 min 才是完整一轮）。所以 40 目标的对照测的是"覆盖率速率"，
把它当成"最终覆盖率"来读，就是之前两个相反结论的来源。

副产品（工具缺陷，已修）：10 次启动里有 1 次作废（0.50 第 3 次，domain 125，25 分钟 0 目标）。
当时日志按 `tag_臂_次数` 命名，重试把失败尝试的同名日志**覆盖**了，证据丢失。现在：
```
① 日志名带 domain       tag_臂_次数_d<domain>.log
② 失败尝试也入 CSV       status=no_goals/nav_bringup_failed，供复盘统计浪费率
③ 断点续跑只认有效行      status=done 且 arrived>0，失败行不会把该轮标记成完成
```

### ⑨ 80 目标：覆盖曲线在 ~48 目标处拐平，加预算没有意义（2026-10-25，tag 1025_2047）

两轮 80 目标（净空 0.40、45 min 上限、NavFn），都跑在**目标数上限**结束（不是"探完"）：

```
轮次   到达    卡死   仿真        内部未知    发呆
#1    63/80    15   40.9 min    15.8 m²   15.6%
#2    57/80    21   40.7 min    16.2 m²   20.1%
```

从每 8 个目标的检查点地图上还原覆盖曲线（内部未知 m²）：

```
目标数     8     16     24     32     40     48     56     64     72     80
run #1  107.2   70.0   57.3   31.5   18.7   18.2   18.0   17.0   16.6   15.8
run #2  111.3   81.9   72.1   32.5   22.5   19.4   19.1   16.7   16.5   16.2
```

**结论：曲线在 ~48 目标处拐平。** 最后 24 个目标只买到 2.2~2.9 m²（≈0.1 m²/目标），
代价是卡死从 6/40 涨到 18/80、发呆从 11% 涨到 18%（≈22% 的预算消耗在被看门狗处死的目标上）。

至此探索质量这条线可以收尾，三条都落在噪声或物理限制上：
```
① 执行层参数（规划器 / BT 顺序 / 看门狗 / 进度判定）  —— 都不在噪声之上（⑥⑧）
② 净空 0.40 / 0.50 / 0.55                          —— 组间 2.9 m² < 组内 13.8 m²，无差别（⑧）
③ 预算 40 → 80 目标                                 —— 边际收益 ~0.1 m²/目标，拐点在 48（⑨）
```

剩余未知（现役 13.5 / 本轮 15.8~16.2）主要是家具背后阴影与不可达缝隙，属物理限制，
不是 `NavigateToPose` 类目标能补的。**现役图 `dan_house_lighthouse3d`（13.5 m²）保留为正式图**，
但不要宣称"它比现配置更好"——13.5 与 15.8 的差值在 40 目标实测的 13.8 m² 极差之内。

技能里"~80 目标 / ~26 min"的预算已按实测校准为：拐点 48~56 目标、80 目标实测 41 min 仿真。

### ⑩ 抢占（中途改主意）已实现并通过离线验收（2026-10-25）

之前交互模式是"一条跑完再下一条"：`nav_client.go_to()` 同步阻塞，车在路上的几分钟里
事件循环是死的，`input()` 读不到键盘 —— 所以"改主意"在物理上不可能生效。

改造后（三步，缺一退化）：
```
mak_demo/preempt.py      词表 + 判定表（零依赖）
mak_demo/agent_nav.py    发车调用 await asyncio.to_thread(...)  + 路线 F（纯叫停→cancel，零模型）
mak_demo/main.py         每条指令一个 asyncio.Task；抢占时先 cancel 再发新目标
```

判定序（关键边界）：**有新目的地优先于取消词** ——「别去厨房了，去卧室」必须改道，
不是停车。其次纯叫停 → cancel；车在路上但问的是问题 → 并行回答不打断。

离线验收 `dev/check_preempt.py` 20/20：
```
① 判定表 6 条边界                        全过
② 真实句子解析（含"新目的优先"边界）        全过
③ 假连接：go_to 卡住时 cancel → 2 s 内返回 canceled，1 目标 1 取消   全过
④ to_thread vs 同步直调：心跳 25 次 vs 0 次；源码护栏不许退回同步调用   全过
```
回归：mak_demo 离线套件 11/11 通过（新增 check_preempt，其余 10 项无变化）。
文档：README §18（含四终端在线验收步骤）。技能 agent-guided-robot-navigation 已补该教训。

**待做**：在线验收（需仿真，Dan 自己跑，命令见 README §18.5）。

### ⑪ 抢占在线首跑：三处并发缺陷已修（2026-10-25）

首次仿真验证（Dan 实测）暴露三个离线测不到的缺陷，全部已修并加护栏：

```
① 桥接无目标序号
   旧目标的 result（结束码 5=canceled）晚到，被写成"新目标 failed"，
   并把 goal_handle 清成 None → 之后取消与看门狗都够不着真目标
   修：goal_seq，三个异步回调带序号回归，过期丢弃

② 取消撤不掉工作线程
   asyncio 任务取消后线程仍在轮询冻结状态，刷 2 分钟到 180 s 超时
   修：nav_client 在飞登记表 + stop()/stop_all()；go_to 查 stop → 立刻 canceled

③ 进度打印与 input() 抢终端
   三行输入被拼成一条指令，问句被当成叫停吞掉
   修：交互模式默认关进度（--progress 打开）+ 判定表补"问句优先于叫停词"
```

离线套件 11/11（含 check_preempt 36 项断言）；README §18.6 记录了三个 bug 的现象→根因→修法→护栏。
**待复验**：在线按 §18.5 重跑一次，确认新目标不再 0.1 s 报 failed、取消后无残留刷屏。

### ⑫ 抢占在线复验通过（2026-10-25，隔离 domain 160）

由助理代跑（Dan 授权）：仿真 + Nav2(loc:=truth, 3D 图) + rosbridge + 新 agent_bridge +
脚本按秒喂指令（t=15 s 插话必须落在"车还在走廊里"的窗口，手工敲不可控）。

```
t=0    带我去厨房                      → 正常执行，开始走
t=15   别去了，去客厅                  → 判定：抢占   ⏹ 已取消「带我去厨房」，改去「别去了，去客厅」
       （桥接）已有目标在飞，先取消旧的（旧 #1 → 新 #2）
       （桥接）忽略已被取代的目标 #1 的结果（当前 #2）     ← 修复点
t=41   ⏹ 「带我去厨房」已取消（发车）                   ← 被取消那段没谎报到达
t=71   ✅ 去 客厅（living_room）中心，耗时 26.9s，状态 succeeded，剩余 0.192 m
t=70   停下                            → 判定：取消   已发送取消，没有发新目标
t=78   冰箱在哪                        → 判定：并行   冰箱在厨房里，从这边看东方向约 3.5 米
```

判据 7/7：抢占提示 ✅｜旧段「已取消（发车）」✅｜新目标 succeeded（不是 failed）✅｜
无 failed 误报 ✅｜叫停后无残留刷屏 ✅｜不把叫停段当目的地（无「别了」）✅｜无谎报到达 ✅

复验中又发现并修掉第 ④ 个缺陷：`别去了，去客厅` 按逗号拆段后，第一段被当成目的地解析，
用户会看到「家里没有叫「别了」的地方」。修法 `preempt.filter_cancel_segments()`，
离线新增 4 条分句用例。离线套件 11/11，check_preempt 40 项断言。

环境：跑完已 kill，domain 160 零残留，9090 已释放。

---

## 恢复点（2026-10-25 收工，明天从这里接）

### 代码现状（今天动过的文件，其余一律未改）
```
mak_demo/preempt.py        新增  词表 + 判定表 + 分句过滤（零依赖）
mak_demo/agent_nav.py      +   发车改 asyncio.to_thread；路线 F（纯叫停）；release()
mak_demo/main.py           ~   REPL 改 Task 化抢占；--progress；_run_sequence 也走 to_thread
mak_demo/nav_client.py     +   VERBOSE 开关 / 在飞登记表 / stop() stop_all() / 抢占返回 canceled
mak_demo/tools.py          ~   go_to 不再写死 verbose=True
agent_bridge.py            +   目标序号 goal_seq（旧回调丢弃）—— 这是"新目标秒报 failed"的根因修复
mak_demo/dev/check_preempt.py   新增  40 项离线验收
```
冻结项仍冻结：第一阶段巡游 / tour_node.py / 2D 世界 / 现役地图 / mak_demo 其余模块。

### 已验证
```
离线 mak_demo 套件        11/11（含 check_preempt 40 项）
在线 抢占（domain 160）   7/7 判据，桥接日志见「忽略已被取代的目标 #1 的结果（当前 #2）」
环境                      domain 160 零残留、9090 已释放
```

### 待做（按优先级）
```
① 可选：把 ④ 号修复也在真机路径上再验一遍（今天这次已覆盖：喂的是「别去了，去客厅」）
② 2A / 2B 全梯回归        今晚改了 nav_client / main / agent_bridge 下层 → 按规矩要重验
                          命令见 README §18.5；验收红线：被取消段不得报到达
③ ~~AMCL 在探索图跳 124°~~ **已剔除**（2026-10-26：Dan 确认那是他当时人为设置仿真造成的，
                          不是系统缺陷。loc:=truth 仍作为仿真推荐做法保留，但不列为待修项）
④ 3d 模式「绝不桥 gz /scan」只有注释 → 已于 ⑭ 变成启动断言（tools/scan_pub_guard.py）
```

### 恢复对话时的一句话
> 读 STATUS.md 第 ⑩~⑫ 节与「恢复点」，然后接着做第 ② 项（2A/2B 全梯回归）。

### ⑬ 2A/2B 全梯回归通过（2026-10-26，隔离 domain 161）

背景：⑩~⑫ 改了 `nav_client.py` / `main.py` / `agent_nav.py` / `agent_bridge.py` 四个下层文件，
按"下层一改、上层整梯重验"的规矩，把 8 个闸门全部重跑一遍（3D 图、loc:=truth、headless）。

```
闸门                        本次实测                                     归档基线              判定
1a 链路 check_link          pose 24 帧 / status 11 条 / idle           25 帧 / 11 条         ✅
1b 定位 check_loc           偏差 0.00 m / 0.0°                         0.00 m / 0.0°         ✅
2  硬编码 3 m test_nav      succeeded 13.3 s，终点误差 0.19 m，0 恢复   57.0 s / 0.197 m      ✅ 更快
3  语义导航「冰箱」          succeeded 63.0 s，落点偏差 0.15 m ≤ 0.30   —                     ✅
4  模式 D「带我去冰箱看看」  观察位 1.79 m、VLM 13.1 s「冰箱」、四核查全绿  1.79 m / 58.5 s       ✅ VLM 快 4.5×
5  模式 E「前面能过去吗」    雷达 1.12 m（可前进 0.91 m）+ 视觉「被挡住」一致   —                  ✅
6  模式 B「洗手间在哪」      41.1 s「南边约 9.2 米」，未发车              41.7 s / 9.2 米       ✅ 一致
7  拒绝「带我去地下室」      0.0 s 正确拒绝，0 次移动                  0.0 s                 ✅
8  复合「先去客厅，然后到冰箱旁边」两段 succeeded 55.6 + 58.2 s，0 恢复  91.6 + 58.0 s         ✅ 更快
```

关键两点：
- **闸门 8 是必跑项**：④ 号修复（`filter_cancel_segments`）改的正是复合指令的拆分路径，
  本闸门证明正常复合指令没被误伤（「先去客厅」两段都保留、都到达）。
- 桥接日志：本轮 6 个目标全部 `succeeded`，`忽略已被取代的目标` 出现 0 次 ——
  序号机制在没有抢占的场景下不产生副作用。

环境：跑完已 kill，domain 161 零残留，9090 已释放。图片证据 `mak_demo/logs/look_132324.png`（冰箱）、
`look_132353.png`（可通行性，视觉看到冰箱挡住）。

剩余未做（明天可选）：① ③ AMCL 跳变（真机才有）② ④ 3d 模式双发布断言。

### ⑭ /scan 单发布者启动断言（2026-10-26）

之前 3d 模式的铁律「绝不桥 gz 自带 /scan」只有一行注释，没有可执行的检查。
双发布时 slam_toolbox 用哪一路取决于到达顺序 → 地图重影/时好时坏/突然清空，
现象完全不指向"多了一路 /scan"，排查代价极高。现在变成启动时就能看见的红字。

```
tools/scan_pub_guard.py     rclpy 节点，--watch 常驻复查，--timeout 启动宽限期
tools/check_scan_guard.py   离线单测 6/6（0 个 / 1 个对 / 1 个错 / 2 个 / 带命名空间）
dan_house.launch.py         3d → --expect pointcloud_to_laserscan，2d → --expect gz_bridge
                            两者用 IfCondition/UnlessCondition 互斥，进 LaunchDescription
```

实测（三个场景全是真跑）：
```
3d 正常     [scan_guard] ✅ 通过：/scan 发布者 1 个：/pointcloud_to_laserscan
3d 负向     [scan_guard] ❌ 失败：/scan 有 2 个发布者：/_ros2cli_12764、/pointcloud_to_laserscan
            （人为起第二个发布者，30 s 内抓到，ros2 topic info 同步显示 2）
野发布者退场 [scan_guard] ✅ 通过  ← 自愈，不需要重启
2d 正常     [scan_guard] ✅ 通过：/scan 发布者 1 个：/gz_bridge
```

设计取舍：判据是"恰好 1 个发布者且节点名匹配"，而不是只看数量 ——
只看数量会放过"桥接配错路数、投影节点没起"这种单发布者但错源的场景。
启动宽限期内"0 个发布者"不下结论（桥接与投影并行启动），但"≥2 个"任何时候立刻算数。

### ⑮ 更正：AMCL 跳 124° 不是本项目缺陷，从待修列表剔除（2026-10-26）

Dan 说明：那次 `map→odom` 平移 1.25 m + 旋转 124.5° 的跳变，是他**人为设置仿真**时造成的，
不是 AMCL 或本项目的缺陷。因此：

```
✅ 做法保留   仿真统一用 loc:=truth（仿真里 /odom 就是真值，没必要让 AMCL 去"修正"）
✅ 预检保留   dev/check_loc.py 每次都跑（"用 /odom 与 TF 对比、而不是读 Nav2 日志"这个方法有效）
❌ 因果作废   "AMCL 在这张探索图上会跳" 不再作为已知缺陷
❌ 待修剔除   从「恢复点」的待办里删掉，不再计划做"位姿跳变守护"
```

就地更正的位置（保留历史记录 + 加日期标注，不删除当时的观测数据）：
```
STATUS.md 自身             恢复点 ③ 划掉；第 79 行那条加注
dan_house.launch.py        loc 参数上方的注释（结论不变，注明出处）
mak_demo/dev/check_loc.py  docstring（换成"为什么要量"，历史注一句）
mak_demo/STATUS.md §2.1    段落首加「更正」引用块
docs/phase2A_results.md    §6.5 与文档头（做法保留，因果判断作废）
技能 agent-guided-robot-navigation / gazebo-fortress-sim
                           同一处 pitfall 加出处注：诊断顺序有效，但别当成预期故障去找
```

结果：AMCL 跳变从"待修缺陷"里剔除。**仍未修的只剩两条（都是已知限制，不在本轮范围）**：
```
· EKF（C 模式）长跑仍会漂（7.1 m / map→odom −37°）—— 仿真默认不用 EKF，属 C 阶段课题
· 放射状喷雾（未命中射线被涂成 free）—— SLAM 侧现象，影响地图边缘美观，不影响导航
```

---

## 交付冻结

本文件与 `README.md`、`docs/phase2A_results.md`、`docs/phase2B_vision.md`、
`docs/phase3_lidar3d.md` 已于 **2026-10-26** 冻结为交付物，快照在 `delivery/2026-10-26/`，
逐字节副本 + `MANIFEST.sha256`。冻结后再改正文请另开一版（`delivery/<新日期>/`），
不要原地改快照。
