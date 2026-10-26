# 第三阶段：64 线激光雷达改造

> 最后更新：2026-10-24（第 3、4 步实测完成，假边界 bug 修复，3D 地图验收归档）
> 目的：把"单线雷达只能扫一根线"这个结构性缺陷从根上解决 —— 换成 64 线，
> 让低于 0.78 m 的家具重新进入地图/代价地图，并把之前为绕开缺陷而抬高的家具退回去。

---

## 0. 一句话现状

**第 1~4 步全部完成并实测通过；3D 版已经能建出"低矮家具在图里"的地图。**
单线那套（`models/lighthouse`、`dan_house.world`、`tour_node.py`、所有旧地图）一个字节没动。

```
✅ 第1步  64 线雷达模型 models/lighthouse_3d（SDF + URDF + 真机尺寸雷达头）
✅ 第2步  build_world.py --lidar 2d|3d；launch 的 lidar:= 开关；pointcloud_to_laserscan 投影
✅ 第3步  lidar:=3d 端到端验收：RTF 0.98、/scan 8.50 Hz、359/360 格有回波
✅ 第4步  3d 世界家具回真实高度（2d 世界仍 0.86 m，与冻结版逐字节一致）
✅ 验收    9 件低矮家具全部进图（茶几 174 格 vs 单线 0 格），地图已归档
✅ 第5步  README §17 / phase2A §10 / phase2B §13 / STATUS.md 已更新；
          2A/2B 在 3D 图上全过（`mak_demo/` 零改动，见 §12）
```

---

## 1. 为什么必须换传感器（这条缺陷的量化证据）

单线雷达在 0.78 m 处只有**一个扫描平面**，顶面低于 0.78 m 的实体既不在图上、也不在代价地图里，
但物理上仍然是障碍 —— 规划器会直接从它中间穿过去，车只会"撞停"。

用孤立探针（1.5 m 外一个 0.40 m 高的矮箱）做的 A/B/C 实测，结论是决定性的：

| 高度带 | 有限距离数 | 正前方 (0°) | 含义 |
|---|---|---|---|
| A `[-0.75, +0.05]` 宽带 | 86 / 360 | 1.358 m | 看见矮箱（64 线投影的效果）|
| B `[-0.05, +0.05]` 单平面 | **0 / 360** | inf | 整圈全是 inf —— 单线雷达的盲区 |
| C `[-0.75, -0.30]` 只取低处 | 87 / 360 | 1.360 m | 证明箱体正好落在低处高度段 |

同一场景、同一台雷达，只把高度带收成一层薄片 → 什么也看不见。

---

## 2. 已完成：模型（第 1 步）

```
models/lighthouse_3d/          从 models/lighthouse 整份复制（19 MB，网格自带）
  model.sdf                    <model name="lighthouse_3d">
  robot.urdf                   <robot name="lighthouse_3d">
  meshes/                      未改，沿用 bases / sensors / wheels
```

几何（相对 base_link，base_link 在 base_footprint 上方 0.010 m）：

| 部件 | 单线版 | 64 线版 |
|---|---|---|
| 雷达头外形 | `lds.stl` 外壳 | Ø90 × 90 mm 圆柱 + 8 mm 顶盖（仿 Ouster OS1-64）|
| 雷达座 | 0.72 → 0.78 | 0.72 → **0.735**（变矮，给雷达头让位）|
| 传感器原点 | 0.780 | **0.780（不变）** |
| 整机最高点 | 0.78 m | 0.833 m |
| URDF `scan_joint` | `-0.064 0 0.770` | **不变** |
| 帧名 | `base_scan` | **不变**（ROS 约定，不是身份）|

传感器参数：

```
gpu_lidar  水平 360 线 / 垂直 64 线（-20° … +15°，共 35°），10 Hz
range      0.10 – 20.0 m，resolution 0.01，noise σ=0.005
gz 话题    /scan/points  → ignition.msgs.PointCloudPacked
           /scan         → ignition.msgs.LaserScan（gz 额外发的一圈，3d 模式下不要桥它！）
```

实测（探针世界，独立 IGN_PARTITION）：

```
点云      width=360 height=64 = 23,040 点/帧，point_step=32 B = 720 KB/帧
带宽      ≈ 7.0 MB/s @ 10 Hz
帧率      9.1 ~ 9.5 Hz（空世界 RTF ≈ 0.92）
ring      33 个不同值（64 圈里朝上的那半看不到东西 → inf，符合 -20°~+15° 视野）
矮箱命中  220 个点，z −0.502…−0.372，最近水平距离 1.356 m
```

---

## 3. 已完成：世界 + launch + 投影（第 2 步）

### build_world.py

```bash
python3 build_world.py              # → worlds/dan_house.world      （默认，与以前逐字节一致）
python3 build_world.py --lidar 3d   # → worlds/dan_house_3d.world
```

两个世界的房间/墙/家具/路线**完全相同**，diff 只差 include 那 2 行（`<name>` 与 `<uri>`）。
`route.json` 与 `semantics.json` 两边内容一致（世界几何没变）。房间几何自检仍是：
32 航点 / 最小净距 42 cm / 7 个门洞通道达标。

### dan_house.launch.py

新增 `lidar:=2d|3d`（默认 `2d`），一个开关同时决定四件事：

| | `lidar:=2d` | `lidar:=3d` |
|---|---|---|
| 世界 | `worlds/dan_house.world` | `worlds/dan_house_3d.world` |
| URDF / 网格 | `models/lighthouse` | `models/lighthouse_3d` |
| 桥接雷达话题 | `/scan` → LaserScan | `/scan/points` → PointCloud2 |
| 投影节点 | 不起 | 起 `pointcloud_to_laserscan` |

桥接被拆成 `bridge_2d` / `bridge_3d` 两个 Node（互斥条件），其余 8 条公共桥接抽成
`BRIDGE_COMMON` 共用。

### 投影参数（3d 模式）

```yaml
target_frame:     base_scan
min_height:      -0.75     # 相对 base_scan(0.78) → 地面以上 0.03 m
max_height:       0.05     # → 0.83 m，刚好不把雷达自己的顶盖(+0.053)算进来
angle_min/max:   ±3.14159265
angle_increment:  0.017453292519943295   # π/180 = 360 格，与单线雷达一致
scan_time:        0.1
range_min:        0.12     # 与 slam_toolbox min_laser_range 对齐
range_max:       12.0      # 与 slam_toolbox max_laser_range 对齐
use_inf:          true
```

**设计意图：下游输出仍然是一条 `/scan`** —— slam_toolbox / Nav2 / `mak_demo/` 智能体层一行都不用改。

---

## 4. 三个坑（都已在代码里写注释，别再踩）

1. **3d 模式绝不能桥 gz 的 `/scan`。** `gpu_lidar` 多线模式会额外发一条单圈 LaserScan，
   桥过来就和投影节点的 `/scan` 撞成两个 publisher，SLAM 会收到混在一起的两圈数据。
2. **QoS：投影节点是 `SensorDataQoS`（Best Effort）发布。**
   - RViz 的 LaserScan 显示项已经是 Best Effort（`rviz/dan_house.rviz:140`）→ 不用改
   - slam_toolbox / Nav2 代价地图都用 sensor-data QoS → 兼容
   - 自己写探针订阅 `/scan` 时**必须**用 `qos_profile_sensor_data`，否则报
     `offering incompatible QoS ... RELIABILITY` 且一帧收不到
3. **`models/` 下没有 `model.config`**，所以世界里的 include 必须用
   `<uri>file://<绝对路径>/model.sdf</uri>`（主世界一直这么写），
   `model://lighthouse_3d` 会报 `Unable to resolve uri ... does not contain a model.config`。

顺带记两个环境事实：`ign gazebo -s -r --headless-rendering` 在无显示环境下会打
`libEGL warning: egl: failed to create dri2 screen`，但 gpu_lidar 仍正常出点云（可忽略）；
`ros2 run` 会再包一层进程，`terminate()` 杀不掉真正的节点，探针里要用
`start_new_session=True` + `os.killpg`。

---

## 5. 怎么跑（验收命令，已实测）

```bash
cd /home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash
ros2 launch ./dan_house.launch.py lidar:=3d slam:=true rviz:=true
```

另开终端自查：

```bash
ros2 topic info /scan        # Publisher count 必须是 1
ros2 topic hz /scan/points   # 点云，wall 帧率 ≈ 10 × RTF
ros2 topic hz /scan          # 投影输出，应与点云同频
ros2 topic echo /scan --once --field ranges | head -3    # 应有有限值，不是全 inf
```

RViz 里应看到：塔顶 Ø90 mm 的大雷达头；LaserScan 是"贴地一圈 + 家具轮廓"，
而不是原来那种只有一面墙一根线的样子。

**要记的数**：RTF（原来单线版 0.34~0.58）、`/scan` 帧率、CPU 占用（预计点云 7 MB/s 是主要新增负载）。

### 存图别覆盖旧图

```bash
ros2 launch ./dan_house.launch.py lidar:=3d explore:=true rviz:=true loc:=truth \
  explore_map_prefix:=dan_house_lighthouse3d_explore
```

---

## 6. 第 4 步预告：把抬高的家具退回去

单线版为了"看得见"，把 13 件家具抬到了 0.86 m（`build_world.py` 里 9 个生成函数都加了
`h=` 参数）。**64 线雷达看得见 0.03~0.83 m 的所有东西，所以这些高度可以全部回退**：

| 物体 | 房间 | 真实高度 | 现在（为单线雷达抬高） |
|---|---|---|---|
| 茶几 | 客厅 | 0.40 | 0.86 |
| 边桌 | 客厅 | 0.69 | 0.86 |
| 餐桌 | 厨房 | 0.75 | 0.86 |
| 床头柜 ×2 | 卧室 | 0.50 | 0.86 |
| 浴缸 | 卫生间 | 0.56 | 0.86 |
| 马桶 | 卫生间 | 0.78 | 0.86 |
| 垃圾桶 ×3 | 走廊 | 0.60 | 0.86 |
| 灭火器 | 走廊 | 0.52 | 0.86 |
| 木箱 2/3 | 储藏间 | 0.70 / 0.60 | 0.86 |

做法：`build_world.py` 里把调用处的 `h=0.86` 去掉（回到函数默认值＝真实高度），
重新生成**两个**世界（2d 世界的家具也会随之变矮 → 2d 的旧地图更加对不上，
但 2d 路径本来就已冻结、只作对照）。占地尺寸/位置/门洞不变，所以路线净距仍应 42 cm。

---

## 7. 冻结/保留清单（已核对，未改）

```
tour_node.py                     10-20 17:21   未动
models/lighthouse/model.sdf      10-23 18:44   未动
models/lighthouse/robot.urdf     10-23 19:37   未动
worlds/dan_house.world           与改动前 diff -q 一致
config/maps/dan_house_lighthouse*.pgm/.yaml    未动
config/maps/preserved/*                        未动
config/route.json / semantics.json             重新生成但内容一致（世界几何未变）
```

备份：`config/maps/preserved/dan_house.world.bak_pre_raise`（抬家具之前）、
`~/.hermes/cache/scratch/dan_house.world.before_step2`（加 --lidar 之前）。

---

## 8. 复用的探针（临时文件，仍在）

```
~/.hermes/cache/scratch/probe3d.world             孤立探针世界（机器人 + 0.40 m 矮箱）
~/.hermes/cache/scratch/probe_cloud_box.py        统计点云里的矮箱命中/ring 分布
~/.hermes/cache/scratch/probe_projector_band.py   投影高度带 A/B/C 实测
~/.hermes/cache/scratch/eval_launch_branches.py   静态求值 launch 的 2d/3d 两条分支
~/.hermes/cache/scratch/probe_furniture_in_map.py 数家具足迹内的占用格（验收用）
~/.hermes/cache/scratch/probe_interior_coverage.py 只算房子内部的覆盖率（公平口径）
~/.hermes/cache/scratch/probe_unknown_blobs.py    未知区连通域分析（判断"没走到"还是"墙外"）
~/.hermes/cache/scratch/probe_map_obstacle.py     某 x 处障碍从墙往走廊凸出多少
~/.hermes/cache/scratch/probe_ascii_corner.py     小区域 ASCII 占用图（两图并排）
```

---

## 9. 第 3/4 步实测结果（2026-10-24，无头 + slam，独立 domain/partition）

### 9.1 链路

```
世界            dan_house_3d.world
桥接            只桥 /scan/points（PointCloud2）；绝不桥 gz 自带的 /scan
/scan 发布者     1（只有 pointcloud_to_laserscan，没撞话题）
/scan            frame_id=base_scan  360 格  359/360 有回波  min 0.978  max 9.020
TF map→base_scan Translation [-0.064, 0.000, 0.780]（与模型几何一致）
slam_toolbox    Registering sensor: [Custom Described Lidar]
```

### 9.2 64 线 vs 单线同条件对照

```
                    2d 单线     3d 64线    说明
RTF（无 Nav2）       0.99       0.98      几乎没有额外代价
RTF（带 Nav2+Slam）   —         0.85      点云 7 MB/s 的主要开销在这里
/scan 帧率          9.90 Hz    8.50 Hz    -14%
/scan/points         —          8.40 Hz    23,040 点/帧、32 B/点、720 KB/帧
正前方              8.956 m    8.952 m    差 4 mm
左侧                1.432 m    1.431 m    差 1 mm
右侧                1.085 m    1.083 m    差 2 mm
```

**关键结论**：单线够用的地方，投影出来的结果几乎一模一样 → 这层投影是忠实的，
下游 SLAM/Nav2/智能体层不需要任何参数改动。

高度带 A/B/C（1.5 m 外、0.40 m 矮箱）：

```
A  min_height -0.75 / max_height +0.05   有限 86/360，正前 1.358 m   ✅ 在用
B  min_height -0.05 / max_height +0.05   有限  0/360，全 inf           ← 单平面等价
C  min_height -0.75 / max_height -0.30   有限 87/360，正前 1.360 m
```

### 9.3 家具可见性（`probe_band_coverage.py`）

```
                          2d 世界(家具0.86)   3d 世界(真实高度)
穿过 0.78 平面                96                87
只有 64 线看得见               0                 9   ← 茶几/木箱/灭火器/垃圾桶×3/浴缸/边桌
在平面上方（窗户/挂画）        28                28
```

### 9.4 地图级验收（`probe_furniture_in_map.py`，数足迹内占用格）

```
物体                    3D 64线图   2D 单线图
茶几 0.40 m 客厅          174 格       0 格   ← 决定性对比
木箱3 0.60 m 储物间        41          0
木箱2 0.70 m 储物间        25          0
边桌 0.60 m 客厅           18          0
黄垃圾桶 0.63 m 储物间       5          0
其余 4 件                 >0          >0（受相邻墙体干扰，不作为判据）
```

原因：`dan_house_lighthouse.pgm`（10-23 17:26 建）用的是家具还矮着的世界，
单线雷达在 0.78 m 平面上什么都扫不到，那几件家具的位置在地图上就是纯空闲格。

---

## 10. 踩过的坑：假边界乒乓（已修，2026-10-24）

### 现象

续跑一轮 80 个目标，**76 个是同一对坐标**：

```
(+6.78, -2.45) ×38
(+5.13, -2.90) ×38
地图产出：与上一轮逐格相同（75.1% / 18.3% 未知）→ 22 分钟 100% 空转
```

### 根因

`blacklist` 只在**失败**时追加，成功到达不加。储物间那两块 frontier 是"假边界"
（车走到了、那里也确实没剩下未知区，但 frontier 检测仍认为它是边界），
于是被无限重选，形成两点乒乓。

### 修法（`frontier_explore.py`，4 处约 20 行）

1. `find_frontiers()` 里维护 `self.known_n = 空闲格 + 占据格`
2. `send_goal()` 记下 `self.known_at_goal = self.known_n`
3. 成功到达（status=4）→ `self.pending_gain = (时刻, known_at_goal, 目标坐标)`
4. `_tick()` 里 **延迟 4 秒**核对：`gain = known_n - known_at_goal`
   - `gain < --min-gain`（默认 40 格）→ 判假边界，拉黑，打 WARN
   - 否则打 `信息增量 +N 格`

**为什么必须延迟**：结果回调一到就比对的话，SLAM 还没把车在那儿看到的新格子写进
`/map`（地图 2 s 一帧），会把所有真边界都误判成零增量 → 全图拉黑、探索立即停死。

### 验证

```
修复后（explore3）：40 个目标，重复坐标 0 次
  目标 #10 (+3.13,-2.10) → 信息增量 +256 格  ✅ 真边界
  目标 #11 (-0.27,-4.55) → 信息增量 +77 格   ✅
  目标 (-7.92,-0.41)     → 只涨 29 < 40 → 假边界，拉黑
  目标 (-8.37,-0.81)     → 只涨 12 < 40 → 假边界，拉黑
```

---

## 11. 地图盘点 + 进度条

### 11.1 内部覆盖率（只算房子内部，排除墙外永远扫不到的边缘）

```
                                        内部空闲  内部墙  内部未知
2D 巡游图 dan_house_lighthouse            90.7%   3.9%    5.3%  (10.0 m²)
2D 探索图 dan_house_lighthouse_explore    89.1%   4.9%    6.0%  (11.1 m²)
3D explore   (120 目标)                   84.7%   5.5%    9.8%  (18.3 m²)
3D explore3  ( 40 目标，修复后)            85.6%   6.6%    7.8%  (14.6 m²)
3D explore4  ( 40 目标，净空 0.50)         85.6%   7.2%    7.3%  (13.5 m²)  ← 当前最佳
```

3D 图的"墙"比例更高（7.2% vs 4.9%）是**好事**：64 线看见了单线看不见的家具边缘。
剩余 13.5 m² 未知主要是家具背后的阴影口袋和几个墙角，每补一轮 40 目标只再换 1.1 m²，
收益已经递减。

### 11.2 地图文件

```
config/maps/dan_house_lighthouse3d.pgm/.yaml                     ← 3D 版正式图（= explore4）
config/maps/dan_house_lighthouse3d_explore{,2,3,4}*              ← 各轮原始产物 + 检查点
config/maps/preserved/dan_house_lighthouse3d_explore_120g_2026-10-24_1135.*
config/maps/preserved/dan_house_lighthouse3d_explore2_80g_pingpong_2026-10-24_1451.*
config/maps/preserved/dan_house_lighthouse3d_explore3_40g_2026-10-24_1606.*
config/maps/preserved/dan_house_lighthouse_explore_83pct_2026-10-23_1955.*
```

### 11.3 进度条工具（`tools/explore_progress.py`）

```bash
# 一次性
python3 tools/explore_progress.py --prefix <前缀> --log <日志> --max-goals 40 --max-minutes 25
# 实时刷新（纯只读查看器，Ctrl-C 退出不影响探索）
python3 tools/explore_progress.py --watch --prefix <前缀> --log <日志> \
  --max-goals 40 --max-minutes 25 --fallback-prefix <上一轮前缀>
```

输出三条独立口径（故意不合并成一个假数字）：

```
[████████░░░░░░░░░░░░░░░░░░]  30%  目标 12/40
覆盖率   76.2% 空闲 · 8.5% 墙 · 15.2% 未知   最新图 ..._checkpoint_08.pgm（1.0 min 前）
用时     5.1 min 墙钟 ≈ 4.6 min 仿真（RTF 按 0.90 折算）  30.3 s/目标  失败 8
预计     还需 ≈ 14 min（按目标数）· 仿真上限还剩 ≈ 20 min（谁先到谁停）
```

### 11.4 已知待办

``` 
· 卡死看门狗已做 A/B（2026-10-24）：12 s vs 18 s 各 25 目标，空地图起步
    12 s → 到达 22 / 卡死 2 (8%) / 内部未知 45.0 m²
    18 s → 到达 20 / 卡死 4 (16%) / 内部未知 56.5 m²
  被杀目标在 12~18 s 内位移 ≤0.2 m → 真卡住，不是误杀 → **保持 12 s**
· 剩余内部未知 13.5 m² 主要是家具背后的阴影口袋；每补一轮 40 目标只再换 ~1 m²，收益递减
· 真正值得改的是目标可达性（贴墙/贴死角剔除），不是看门狗计时器
```

---

## 12. 第 5 步：2A/2B 在 3D 图上复验（2026-10-24）

雷达与地图换成 3D 版后，把阶段 2 的验收整套重跑一遍 —— **`mak_demo/` 一行没改**。
原因：3D 版最终仍发 `/scan`（LaserScan，360 格）、帧名 `base_scan`、`scan_joint z=0.770`、
Nav2 参数全部不变，所以智能体侧感知不到"换传感器"这件事。

启动（与 `docs/phase2A_results.md` §7 完全一致，只换地图与 lidar）：

```bash
ros2 launch ./dan_house.launch.py lidar:=3d nav:=true loc:=truth headless:=true \
    map:=config/maps/dan_house_lighthouse3d.yaml
PATH=/usr/bin:$PATH ros2 launch rosbridge_server rosbridge_websocket_launch.xml
/usr/bin/python3 agent_bridge.py        # 系统 python，不是 venv
```

| 2A 项 | 3D 图实测 | 判定 |
|---|---|---|
| 1a 链路 `check_link.py` | `/agent/pose` 25 帧、`nav_status` 11 条 | ✅ |
| 1b 定位 `check_loc.py` | 偏差 0.00 m / 0.0° | ✅ |
| 2 硬编码 3.0 m | succeeded 57.0 s，终点 [2.843,0.091,0.265]，恢复 0 | ✅ |
| 3「带我去冰箱看看」 | 39.4 s 到观察站位 1.79 m + VLM 58.5 s，四项核查全绿，总 131.4 s | ✅ |
| 4「洗手间在哪」 | 41.7 s，未发车 | ✅ |
| 4b「带我去地下室」 | 0.0 s 正确拒绝 | ✅ |
| 5「先去客厅，然后到冰箱旁边」 | 91.6 s + 58.0 s，两段 succeeded，恢复 0 | ✅ |
| 离线 5 项 + `check_standoffs` | 全通过，61/61（53 在房间内） | ✅ |

| 2B 项 | 3D 图实测 | 判定 |
|---|---|---|
| 「去客厅」 | succeeded 55.4 s，剩余 0.242 m，恢复 0 | ✅ |
| 「冰箱还在吗」（从客厅开回） | 54.6 s，VLM 3.5 s「双开门白色立式柜体」一致，四项核查全绿 | ✅ |
| `check_passability.py` / `check_vlm_guard.py` | 19/19 / 3/3 | ✅ |

启动时踩到一个坑：rosbridge 被 Hermes 自带的 python 3.14 拉起 → `ModuleNotFoundError:
No module named 'rclpy._rclpy_pybind11'`。ROS 节点只能用系统 3.10，启动前 `PATH=/usr/bin:$PATH`。

文档落点：`README.md` §17（雷达开关与全部数据）、`docs/phase2A_results.md` §10、
`docs/phase2B_vision.md` §13、`STATUS.md`（一页现状）。
