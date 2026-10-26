# STATUS —— 第二阶段进度与恢复手册

> 最后更新：2026-10-21 晚（休息前）
> 一句话现状：**链路和语义都通了，导航卡在定位上 —— 已改好未验证，回来跑四条命令就能接着走。**

---

## 1. 已完成并**实测通过**的

| 关卡 | 内容 | 证据 |
|---|---|---|
| 环境 | `mak_demo/.venv`（Python 3.10.12，863 MB），`agent-framework==1.22.0` 全部依赖在里面 | `pip show agent-framework` → 1.22.0 |
| LLM | 本地 ollama `qwen2.5:7b`，OpenAI 兼容端点，**不需要密钥** | `dev/smoke_agent.py` → 模型真的调用了工具，回答里带出工具里的坐标 |
| 语义地基 | `data/lexicon.json`（33 类物体·中文正名+别名）+ `semantic_map.py` | `dev/check_standoffs.py` → **61/61 物体有可站位姿**，60/61 在房间内 |
| 中文解析 | 不用 LLM 就能落点 | "带我去厨房"→厨房；"去卧室2的床边"→bedroom_2+bed_2；"洗手间"→卫生间 |
| **关卡 1 · 链路** | rosbridge + `agent_bridge.py` + agent 侧可读 | `dev/check_link.py` → `/agent` 四话题齐、pose 24 帧、`state=idle` ✅ |
| **关卡 2 · 开车** | 发目标、看进度、判到达 | `dev/test_nav.py` → `succeeded`，剩余距离 2.85→0 单调降，0 恢复，22.4 s |

---

## 2. 卡在哪（**已修好，等验证**）

### 2.1 当时的判断：AMCL 在这张探索图上跳了 124°（结论已更正，见下）

> **更正（2026-10-26）**：这一节记录的现象当时是真的，但"真因"这个判断错了 ——
> Dan 确认那次跳变是他**人为设置仿真**时造成的，不是 AMCL 或本项目的缺陷。
> 结论（仿真里用 `loc:=truth`）不受影响；但不要再把 AMCL 当成待排查对象。
> 保留下文是因为"用 /odom 与 TF 对比、而不是去读 Nav2 日志"这个方法仍然有效。

```
/odom        = (-1.5959, +0.3690)          ← 与仿真真值误差 < 1 mm（仿真里 odom 就是真值）
map→odom     = 平移(0.950, 1.055) + 旋转 124.5°   ← AMCL 强加的"修正"
TF map→base  = (+1.549, -0.470)            ← 整体偏了 3.25 m
```

后果链：规划器按错位姿规划 → `剩余 7.2 m`（实际 4.6 m）→ 车朝错误门洞开 →
厨房门口反复恢复 → 看门狗 12 s 判 `stalled` 取消。
语义解析和可站位姿在这期间**都是对的**，所以这不是语义层的问题。

AMCL 无法自愈：`recovery_alpha_fast/slow = 0.0`（禁用随机粒子注入）+
`always_reset_initial_pose: false`（错误位姿被 latch 住）。

### 2.2 已做的三处修改（**都还没跑过真机**）

| 改动 | 文件 | 说明 |
|---|---|---|
| 新增 `loc` 参数 | `dan_house.launch.py` | `loc:=amcl`（默认，真实机器人）/ `loc:=truth`（静态 `map→odom` 恒等 + `map_server` + `navigation_launch`，**不起 AMCL**）。已通过 `py_compile` |
| 残留状态判定 | `mak_demo/nav_client.py` | 只接受"发布之后收到的"状态，且 `goal` 坐标匹配；8 s 内没有就返回 `rejected`。离线测试 2/2 通过 |
| 定位预检脚本 | `mak_demo/dev/check_loc.py` | 3 秒比对 `/odom` 与 `map→base`，偏差 > 0.3 m 或 > 15° 就报错，**开车前必须过** |

### 2.3 已定位但**故意没改**的（避免一次改多个变量）

- 可站位姿只校验"0.30 m 内无墙"，而 Nav2 的 `inflation_radius = 0.45`。若 `loc:=truth` 后
  仍出现目标点不可达，再把 `semantic_map.py` 的 `CLEAR_M` 从 0.30 提到 0.40~0.45。

---

## 3. 回来照着敲（三个终端）

### 终端 A —— 仿真 + Nav2（**注意 `loc:=truth`**）

```bash
cd /home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash
bash preflight.sh
ros2 launch ./dan_house.launch.py nav:=true rviz:=true loc:=truth \
    map:=config/maps/preserved/dan_house_explore_FINAL_2026-10-21_1800.yaml
```

### 终端 B —— rosbridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 终端 C —— 桥接节点（系统 python，不是 venv）

```bash
cd /home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash
/usr/bin/python3 agent_bridge.py
```

### 终端 D —— 两道闸门，按顺序过

```bash
cd /home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot/mak_demo

# 闸门 1：定位必须一致（偏差 0.00 m / 0.0°）
.venv/bin/python dev/check_link.py
.venv/bin/python dev/check_loc.py

# 闸门 2：语义 + 导航串起来
.venv/bin/python dev/test_semantic_nav.py 冰箱
.venv/bin/python dev/test_semantic_nav.py 沙发
```

**闸门 1 不过就别跑闸门 2** —— 定位错的时候，下游每个数字（路径长度、剩余距离、"已到达"）
都是在错误坐标系里算的。

---

## 4. 验收标准

| 闸门 | 通过条件 |
|---|---|
| `check_loc.py` | 偏差 < 0.30 m 且 < 15° |
| `test_semantic_nav.py` | ① 打印出房间/物体/位姿/理由 ② `succeeded` 且恢复次数 0 ③ **`|d_final − d_plan| ≤ 0.30 m`（相对规划站姿，有物理依据：`xy_goal_tolerance` 0.20 + 余量 0.10）且 `d_final ≥ 物体半长 + 0.15`** |
| 时间预期 | 走廊东头到厨房约 10 m，实际耗时 90 秒上下（RTF≈0.5，别看百分比看剩余距离） |

**不要用绝对区间判落点。** 曾经用 0.35~1.1 m 的硬区间，冰箱站姿 0.94 m + 正常偏差 0.17 m = 1.11 m，
正确的一趟被判失败；沙发站姿本身就是 1.80 m（沙发 1.0×2.5 很大），又被 `d ≤ 1.8` 的绝对上限误杀。
两次都是"尺子刻错"而不是车的行为错 —— 判据必须挂到规划站姿上。

---

## 5. 卡过你的坑（都已修，别再踩）

1. **残留终态假成功**：发完目标立刻读缓存状态，会读到上一个目标的 `succeeded` ——
   实测 `[0.0s] succeeded 剩余 0.21 m`，车一步没动。而且**重复去同一个地方时坐标也相同**，
   所以只有"接收时间戳"能区分新旧。离线用例在 `dev/test_nav_stale.py`（2/2 通过）。
2. **`xy_goal_tolerance = 0.20`**：Nav2 在离目标 20 cm 内就报 SUCCEEDED。
   所以"停在物体旁"必须按机器人↔物体距离判定，不能只看状态字符串。
3. **墙钟速度 ≠ 配置速度**：2.8 m 用 22.4 s = 0.125 m/s，但参数是 0.26 —— 因为 RTF≈0.5。
   参数是生效的。
4. **目标朝向**：不要写死 `orientation.w=1.0`，否则到点后要原地转，万向球打滑 + 触发恢复。
   用 `atan2(gy-ry, gx-rx)`（探索阶段踩过 187 s/目标，改成朝向后 12~15 s/目标）。
5. **不要用 `python -c` / heredoc 做探针**：在这个环境里会触发审批并静默超时。写成文件再跑。
6. **`AndSubstitution` 只吃两个位置参数**：`AndSubstitution([a, b])` 直接
   `TypeError: ... missing 1 required positional argument: 'right'`，launch 根本起不来。
   合成条件写一个 `PythonExpression`（条件接受任何给出 `true`/`false` 的 substitution）。
7. **DWB 默认不能倒车**（`min_vel_x: 0.0`）：车头朝里挤进家具凹槽后只能原地转，
   转也转不动 → 满速指令 + `位移 0.02 m` + **`recoveries` 仍然是 0** + 局部代价地图 164 个致命格。
   判据很好认：`/cmd_vel` 有满速、车不动、恢复次数为 0（不是规划器失败，是本体动不了）。
   修：`min_vel_x: -0.15` + `max_vel_x_backwards: 0.15`，并且**别再把站位放在两件家具之间**。
8. **地图上"空"不等于能过**：椅子腿/桌腿在 0.132 m 高的 2D 雷达图上很稀疏，
   地图看着通的缝，车开进去撞实体。站位必须同时避开**其他物体包围盒**（`obj_clearance`），
   按 `[0.45,0.35] → [0.40,0.25] → [0.32,0.15] → [0.30,0]` 逐级放宽，保证 61/61 仍有解。

---

## 6. 进度（2026-10-22 中午更新）

> 完整的实测报告在 `docs/phase2A_results.md`：六步验收数据、架构演进的反证、
> 模型选型表、三个核心 bug 的机理与复现命令。本文件偏向"怎么恢复和怎么排错"。

| 步骤 | 内容 | 状态 |
|---|---|---|
| 1a | 链路（rosbridge + `/agent/*` 四话题） | ✅ 通过 |
| 1b | 定位一致性（`check_loc.py` 偏差 0.00 m / 0.0°） | ✅ `loc:=truth` 下通过 |
| 2 | 硬编码目标 3 m → succeeded 22.4 s | ✅ 通过 |
| 3 | 语义 → 导航：冰箱 12.9 m/88.8 s/偏差 0.13 m，沙发 9.5 m/108.6 s/偏差 0.12 m，**均 0 恢复** | ✅ 通过 |
| 4 | `tools.py` + `agent_nav.py`：中文指令闭环（确定性主导 + LLM 汇报） | ✅ 通过（三条全绿） |
| 5 | 复合指令 `planner.py` + `main.py`（串行多目标 / `--interactive`） | ✅ 串行模式已实跑通过（见下） |
| 6 | **① 相机 / VLM 视觉确认**（`vision.py` + `look_at` + 模式 D） | ✅ 通过（见 §6.1） |

### 6.1 步骤 6（① 相机/VLM）通过 —— 2026-10-22 夜

完整记录见 `docs/phase2B_vision.md`。要点：

- **关键改动不是换模型，是把相机从离地 0.117 m 抬到 0.60 m**。
  0.117 m 时在 0.55~1.41 m 处只能框到冰箱面板下部 0.57 m 高的一带，
  门缝(1.15)/把手(0.75、1.45)全在画外 → VLM 把冰箱判成"房门"（两帧一致）；
  抬到 0.60 m 后同一模型同一句话答"冰箱，依据：竖把手 + 门缝"。
- **观察站位**（`OBSERVE_GAP_M=1.40`，距中心约 1.8 m）用于"看看 X"，
  贴到旁边（0.55 m）仍用于"到 X 旁边"。两者共用同一套环形采样与净空检查。
- **`look_at` 工具 + 模式 D**：`看看 / 看一眼 / 还在吗 / 确认` + 能解析到物体 → 退到观察站位 + VLM 核对。
  `classify_intent` 顺序修过一次：先判"问位置"，再判"要动手/要看"，否则 `冰箱还在吗` 掉进 ASK、车不动。
- **VLM 只做确认，不做理解**：封闭四选一 + `num_predict=90`。
  开放式"描述画面"会让 8B 编造场景（"床铺、卧室"），耗时也从 2.5 s 涨到 20~41 s。

实跑数据：

```
main.py "去客厅"        剩余 11.73 → 0.178 m，79.0 s，0 恢复
main.py "冰箱还在吗"    剩余 11.79 → 0.23 m，75.2 s → look_at
                        VLM 10.1 s「白色带门把手分层结构」→ 与语义地图一致
                        核查①②③④ 全绿
dev/test_look_at.py fridge_1   4.7 m / 54.7 s；VLM 11.1 s；一致 ✅
```

相机链路实测：原生 7.1 Hz、320×240 rgb8、raw base64 300 KB/帧（2.36 MB/s）、
**转 PNG 后 58 KB**（小 5 倍，VLM 输入一律用 PNG）、VLM 单次 10~26 s。

### 6.2 可通行性问答（模式 E，2026-10-22 深夜加）

物体身份识别受取景几何限制，但"正前方能不能过去"落在雷达与相机的互补区间：
雷达给几何硬约束（墙/家具，毫秒级），相机补雷达在 0.132 m 平面上看不见的东西
（贴地线缆、门槛、地毯边、玻璃）。

```
passability.py    正前方 ±25° 扇区量测 → blocked / tight / partial / open / no_return
                  判据用 free_until = 最近障碍 − 0.21（车体半径）
                  加 partial 档的原因：1.5 m 外有墙若判"可以通过"是误导（能走 1.29 m 就撞上）
vision.assess_passability()   封闭三选一：畅通 / 被挡住 / 看不清
tools.check_path()            双源合并：雷达优先，视觉补盲区，不一致时如实报"两者说法不一样"
模式 E                        "能过去/通不通/挡住/有障碍/门开着" → 只用传感器，不开车
```

离线验收：`dev/check_passability.py` **19/19**（12 种合成扫描 + 7 条指令路由）、
`dev/check_vlm_guard.py` **3/3**（VLM 空回答的防护）。

> 首跑抓到两个真 bug，都已修 + 补回归用例：
> ① 雷达扫到车壳（0.13 m @ -25° 落在车身内）→ 按车体矩形几何剔除，不再靠 `MIN_R` 阈值；
> ② qwen3-vl 是 thinking 模型，`num_predict=80` 被思考吃光导致 `content=''`（18.1 s 白等）
>    → 预算提到 600 + 截断重试 + 从 thinking 末尾兜底；
>    `think:false` 与 `/no_think` 在这版 ollama 上**都无效**，别再试。

> 这一条把视觉用在了它真正擅长的地方，而不是硬让它认物体。

### 6.3 车体塔身改造（2026-10-23，扫描平面 0.132 → 0.78 m）

**根因**（Dan 指出）：为了让相机到 0.60 m，我加的那根 `r=0.012`、只有 visual 没有 collision
的立杆，span 是 0.13~0.60 m —— 底端正好插进雷达 0.132 m 的扫描平面。实测自身回波
0.124~0.138 m @ -24°~-31°，换算到车体系 `(0.054,-0.055)`，就在杆边上 2 mm。
所以"雷达扫到自己"的元凶是杆，不是车壳；`MIN_R=0.12` 只是恰好放行了 0.124。

**自洽性论证**：任何从车体升到 0.6 m 的支撑都必然穿过 0.132 m 平面 →
"低雷达 + 高相机"在机械上不成立 → 必须改结构，不能靠软件擦回波。

**新结构**（`models/lighthouse/model.sdf`，挂在 base_link 上，不新增 link）

```
塔身   cylinder r=0.055, z=0.10→0.72   有 collision，真结构
雷达座 cylinder r=0.040, z=0.72→0.78
相机   (-0.004, 0, 0.70)  挂在塔前，低于雷达；支架 box 0.058×0.050×0.040
base_scan  (-0.064, 0, 0.78)   360° 平面上无车体部件（塔座/支架都在 0.12 m 死区内）
静态 TF    base_link→camera_link = (-0.004, 0, 0.690)
幽灵杆 mast_visual 已删除
```

**世界跟着改**（`build_world.py`，只改高度，不动位置/进深/门洞/间距）
`stool -4.60 → 0.90`、`stool_2 -3.40 → 0.92`、`umbrella_stand -5.00/+4.40 → 1.10`、
`shoe_cabinet -8.00 → 0.86`、`shoe_cabinet_2 +2.60 → 0.86`、`console_table_3 +7.00 → 0.86`。
验证：`dev/probe_slice.py` → **走廊地标 12/12 存活**（改造前 7/12）。

**代价（Dan 已确认接受）**：可见物体 116/135 → 85/135；低矮障碍（茶几 0.40、垃圾桶 0.63、
灭火器 0.61、边桌 0.69、浴缸 0.69、木箱 0.60）不再进代价地图 → 记为已知限制；
**旧地图作废，必须重做**（新的写 `dan_house_tower.*`，旧三张图一个字节不动）。

详见 `docs/phase2B_vision.md` §13。

**更名（2026-10-23）**：这台车改名 **Lighthouse（灯塔）**，不再叫 TurtleBot3。
`models/turtlebot3_waffle/` → `models/lighthouse/`；URDF `<robot name>`、world 里的 include、
launch 变量（`TB3_URDF` → `LH_URDF`）全部跟着改；网格复制进项目自己的 `meshes/`，
SDF 用 `model://lighthouse/meshes/...`、URDF 用 `$(arg mesh_dir)/meshes/...`，
不再依赖 `turtlebot3_description` 的安装位置。**frame 名保持 ROS 约定不变**
（`base_link / base_scan / camera_link / imu_link / wheel_*`）。
详见 `docs/phase2B_vision.md` §13.8。

### 6.4 2026-10-22 收工点（历史记录，供追溯）

**已完成并真机验收**

- ① 相机 / VLM 视觉确认（模式 D）：
  `去客厅` 11.73→0.178 m / 79.0 s → `冰箱还在吗` 11.79→0.23 m / 75.2 s + VLM 10.1 s，
  核查①②③④ 全绿；`dev/test_look_at.py fridge_1` 端到端 4.7 m / VLM 11.1 s。
- 相机几何修复：`camera_link` 0.104→0.60 + 纯视觉立杆 + `<gz_frame_id>=camera_link` +
  launch 里 4b 静态 TF（0.064,-0.065,0.590）+ RViz Camera 显示项（属性名是 `Topic`）。
- 语言层：`dev/bench_language.py` 10/10（代码注入方位/距离后）；`vision.full_options` 覆盖 33/33。
- 文档：`docs/phase2B_vision.md`（12 节 / 460 行）、技能库 `references/visual-confirmation.md` 增 §6/§7。

**代码完成、离线 16/16 通过，但真机还没跑**

- 可通行性问答（模式 E）：`passability.py` + `vision.assess_passability()` + `tools.check_path()`。

**下次第一件事**（仿真在跑 + 终端 C 有 `agent_bridge.py`）

```bash
cd /home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot/mak_demo

# ① 当前位姿（正对冰箱 1.4 m）→ 期望 partial + 视觉「被挡住」，两者一致
.venv/bin/python main.py "前面能过去吗"

# ② 空通道 → 期望 no_return + 视觉「畅通」→ ✅ 可以通过
.venv/bin/python dev/snap_at.py 0 0 0
.venv/bin/python main.py "前面能过去吗"
```

两条都过 → 按 `docs/phase2B_vision.md` §12.6 的三种朝向各跑一次收数据；
之后可选：两帧投票（去偶发误判）、② 抢占模式、③ AMCL 调参。

顺带做掉的**语言层基准**（`dev/bench_language.py`，10 条固定用例）：

| 条件 | 得分 |
|---|---|
| 工具只返回坐标 | 7/10，且两个不同问题给出一字不差的同一句（模板化） |
| 代码算好方位/距离再喂进去 | **10/10**（同一 8B 模型） |

所以"要不要换 cloud 大模型"的答案是：**语言层不必换**（本地已满分为 10/10，汇报层 1.3~1.9 s），
换能买到的是延迟（问句 17.8~50.5 s → 秒级）与多轮理解；端点是配置项，
`--base-url / --api-key / ROBOT_LLM_*`，不改代码。

### 步骤 5 已交付（第一种模式：串行执行）

- `planner.py` — `split_segments()` 按 然后/接着/之后/再/，/； 拆句；
  `plan_sequence()` 逐段调 `plan_action()` 解析；`summarize()` 拼汇报文本。
  **拆句子是代码干的**，不交给 LLM（拆错就开错房间）。
- `main.py` — 统一入口：单目标/问句转交 `agent_nav.run_instruction`（A/B/C 三条路线不变），
  只有"多目标且至少一个能解析"才串行执行；`--interactive` 进 REPL。
- `dev/check_planner.py` — 离线验收 15/15（拆句、分流、汇报组装）。

顺带修掉一个离线测试抓到的 bug：`plan_action` 之前没把房间限定传给 `resolve_object`，
「到卧室2的床边」会解析成 `bed_1`（卧室1的床）；现在先算房间再限定物体 → `bed_2`。

**第二种模式（中途插话改主意）还做不到**，缺两处改造（见 `main.py` 文件头）：
① `nav_client.go_to()` 增加 `cancel_if=callable`，每 0.2 s 检查一次；
② 后台线程读 stdin 入队。先用 `--interactive` 磨多轮，再动这两处。

### 交互模式实跑（2026-10-22 夜，四条路径全过）

```
洗手间在哪            1 次 find_place，不发车，口语回答
带我去冰箱看看        118.1 s，11.1 → 0.17 m，0 恢复（从卫生间绕过来）
先去客厅，然后去卫生间  141.5 s + 99.6 s，两段都 succeeded，0 恢复
                      答"客厅到了，卫生间也到了，都完成了。"
带我去地下室          0.0 s 拒绝，零 LLM
/cancel              空闲时正常打印提示；q 退出干净
```

修掉一个只影响交互模式的缺陷：每条指令 `asyncio.run()` 一次会打印
`RuntimeError: Event loop is closed`（MAF 的 httpx2 客户端**没有 close 接口**，
清理任务排在关循环之后，连接不回收）。改为**整个 REPL 一个事件循环**
（`asyncio.run(repl_async(...))` + `asyncio.to_thread(input, ...)`），并给 Ctrl+C
加了"退出前先发一次 `/agent/cancel`"，避免程序没了车还在跑。

新注意点（不是 bug）：窄走廊里 `distance_remaining` 会来回抖 ±0.5 m，是 DWB
反复重规划（探一步→发现要掉头→重算）。看门狗不会误杀，因为基线在持续变好。


跑法：

```
.venv/bin/python dev/check_planner.py            # 离线，秒级
.venv/bin/python main.py "先去客厅，然后到冰箱旁边"
.venv/bin/python main.py "去厨房，然后去地下室"
.venv/bin/python main.py --interactive
```

### 多目标串行实跑（2026-10-22 晚）

```
先去客厅，再到冰箱旁边
  目标 1/2 客厅中心   ✅ 70.0 s，剩余 0.228 m 收尾（Nav2 容差内）
  目标 2/2 冰箱       ✅ 146.7 s，0 恢复，终点 (-1.59,+5.10)，距中心 1.07 m（规划 0.94）
  回答：先去了客厅，然后到了冰箱旁边，两个地方都顺利到达了。

去客厅，然后去地下室
  目标 1/2 客厅中心   ✅ 88.0 s
  目标 2/2 地下室     ❌ 代码拒绝，这一步跳过（不占时间、不发车）
  回答：先去了客厅，到了；家里没有地下室这个地方，那一步没执行。
        ← 关键：完成的部分和没做的部分分开说，没有滑成"都顺利"
```

**这一步顺手逼出了桥接层一个只背靠背目标才出现的 bug**：第二个目标的看门狗
误判 stalled（13.1 s，可距离明明从 11.66 → 10.13 在下降）。根因是
`on_goal` 虽然把 `distance`/`best_distance` 置 None，但上一条目标的 feedback
在新目标注册之后才到，把 `best_distance` 又写成了残留的 0.22 m，此后真实距离
永远比不过它。已在 `agent_bridge.py` 三处修复：

1. 基线重置挪到 `on_goal_response`（目标被接受那一刻），而不是 `on_goal`；
2. `on_feedback` 只在 `state == 'active'` 时采信（挡掉旧目标的 feedback）；
3. 看门狗自愈：`distance > best + 1.0` → 判定基线被污染，以当前采样重新起算并打 warn。

单目标跑一百次都不会触发这个 bug —— 只有复合指令才会。


### 步骤 4 最终形态：确定性主导，LLM 只占语言边缘（2026-10-22 傍晚定稿）

```
指令
 ├─ classify_intent()   代码：祈使句 ACT / 问句 ASK
 ├─ ACT 且能解析出目标 → 代码直接发车 → reporter(无工具) 说一句人话
 ├─ ACT 但解析不到     → 代码直接拒绝：列出现有房间（不调 LLM，0.0 s）
 └─ ASK               → LLM 带工具查证 → 口语回答
```

**为什么"要不要动手"不能交给 LLM**：实测 qwen3-vl:8b 收到「带我去沙发看看」时只调
`find_place` 就答"沙发在客厅正中间"收工，车一步没动；加提示词、加 `tool_choice="required"`
都没改掉（ollama 的 OpenAI 兼容层直接忽略该字段）。而"该动没动"的代价是用户站旁边干等。
所以动作由代码保证，LLM 负责它真正擅长的：理解口语歧义、把结果说成人话。

模型分工（实测，`dev/check_report.py` 可复跑）：

| 用途 | 模型 | 热态耗时 | 备注 |
|---|---|---|---|
| 理解/查证（带工具） | qwen3-vl:8b | 20~45 s | 中文准、参数抄得对 |
| 汇报（无工具，只改写） | qwen2.5:7b | **1.9 s** | 同一任务 qwen3-vl 要 31.7 s，差 15 倍 |

四个候选模型的实测结论：

| 模型 | capabilities | 实测表现 |
|---|---|---|
| qwen3-vl:8b | completion+vision+tools+thinking | ✅ 中文准、工具参数正确；慢 |
| qwen2.5:7b | completion+tools | ❌ 工具模式下吐碎片乱码（"pora房间信息…"）；**做汇报却很好** |
| llama3.1:8b | completion+tools | ❌ 把「洗手间」抄成「汉测式」 |
| gemma3:12b | completion+vision | ❌ 没有 tools，无法 function calling |

三个反直觉的实测结论（都别再试）：

1. `tool_choice="required"` 在 ollama 的 OpenAI 兼容端点**无效** —— 传了不报错，也不生效。
2. `/no_think` 后缀同样**无效且更慢**（47~69 s vs 31.7 s）：它只有原生 `/api/chat` 认，
   走兼容端点时只是普通 token。
3. 模式 A 下第一次 LLM 调用就是汇报，冷加载 6.1 GB 要 ~20 s，别把冷启动算进"模型太慢"。

第四条不是参数问题，是行为问题 —— **模型在"该不该开口"上也会失手**：
「带我去地下室」走 LLM 那条路时它思考了 **53.8 s 却返回空正文**，用户拿到一片空白；
而"没乱开车"的核查依然是 ✅（因为确实没发车）。所以每条路线都要有代码级兜底断言：

- 目标解析不到 → 代码直接拒绝（0.0 s、不调 LLM），因为"家里有哪些房间"是我们自己的真值；
- 任何路线拿到空字符串 → 必须有兜底文案，绝不把空白交给用户。

实绩（三条全绿）：

```
带我去沙发看看   模式A  1 次 go_to  12.59→0.17 m  135.3 s  0 恢复  偏差 0.12 m  汇报 8.2 s
带我去冰箱看看   模式A  1 次 go_to  12.95→0.22 m   86.6 s  0 恢复  偏差 0.13 m
洗手间在哪       模式B  1 次 find_place，不发车，口语回答                25.5 s
带我去地下室     模式C  0.0 s，不调 LLM，不发车，直接列出现有房间
```

离线验收（都不连 ROS）：

```
dev/check_tools.py     工具输出格式、假 id 拦截、不存在地点    4/4
dev/check_intent.py    意图判定 8 例 + 执行计划 3 例           11/11
dev/test_nav_stale.py  残留终态时序                            2/2
dev/check_report.py    两个模型的汇报延迟对比                  —
```

跑法：

```
.venv/bin/python agent_nav.py "带我去冰箱看看"
.venv/bin/python agent_nav.py "洗手间在哪"
.venv/bin/python agent_nav.py "带我去地下室"
```


---

## 7. 文件清单

```
robot/agent_bridge.py              ROS 侧桥：NavigateToPose → /agent/* 话题（系统 python）
dan_house.launch.py                新增 loc:=amcl|truth

mak_demo/
├── .venv/                         MAF 1.22.0 + 依赖（不进版本库）
├── requirements.txt               216 行 pip freeze
├── README.md                      架构判断（单 vs 多智能体）+ 环境 + 铁律
├── STATUS.md                      ← 本文件
├── rosbridge_link.py              极简 rosbridge 客户端（publish/subscribe/service）
├── nav_client.py                  go_to / cancel / 残留状态判定
├── semantic_map.py                resolve_room / resolve_object / standoff_pose / obj_clearance
├── tools.py                       五个 function tools（坐标只来自工具，LLM 不能编）
├── agent_nav.py                   MAF Agent + ollama：一句中文进，开车 + 一句话出
├── data/lexicon.json              33 类物体的中文正名 + 口语别名
└── dev/
    ├── smoke_agent.py             MAF + ollama + 工具调用
    ├── probe_semantics.py         semantics.json 家底盘点
    ├── check_standoffs.py         61 物体位姿验收 + 口语解析自测
    ├── check_tools.py             离线验收 tools.py（假 nav，4/4）
    ├── check_link.py              闸门 1a：链路
    ├── check_loc.py               闸门 1b：定位一致性
    ├── test_nav.py                闸门 2a：硬编码目标
    ├── test_nav_stale.py          离线时序单元测试（不需要 ROS）
    ├── test_semantic_nav.py       闸门 2b：语义 → 导航
    └── diag_goto.py               带仪表复现：plan/cmd_vel/odom/局部代价地图四路同时记
```
