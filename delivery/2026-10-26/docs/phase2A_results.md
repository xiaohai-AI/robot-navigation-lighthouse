# 阶段 2A 结果报告：AI 智能体引导的语义导航

日期：2026-10-22　环境：ROS 2 Humble + Gazebo Fortress 6.18 + TurtleBot3 Waffle 类差速车
代码：`robot/mak_demo/`（MAF 1.22.0 + 本地 ollama）　地图：`dan_house_explore_FINAL_2026-10-21_1800`
定位：`loc:=truth`（仿真里 `/odom` 即真值，不必让 AMCL 去"修正"，见 §6.5）

---

## 0. 一句话结论

一句中文说出去，车真的开到那个家具旁边并如实汇报；复合指令能拆成多个目标串行执行；
目标不存在时 0.0 s 拒绝且不发车。

| 指令 | 结果 | 耗时 |
|---|---|---|
| `带我去冰箱看看` | 12.95 m → 到达冰箱旁 1.07 m（规划 0.94 m），0 恢复 | 86.6 s 行驶 + 8 s 汇报 |
| `带我去沙发看看` | 12.59 m → 到达沙发旁 1.92 m（规划 1.80 m），0 恢复 | 135.3 s + 8.2 s |
| `洗手间在哪` | 答"洗手间在走廊南边…"，不发车 | 25.5 s |
| `带我去地下室` | 答"家里没有叫地下室的地方"并列出现有房间，不发车 | 0.0 s |
| `先去客厅，然后到冰箱旁边` | 两段都到达（70.0 s + 146.7 s），0 恢复 | 216.7 s + 汇报 |
| `去客厅，然后去地下室` | 客厅到达（88.0 s）；地下室由代码拒绝，一步跳过 | 88.0 s |

---

## 1. 系统构成

```
            ┌──────────────────── agent 侧（mak_demo/.venv，纯 WebSocket）────────────────────┐
中文指令 → │  classify_intent  →  plan_action  →  go_to/go_to_room  →  reporter(LLM, 无工具)  │
            └───────────┬──────────────┬───────────────┬───────────────────┬─────────────────┘
                        │              │               │                   │
                  语义地图真值    Nav2 目标        /agent/* 话题        一句中文回答
                (semantics.json)      ↑               ↑
                                      │               │
                          ┌───────────┴───────────────┴─────────── ROS 侧（系统 python 3.10）┐
                          │  Nav2 (planner/controller/BT)   ←   agent_bridge.py (rclpy)      │
                          │  map_server + static map→odom   ←   /agent/nav_goal|status|pose  │
                          └──────────────────────────────────────────────────────────────────┘
```

三层铁律（踩过才知道）：

| 边界 | 规则 | 违反后的现象 |
|---|---|---|
| venv ↔ 系统 python | **绝不在 venv 里跑 ROS 节点** | `No module named 'rclpy._rclpy_pybind11'` |
| agent ↔ ROS | 只走 rosbridge，不手写 action 协议 | rosbridge 的 ROS2 action 支持不完整 |
| LLM ↔ 执行 | **动作权归代码**，LLM 只做意图与汇报 | 模型"查完就不动"，车一步不走 |

---

## 2. 验收总表（全部为实跑数据）

| 步骤 | 验收内容 | 实测结果 | 判定 |
|---|---|---|---|
| 1a | 链路：rosbridge + `/agent/*` 四话题 | 65 话题，4 个 `/agent` 话题，`/agent/pose` 5 s 收 24~25 帧 | ✅ |
| 1b | 定位一致性 `check_loc.py` | `/odom` 与 `map→base_link` 偏差 **0.00 m / 0.0°** | ✅ |
| 2 | 硬编码目标 3.0 m | `succeeded` 22.4 s，剩余 2.85→0.21 单调降，终点 `[2.801,-0.040,0.085]`，0 恢复 | ✅ |
| 3 | 语义 → 导航 | 冰箱 12.9 m/88.8 s/偏差 0.13 m；沙发 9.5 m/108.6 s/偏差 0.12 m，均 0 恢复 | ✅ |
| 4 | 中文指令闭环（A/B/C 三条路线） | 143.5 s / 25.5 s / 0.0 s，文本与动作全部合规 | ✅ |
| 5 | 复合指令串行 | 客厅→冰箱两段全到达；客厅→(地下室) 中间被正确拒绝 | ✅ |
| 5b | 交互模式（REPL 多轮） | 问句 → 冰箱 118.1 s → 客厅+卫生间 141.5/99.6 s → 拒绝 0.0 s；无 traceback，`/cancel`、`q` 正常 | ✅ |

离线验收（不连 ROS、不调 LLM，秒级）：

| 脚本 | 覆盖 | 结果 |
|---|---|---|
| `dev/check_tools.py` | 工具输出格式、假 id 拦截、不存在地点 | 4/4 |
| `dev/check_intent.py` | 意图判定 8 例 + 执行计划 3 例 | 11/11 |
| `dev/check_refusal.py` | 动作词剥离 + 拒绝文案 | 7/7 |
| `dev/check_planner.py` | 复合拆句 6 例 + 分流 5 例 + 汇报组装 | 15/15 |
| `dev/test_nav_stale.py` | 残留终态时序（假链路） | 2/2 |
| `dev/check_standoffs.py` | 61 物体可站位姿 | 61/61（53 在房间内） |

---

## 3. 架构演进：为什么把动作权从 LLM 手里拿走

### 3.1 初版（已被实测推翻）

```
指令 → LLM 决定调哪个工具 → 工具查坐标 → LLM 决定调 go_to → 执行 → LLM 汇报
```

设想很自然：让模型自主决策。实测它**只做完第一步就收工**：

```
指令：带我去沙发看看
① find_place({'text':'沙发'}) → sofa_1 中心(-8.40,+4.30) 站姿(-6.60,+4.30)
② 模型回答：沙发在客厅正中间，距离这里约一米八。
   ← 1.80 m 是"站姿距物体中心"，被它当成了"离你多远"
③ 核查：本次没有发出移动指令        ← 车一步没动
```

三种补救都无效：

| 补救 | 结果 |
|---|---|
| 提示词加"查询不等于完成任务，只有查询没有 go_to 就是失败" | 无效，行为不变 |
| 加答案范例（Few-shot） | 回答变好，动作仍然不做 |
| `default_options={'tool_choice':'required'}` | **ollama 的 OpenAI 兼容端点直接忽略该字段**：不报错，也不生效 |

### 3.2 定稿：确定性主导，LLM 只占语言边缘

```
指令
 ├─[代码] classify_intent()   祈使句 ACT / 问句 ASK（判错代价不对称，从严判 ACT）
 ├─[代码] plan_action()       房间优先 → 物体解析（房间限定）+ 房间兜底
 ├─[代码] go_to / go_to_room  Nav2 执行 + 看门狗 + 到达判据
 ├─[LLM]  reporter（无工具）   把工具结果说成一句人话 —— 1.9 s
 └─[代码] 目标不存在          直接拒绝并列出现有房间 —— 0.0 s，零 LLM
```

三条边界规则：

1. **LLM 没有"要不要动手"的决定权**。判错的代价极不对称：动错了可以取消并如实说；
   该动没动，用户站在旁边干等。
2. **reporter 不绑任何工具**，物理上不可能边汇报边乱动。
3. **任何路线都不许把空字符串交给用户**（见 §6.3）。

实测收益：同一句祈使句从 3 次 LLM 往返（209 s）降到「行驶 + 8 s 汇报」，重复跑行为一致。

---

## 4. 模型选型（本地 ollama，实测）

先用 `ollama show <model>` 看 `Capabilities`，没有 `tools` 的模型直接出局。

| 模型 | capabilities | 实测表现 | 用途 |
|---|---|---|---|
| qwen3-vl:8b | completion, vision, tools, thinking | 中文准、把「洗手间」原样传给工具 | ✅ 理解/查证 |
| qwen2.5:7b | completion, tools | 工具模式下吐碎片乱码 `pora… sidelineda…`；但**改写一句话很好** | ✅ 仅做 reporter |
| llama3.1:8b | completion, tools | 把「洗手间」抄成「汉测式」 | ❌ |
| gemma3:12b | completion, vision | 无 tools 能力 | ❌ |

按角色分工，并用 `dev/check_report.py` 量出来的延迟（同一句改写任务，热态）：

| 角色 | 模型 | 冷加载 | 热态 |
|---|---|---|---|
| 改写汇报 | qwen2.5:7b | 8.9 s | **1.9 s** |
| 改写汇报 | qwen3-vl:8b | 51.7 s | 31.7 s |

两个静默失效的开关（都别再试）：

```
tool_choice="required"   传了不报错、不生效（ollama 兼容层忽略）
Qwen3 /no_think 后缀     被当普通 token，且更慢：69.1 / 47.2 s vs 31.7 s
```

---

## 5. 速度与时间预算

| 量 | 数值 | 说明 |
|---|---|---|
| 配置速度 | `max_vel_x = 0.26 m/s`（`nav2_params_nav.yaml`） | 建图/探索另用 0.15 m/s 那份 |
| 实测速度 | 0.089 ~ 0.15 m/s | 12.59 m/135.3 s、12.95 m/86.6 s 等换算 |
| 换算原因 | RTF ≈ 0.34 ~ 0.58 | 仿真跑不满实时，参数本身生效 |
| 汇报开销 | 8.2 s（qwen2.5:7b） | 若用 qwen3-vl 则 31.7 s 起 |
| 多目标总量 | 216.7 s 行驶 + 汇报 | 客厅 70.0 s + 冰箱 146.7 s |

---

## 6. 核心问题（现象 → 数据 → 根因 → 修法）

### 6.1 DWB 根本不能倒车 → 车头朝里挤进凹槽就锁死

**现象**：目标被接受，状态 `active`，但车不动，直到看门狗取消。

**四路数据一起记**（`dev/diag_goto.py`：`/plan` + `/cmd_vel` + `/odom` + 局部代价地图）：

```
|vx| 最大 0.260 m/s（满速在发）      |wz| 最大 0.579 rad/s
位移 0.02 m                          ← 满速指令，车纹丝不动
recoveries = 0                       ← 不是规划失败（那会 abort 并计数）
局部代价地图 80x80@0.05：车周围 0.8 m 内 164 个致命格   ← 被围死
全局代价地图同范围：0 个致命格        ← 椅子腿在 0.132 m 高的 2D 图上很稀疏
路径 489 点 / 12.29 m，末点正是目标站姿  ← 全局规划是好的
```

**根因**：`min_vel_x: 0.0` —— DWB 在物理上禁止倒车。车头朝里停进家具缝后只能原地转，
而转向空间也被家具吃掉，于是满速空转。

**修法**（两处，缺一不可）：

```yaml
# config/nav2_params_nav.yaml
min_vel_x: -0.15
max_vel_x_backwards: 0.15
```

```python
# semantic_map.py：站位同时避开"其他家具包围盒"
CLEAR_M = 0.45        # 对齐 Nav2 inflation_radius（原 0.30 会选到家具缝）
MIN_OBJ_GAP = 0.35    # 到其他物体包围盒的最小距离
# 逐级放宽 [0.45,0.35] → [0.40,0.25] → [0.32,0.15] → [0.30,0] 保证 61/61 有解
```

修后冰箱站位从 `(-1.79,+4.84)`（卡死点 0.14 m 内）移到 `(-1.72,+5.20)`，离卡死点 0.84 m。

**验证**：`dev/check_standoffs.py`（61/61）+ 冰箱/沙发实跑 0 恢复。

### 6.2 背靠背目标的看门狗基线污染（只在复合指令下出现）

**现象**：第二个目标 13.1 s 就被判 `stalled`，可距离明明在下降。

```
--- 目标 2/2：到沙发旁边 ---
[  0.0s] active  剩余 0.22 m    ← 上一个目标的残留距离
[  2.0s] active  剩余 11.66 m
[ 12.0s] active  剩余 10.13 m   ← 走了 1.5 m，正常行驶
[ 13.1s] stalled                ← 仍然被砍
```

**根因**：看门狗以第一个采样作为基线。`on_goal` 虽然把 `distance`/`best_distance` 置 `None`，
但上一条目标的 feedback 在新目标注册之后才到，又把基线写成 0.22 m —— 真实距离 11.66 m
永远无法比它改善 0.15 m，12 s 定时器到点即判死。`NavigateToPose` 的 feedback 没有目标 id，
无法从内容区分新旧。

**修法**（`agent_bridge.py`，前两处治本，第三处免疫）：

1. 基线重置挪到 `on_goal_response`（目标被**接受**那一刻），不放 `on_goal`；
2. `on_feedback` 只在 `state == 'active'` 时采信；
3. 看门狗自愈：`distance > best + 1.0` → 判定基线污染，以当前采样重新起算并打 warn。

**验证**：修后第二个目标 `t=0` 显示 `剩余 None`，距离全程单调递减到 0.17 m，0 恢复。
单目标跑一百次都不会触发，只有复合指令才暴露 —— 这也是选复合指令方向带来的额外收获。

### 6.3 模型"想了 53.8 秒却不说话"

**现象**：`带我去地下室`（走 LLM 路线）工具调用 0 次、**回答为空**；而"没乱开车"的核查仍是 ✅
（因为确实没发车）—— 缺陷只表现为一行空白。

**修法**：目标解析不到时**不调 LLM**，由代码直接拒绝（"家里有哪些房间"是我们的真值），
并对所有路线加空字符串兜底：

```
执行模式：C 确定性拒绝（目标不存在，不调 LLM）   总耗时 0.0 s
回答：家里没有叫「地下室」的地方。现有的是：客厅、厨房、书房、卧室1、卧室2、卫生间、储藏间。
```

**验证**：`dev/check_refusal.py` 7/7（含动作词剥离：`带我去地下室` → `地下室`）。

### 6.4 交互模式：每条指令关一次事件循环 → httpx 连接不回收

**现象**：`main.py --interactive` 每敲完一条指令就打印一段 `RuntimeError: Event loop is closed`
（`Task exception was never retrieved`，来自 `httpx2.AsyncClient.aclose()`），结果本身还是对的。

**根因**：REPL 里每条指令调一次 `asyncio.run()`，跑完立即关循环；而 MAF 的 chat client 内部包着
`httpx2.AsyncClient` 且**没有提供 `close()`/`aclose()`**，它的清理任务只能排在关循环之后执行。
功能不受影响，但连接始终不回收，长会话会泄漏。

**修法**：整个交互会话只用一个事件循环（`asyncio.run(repl_async(...))` 一次），
stdin 用 `await asyncio.to_thread(input, ...)` 读。一次性 CLI 调用不受影响（只关一次循环）。

**附带收益**：`try/except KeyboardInterrupt` 包住 REPL，Ctrl+C 退出前先发一次 `/agent/cancel`，
避免"程序已经没了、车还在跑它剩下的目标"。

### 6.5 附：定位跳变伪装成导航 bug（改用 loc:=truth）

> **更正（2026-10-26）**：本例后来确认是**人为设置仿真**造成的，不是 AMCL 或本项目的缺陷。
> 下面记录的是当时的观测与由此得出的做法（`loc:=truth`），做法仍然成立；
> "AMCL 在这张图上会跳"这个因果判断作废。

AMCL 在这张探索图上出现 `map→odom` 平移 1.25 m + 旋转 124.5° 的"修正"，
使 `map→base_link` 偏 3.25 m，而 `/odom` 与真值误差 < 1 mm —— 表现是"规划的目标是错的"，
语义解析和站位计算却完全正确。`recovery_alpha_fast/slow = 0.0` 关掉了随机粒子注入，
跳错后不可恢复。仿真里 `/odom` 即真值，故用 `loc:=truth`：

```
navigation_launch.py + map_server + static_transform_publisher 0 0 0 0 0 0 map odom
```

launch 参数 `loc:=amcl|truth`，真机路径保持 amcl 不变。AMCL 调参留作独立课题。

---

## 7. 复现命令

```bash
# 终端 A：仿真 + Nav2 + RViz（真值定位 + 最终探索图）
cd ~/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash
bash preflight.sh
ros2 launch ./dan_house.launch.py nav:=true rviz:=true loc:=truth \
    map:=config/maps/preserved/dan_house_explore_FINAL_2026-10-21_1800.yaml

# 终端 B：rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 终端 C：桥接（系统 python，不是 venv）
cd ~/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot
source /opt/ros/humble/setup.bash
/usr/bin/python3 agent_bridge.py

# 终端 D：agent 侧
cd ~/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot/mak_demo
.venv/bin/python dev/check_loc.py                                   # 闸门：偏差应为 0.00 m
.venv/bin/python agent_nav.py "带我去冰箱看看"                       # 单目标
.venv/bin/python agent_nav.py "洗手间在哪"                           # 问句
.venv/bin/python agent_nav.py "带我去地下室"                         # 确定性拒绝
.venv/bin/python main.py "先去客厅，然后到冰箱旁边"                   # 复合
.venv/bin/python main.py --interactive                              # 交互
```

离线（不需要仿真，秒级）：

```bash
.venv/bin/python dev/check_tools.py
.venv/bin/python dev/check_intent.py
.venv/bin/python dev/check_refusal.py
.venv/bin/python dev/check_planner.py
.venv/bin/python dev/test_nav_stale.py
.venv/bin/python dev/check_standoffs.py
```

排查"目标被接受但车不动"：`dev/diag_goto.py 沙发`（同时记 plan / cmd_vel / odom / 局部代价地图，
直接给出四选一判定）。模型选型：`dev/probe_llm.py`；汇报延迟：`dev/check_report.py`。

---

## 8. 数据来源对照

| 结论 | 出处 |
|---|---|
| 链路、定位、开车、语义导航 | 终端实跑输出（本文 §0、§2） |
| 站姿 61/61、53 在房间内 | `dev/check_standoffs.py` |
| 满速指令 + 0.02 m 位移 + 164 致命格 | `dev/diag_goto.py` 实跑 `logs/diag_114957.json` |
| 模型 capability | `ollama show <model>`；`dev/probe_llm.py` |
| 汇报延迟 1.9 s / 31.7 s / 冷加载 20 s | `dev/check_report.py` |
| `tool_choice` / `/no_think` 无效 | 同一指令两次实跑对照（§4） |
| 看门狗误杀 13.1 s | 复合指令实跑输出（§6.2） |
| 空回答 53.8 s | `agent_nav.py "带我去地下室"` 实跑（§6.3） |

---

## 9. 未完成 / 下一步

| 项 | 状态 | 需要的改动 |
|---|---|---|
| 交互模式多轮实跑 | ✅ 已跑 | `main.py --interactive` 四条路径全过；Ctrl+C 退出会先发取消 |
| 中途插话改主意（抢占） | 未做 | ① `nav_client.go_to()` 加 `cancel_if=callable` 每 0.2 s 检查；② 后台线程读 stdin 入队；③ 取消后等 `/cmd_vel` 归零再发新目标 |
| 相机 / VLM 视觉确认 | 未开始 | 相机已桥接但零订阅者；`qwen3-vl:8b` 已具备 vision，可做"到了吗 / 门开着吗"的确认 |
| AMCL 调参 | 独立课题 | `recovery_alpha_fast/slow` 从 0 放开，验证跳变能否自愈 |

---

## 10. 3D（64 线）复验：2026-10-24，代码零改动

雷达换成 64 线（`lidar:=3d`）、地图换成 3D 图之后重跑同一套验收 —— **`mak_demo/` 一行没改**，
因为 3D 版最终仍发布 `/scan`（LaserScan）、帧名仍是 `base_scan`、Nav2 参数不变。

启动方式与 §7 完全一致，只把地图和 `lidar` 换掉：

```bash
ros2 launch ./dan_house.launch.py lidar:=3d nav:=true loc:=truth headless:=true \
    map:=config/maps/dan_house_lighthouse3d.yaml
```

| 步骤 | 验收内容 | 3D 图实测 | 判定 |
|---|---|---|---|
| 1a | 链路 `check_link.py` | `/agent/pose` 25 帧、`/agent/nav_status` 11 条，步骤 1 通过 | ✅ |
| 1b | 定位 `check_loc.py` | 偏差 **0.00 m / 0.0°** | ✅ |
| 2 | 硬编码 3.0 m（`dev/test_nav.py`） | succeeded 57.0 s，终点 `[2.843,0.091,0.265]`，剩余 0.197 m，0 恢复 | ✅ |
| 3 | 语义→导航「带我去冰箱看看」 | 39.4 s 到观察站位（距中心 1.79 m）+ VLM 58.5 s；四项核查全绿；总 131.4 s | ✅ |
| 4 | 问句「洗手间在哪」 | 41.7 s，「南边，约 9.2 米」，未发车 | ✅ |
| 4b | 拒绝「带我去地下室」 | 0.0 s 正确拒绝，未发车 | ✅ |
| 5 | 复合「先去客厅，然后到冰箱旁边」 | 客厅 91.6 s + 冰箱 58.0 s，两段均 succeeded，0 恢复 | ✅ |
| 离线 | tools / intent / refusal / planner / nav_stale | 全部通过 | ✅ |
| 离线 | `check_standoffs.py` | 61/61 可站位姿，53 在所属房间内 | ✅ |

结论：换传感器对智能体层**完全透明**。唯一要注意的是启动 rosbridge 时必须用系统
python3.10（`PATH=/usr/bin:$PATH`），否则 `rclpy` 的 C 扩展找不到。
