# mak_demo —— 第二阶段：AI 智能体引导的语义导航

用 **Microsoft Agent Framework（MAF，`agent-framework` 1.22.0）** 把「自然语言指令」变成
「Nav2 能执行的目标点」，并在执行中监控、失败时改选、结束后用自然语言汇报。

---

## 0. 架构判断：单智能体，不是多智能体

**结论：一个 LLM 智能体 + 一条确定性工作流。** 不是多智能体协作。

判断依据（多智能体的四条经典判据，这个任务一条都不满足）：

| 判据 | 这个任务 | 结论 |
|---|---|---|
| 能否并行拆分子任务 | 导航是物理串行动作，同一时刻只有一个目标 | 否 |
| 单个上下文是否装不下 | 8 房间 / 61 物体，检索后只回 3~5 个候选，几 K token | 否 |
| 是否需要不同模型 / 不同权限 | 一个 7B 模型够用 | 否 |
| 是否需要故障隔离的独立技能 | function tools 的边界就够了 | 否 |

真正需要 LLM 的只有两处：**听懂话**和**说人话**（汇报）。中间的语义解析、可达性校验、
Nav2 执行、卡死监控全是确定性代码 —— 交给 LLM 只会更不可靠。

### 架构（实测定稿：确定性主导，只让 LLM 说话）

原计划是「LLM 决定调哪个工具」。实测推翻了这个设计：qwen3-vl:8b 收到「带我去沙发看看」
只调了 `find_place` 就答"沙发在客厅正中间"，**车一步没动**；改提示词、
加 `tool_choice="required"` 都没用。所以最终形态是：

```
用户指令
  │
  ├─[代码] classify_intent()   祈使句 ACT / 问句 ASK（判错的代价不对称，所以从严）
  ├─[代码] plan_action()       语义解析 → 目标物体或房间（61 物体 + 8 房间全覆盖）
  ├─[代码] go_to / go_to_room  Nav2 执行 + 反馈监控 + 卡死看门狗
  ├─[LLM]  reporter（无工具）   把工具结果说成一句人话 —— 1.9 s
  ├─[LLM]  agent（仅 ASK 时）   查证、消歧，20~45 s
  └─[代码] 目标不存在时         直接拒绝并列出现有房间 —— 0.0 s，零 LLM
```

三条硬边界：

1. **LLM 没有"要不要动手"的决定权** —— 祈使句由代码保证一定发车；
   「该动没动」让用户干等，比「动错了」可撤销得多。
2. **reporter 不绑任何工具** —— 物理上不可能边汇报边乱动。
3. **任何路线返回空字符串都兜底** —— 实测「带我去地下室」走 LLM 时思考 53.8 s
   却返回空正文，用户拿到一片空白；而"没乱开车"的核查还是 ✅。
   现在这条直接由代码拒绝，0.0 s 给出准确的房间清单。

实测收益：同一句祈使句从 3 次 LLM 往返（209 s）降到「行驶 + 8 s 汇报」，且重复跑结果一致。

### 什么时候才该拆成多智能体（先写下来，别现在做）

1. **多机器人并行**：两台车各自导航 → agent 天然对应实体
2. **长时任务需要各自记忆**：例如"上午建图 + 下午巡逻"两个长期目标
3. **技能栈差异大**：探索建图 agent（frontier）与导航 agent（Nav2）分别维护不同工具集
4. **需要不同模型**：便宜的解析模型 + 强的推理模型

到那时拆是自然的（MAF 的 `AgentExecutor` 直接把 agent 挂进 workflow 图），现在拆是纯负担。

---

## 1. 环境（全部隔离在这个目录）

```bash
cd mak_demo
.venv/bin/python --version     # Python 3.10.12
.venv/bin/pip list | grep -i agent-framework | head -3
```

- venv：`mak_demo/.venv`（MAF 及其 200+ 依赖全在这里，不污染系统 python）
- MAF：`agent-framework==1.22.0`（含 openai / ollama / anthropic / gemini / azure 客户端）
- LLM：默认走**本地 ollama**，不需要密钥；两个模型分工（实测依据见 STATUS §6）：
  - 理解 / 查证：`qwen3-vl:8b`（`completion+vision+tools+thinking`，中文准、工具参数抄得对）
  - 汇报：`qwen2.5:7b`（同一句改写 1.9 s，qwen3-vl 要 31.7 s，差 15 倍）
  - 端点 `http://127.0.0.1:11434/v1`；想换云端只改 `base_url` + `api_key`
  - 不能用的：`llama3.1:8b`（把「洗手间」抄成「汉测式」）、`gemma3:12b`（无 tools 能力）
- ROS 通信：**rosbridge**（`rosbridge_server` 已装），agent 用 JSON over WebSocket 跟 ROS 说话

### 铁律

| 规则 | 原因 |
|---|---|
| **绝不用这个 venv 跑 ROS 节点** | `rclpy` 是给系统 python3.10 编译的，混进 venv 必炸（`No module named 'rclpy._rclpy_pybind11'`） |
| ROS 侧一律 `source /opt/ros/humble/setup.bash` + `/usr/bin/python3` | 系统 python 才有 rclpy |
| agent 侧一律 `.venv/bin/python` | 只有这里才有 MAF |
| 两边靠 rosbridge 通信，不共享解释器 | 这是唯一稳定的隔离边界 |

---

## 2. 已验证到哪一步（都是实跑，不是"应该能"）

### 2.1 MAF 全链路：LLM + function tool

```bash
.venv/bin/python dev/smoke_agent.py
```

```
--- LLM 回答 ---
地点 + 坐标：厨房 -> (0.8, 5.2)
```

坐标 0.8/5.2 只存在于工具里，说明 **LLM 真的调用了 Python 工具并采信了返回值**。

### 2.2 语义地图：物体 → 可站位姿

```bash
.venv/bin/python dev/check_standoffs.py
```

```
可站位姿 61/61   其中在所属房间内 53/61
```

61 个物体**每一个**都能算出一个"机器人站得住、不撞墙、朝向物体"的位姿。
在房间外的 8 个都是贴墙小物件（垃圾桶、边桌、伞架），站到门口/走廊里是正确解。

算法规格（两次实测教训后的版本）：绕物体采样 16 个方向，半径 = 物体半径 + 0.55 m；
要求站姿周围 **0.45 m** 无墙（对齐 Nav2 的 `inflation_radius`，原来写 0.30 m 会选到
"地图上可走、实际进去出不来"的家具缝）+ 到**其他物体包围盒** ≥ 0.35 m（椅子腿在 0.132 m 高的
2D 雷达图上很稀疏，只查地图会撞）；逐级放宽 `[0.45,0.35]→[0.40,0.25]→[0.32,0.15]→[0.30,0]`
保证贴墙小物体也有解；优先朝向房间中心；yaw 指向物体。纯几何、可复现。

### 2.3 中文口语 → 房间/物体（**不用 LLM**）

```
带我去厨房          -> 房间=厨房
到沙发旁边          -> 物体=sofa_1(沙发)        正名命中
去卧室2的床边       -> 房间=卧室2 + 物体=bed_2  房间加成
洗手间              -> 房间=卫生间              俗称映射
书房的书架          -> 房间=书房 + 物体=bookshelf_2
```

这说明**大部分指令根本不需要 LLM**；LLM 只在真正歧义（"瓶子在哪"）或需要解释时上场 ——
这也正是"单智能体 + 确定性工作流"划算的原因。

---

## 3. 目录

```
mak_demo/
├── .venv/                   # MAF 及其依赖（不进版本库）
├── requirements.txt         # pip freeze 结果（216 行）
├── rosbridge_link.py        # 极简 rosbridge 客户端（publish/subscribe/service）
├── nav_client.py            # go_to / cancel / 残留终态判定（ts + goal 双条件）
├── semantic_map.py          # 语义地图：resolve_room / resolve_object / standoff_pose / obj_clearance
├── tools.py                 # 5 个 function tools（坐标只能来自工具返回值）
├── agent_nav.py             # 主入口：意图判定 → 发车 → 汇报（CLI）
├── data/lexicon.json        # 33 个物体类别的中文正名 + 口语别名
├── dev/                     # 开发期验证脚本（不参与运行时）
│   ├── smoke_agent.py       # MAF + ollama + tool 冒烟
│   ├── check_standoffs.py   # 61 物体位姿验收 + 口语解析自测
│   ├── check_tools.py       # 工具输出/假 id 拦截（4/4）
│   ├── check_intent.py      # 意图判定 + 执行计划（11/11）
│   ├── check_report.py      # 两个模型的汇报延迟对比
│   ├── check_link.py        # 闸门 1a：链路
│   ├── check_loc.py         # 闸门 1b：定位一致性
│   ├── test_nav.py          # 闸门 2a：硬编码目标
│   ├── test_nav_stale.py    # 离线时序单测（残留终态）
│   ├── test_semantic_nav.py # 闸门 2b：语义 → 导航
│   ├── probe_llm.py         # ollama 模型能力 + MAF API 签名
│   └── diag_goto.py         # 带仪表复现：plan/cmd_vel/odom/局部代价地图
└── README.md
```

上游数据（都在上一级 `robot/` 目录，只读）：

| 文件 | 用途 |
|---|---|
| `config/semantics.json` | 8 房间 + 61 物体的真值坐标（build_world.py 生成） |
| `config/maps/dan_house_explore.pgm` | 站姿可站性校验用的地图 |
| `config/nav2_params_nav.yaml` | 导航速度参数（0.26 m/s） |

---

## 4. 阶段 2A 进度

> 完整实测报告（架构演进 / 模型选型 / 三个核心 bug 的机理与复现命令）：
> `docs/phase2A_results.md`（323 行）

| 步骤 | 内容 | 状态 |
|---|---|---|
| 1a | 链路：rosbridge + `/agent/*` 四话题 | ✅ |
| 1b | 定位一致性：`check_loc.py` 偏差 0.00 m / 0.0° | ✅（`loc:=truth`） |
| 2 | 开车：硬编码目标 3 m → succeeded 22.4 s | ✅ |
| 3 | 语义 → 导航：冰箱 12.9 m / 偏差 0.13 m，沙发 9.5 m / 偏差 0.12 m，均 0 恢复 | ✅ |
| 4 | 中文指令闭环：`agent_nav.py`（确定性主导 + LLM 汇报） | ✅ |
| 5 | 复合指令：`planner.py` + `main.py`（串行多目标 + `--interactive`） | ✅ 实跑通过 |
| 6 | **① 相机 / VLM 视觉确认**：`vision.py` + `look_at` + 模式 D | ✅ 实跑通过（见下） |

### 4.1 ① 相机 / VLM（2026-10-22 夜通过）

完整记录：`docs/phase2B_vision.md`。

结论一句话：**打通视觉确认的关键不是换更大的模型，是把相机从离地 0.117 m 抬到 0.60 m**。

```
低视角（0.117 m）在 0.55~1.41 m 处只框到冰箱面板下部 0.57 m 高的一带
  → 门缝(1.15 m)与把手(0.75/1.45 m)全在画外 → VLM 两帧都答"房门"
抬到 0.60 m，同一模型同一句话
  → "能判断它是一台冰箱：立式长方体 + 右侧垂直长条把手 + 顶部水平横杠" ✅
```

- 观察站位 `OBSERVE_GAP_M=1.40`（距中心约 1.8 m）用于"看看 X"；贴到旁边 0.55 m 仍用于"到 X 旁边"。
- `look_at(object_id)` 工具：开车到观察站位 → 抓帧 → 封闭四选一问 VLM → 与语义地图正名比对。
- 模式 D：`看看 / 看一眼 / 还在吗 / 确认` + 目标可解析 → 走 `look_at`。

```bash
.venv/bin/python main.py "去客厅"        # 11.73 → 0.178 m，79.0 s，0 恢复
.venv/bin/python main.py "冰箱还在吗"    # 11.79 → 0.23 m，75.2 s
# 执行模式：D 视觉确认 → VLM 10.1 s「白色带门把手分层结构」→ 与语义地图一致
# 核查①②③④ 全绿
```

相机链路：原生 7.1 Hz、320×240 rgb8、raw base64 300 KB/帧（2.36 MB/s），
**转 PNG 后 58 KB**（小 5 倍，VLM 输入一律用 PNG）；VLM 单次 10~26 s，够做"到点确认"，不进控制环。

语言层基准（`dev/bench_language.py`，10 条用例）：工具只给坐标 **7/10**（两问一答，模板化）；
代码算好方位距离注入后 **10/10**（同一 8B 模型）。
换 cloud 买到的是延迟和多轮理解，不是正确性 —— 端点是配置项：`--base-url/--api-key`。

步骤 4 的实绩：

```bash
.venv/bin/python agent_nav.py "带我去沙发看看"
# 意图判定 ACT → 执行计划 沙发(sofa_1) → 模式 A → 12.59→0.17 m, 135.3 s, 0 恢复
# 汇报 qwen2.5:7b 8.2 s：已经到沙发旁边了，用了两分多钟。

.venv/bin/python agent_nav.py "洗手间在哪"
# 模式 B → 调 find_place → 不发车：洗手间在走廊南边，从这边走过去大概十米。

.venv/bin/python agent_nav.py "带我去地下室"
# 模式 B → 不发车 → 家里没有叫地下室的地方，现有的是客厅、厨房、书房、两间卧室、卫生间和储藏间。
```

下一步（步骤 5）要做的事：把三条路线收进 `main.py`，并支持
「去厨房，然后到冰箱旁边」这类多目标序列 —— 关键设计点已经明确：
**每个子目标仍由代码执行，LLM 只负责把整段话拆成目标序列并串起汇报**。
