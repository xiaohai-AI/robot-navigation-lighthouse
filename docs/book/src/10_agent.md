# The agent layer: language in, motion out

## 10.1 Four processes, two Python environments

The agent stack is split across four processes on purpose, and the split is the most common
source of confusion for newcomers:

```
terminal 1   ros2 launch …                  system python 3.10 + rclpy     (simulation, SLAM, Nav2)
terminal 2   rosbridge_websocket            system python 3.10 + rclpy     (port 9090)
terminal 3   agent_bridge.py                system python 3.10 + rclpy     (/agent/* ⇄ Nav2)
terminal 4   mak_demo / main.py             project venv (Python 3.10)     (LLM, tools, semantic map)
```

The iron rule: **every process that imports `rclpy` must run on the system interpreter.**
`rclpy` in ROS 2 Humble is compiled for the system Python; running it inside the project venv
produces `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`, which looks like a
broken ROS installation and is not. The agent side runs in the venv because it needs
`agent-framework` and the OpenAI SDK, which are not system packages.

The two halves meet only over a websocket. That boundary is also a debugging boundary: if
`/agent/pose` updates but the robot does not move, the problem is downstream (bridge or Nav2);
if nothing updates, the problem is upstream (rosbridge or the venv side).

## 10.2 The `/agent/*` contract

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/agent/nav_goal` | `geometry_msgs/PoseStamped` | agent → bridge | target pose; sending a new one **supersedes** the old |
| `/agent/nav_status` | `std_msgs/String` (JSON, latched) | bridge → agent | state, progress, remaining distance, recovery count |
| `/agent/pose` | `geometry_msgs/PoseStamped` | bridge → agent | current pose at 5 Hz |
| `/agent/cancel` | `std_msgs/Empty` | agent → bridge | cancel the current goal |

`/agent/nav_status` is **latched** (transient-local). This is not cosmetic: a status published
by a daemon thread before any subscriber connects is otherwise lost forever, and the agent then
waits for a state change that already happened. The same lesson applies to any status topic a
tool subscribes to late.

`agent_bridge.py` (287 lines) owns one more thing: **goal sequencing**. Every goal sent gets a
monotonically increasing `seq`, and all three callbacks (`on_goal_response`, `on_feedback`,
`on_result`) carry it. Results whose `seq` is stale are logged and dropped:

```
已有目标在飞，先取消旧的（旧 #1 → 新 #2）
忽略已被取代的目标 #1 的结果（当前 #2）
```

Without this, Nav2's terminal result for the *cancelled* goal arrives after the *new* goal has
started, and marks it `failed` — and worse, clears the handle that the cancel path and the
watchdog need (Chapter 12 §12.4, bug 1).

## 10.3 Models: measured, not evangelised

| Model | Capabilities | Verdict |
|---|---|---|
| `qwen3-vl:8b` | completion + vision + tools + thinking | **default agent** — accurate Chinese, copies arguments correctly |
| `qwen2.5:7b` | completion + tools | **reporter only** — as a tool agent it emits fragmentary garbage (`"pora房间信息…"`) |
| `llama3.1:8b` | completion + tools | rejected: transcribed 洗手间 as 汉测式 |
| `gemma3:12b` | completion + vision | rejected: no tool calling at all |

The reporter split is a latency decision with a measurement behind it: rewriting a result into
one sentence needs no reasoning, and `qwen2.5:7b` does it in **1.9 s** hot versus **31.7 s** for
`qwen3-vl:8b` — a 15× difference that dominates every user-visible interaction. So the thinking
model plans and calls tools; the fast model narrates.

Both endpoints are configurable (`--base-url`, `--api-key`, `ROBOT_LLM_*` environment
variables), so moving to a cloud model is a flag change, not a refactor.

## 10.4 Why the model is not allowed to decide to move

The routing tree, implemented in code (`planner.py`, `preempt.py`, `agent_nav.py`):

```
utterance
 ├─ classify: imperative (ACT) vs question (ASK)          — code, regex + verb table
 ├─ ACT, resolvable target
 │    ├─ "看看/确认" + object → mode D: look_at + VLM
 │    └─ otherwise            → code sends the goal directly
 ├─ ACT, unresolvable target  → code refuses, lists real places (no LLM call, no motion)
 ├─ cancel words              → code sends /agent/cancel (route F), no LLM call
 └─ ASK                       → LLM with tools: find_place / list_rooms / where_am_i
```

The reason is a measured rebellion, not a philosophy: given 「带我去沙发看看」, `qwen3-vl:8b`
called `find_place`, answered "the sofa is in the middle of the living room", and stopped — the
robot never moved. Adding prompt instructions and `tool_choice="required"` did not fix it,
because the ollama compatibility layer ignores that field. For a robot standing in front of a
user, "decided not to act" is indistinguishable from "broken".

So the contract is inverted: **code guarantees motion; the model handles what only it can do
well** — resolving colloquial ambiguity, choosing among scored candidates, and phrasing the
result as a sentence a person would say.

## 10.5 The tool surface

Seven tools are exposed to the LLM (`mak_demo/tools.py`, 292 lines):

| Tool | Arguments | Effect |
|---|---|---|
| `list_rooms()` | — | names + one-line description of every room |
| `find_place(text, room='')` | Chinese phrase | scored candidates with reasons (no motion) |
| `where_am_i()` | — | current pose and the room it falls in |
| `go_to(object_id)` | resolved id | drives to the object's standoff pose |
| `go_to_room(room)` | room name | drives to room centre |
| `look_at(object_id)` | resolved id | drives to the observation pose, then VLM confirms |
| `check_path()` | — | laser + vision passability verdict (no motion) |

Every tool returns a machine-readable status **and** a sentence, and every call is logged into a
trace that is printed after the run. The trace is what makes "the robot said something odd"
debuggable: you can see the exact arguments the model copied from the semantic map, which is
where transcription errors (汉测式) surface.

## 10.6 Three self-checks after every run

`agent_nav.py` ends with an assertions block that has caught more defects than any unit test:

```
核查①  did the robot actually send motion commands, and did they succeed?
核查②  is the answer text clean (no English fragments, no leaked tool names)?
核查③  if a target was named, did the robot actually arrive?
```

Check ③ is the one that prevented the project from shipping a lying robot: after a preemption,
an older build printed "arrived" for a goal that was cancelled. The check turns that into a
loud `❌ 有明确目标却没有成功到达 —— 任务未完成`.

> **Takeaways.** Four processes, one websocket, and a hard rule about which interpreter may
> import `rclpy`. The status topic is latched because late subscribers matter. Goals carry
> sequence numbers. The model plans and narrates; code moves the robot and refuses impossible
> requests. Self-checks assert *arrival*, not *intent*.
