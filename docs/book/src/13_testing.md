# How we know it works

## 13.1 Test layers

| Layer | Cost | What it can catch | What it cannot |
|---|---|---|---|
| Offline suite (11 checks) | seconds, no ROS | parsing, planning tables, geometry, source-level guardrails | anything about ROS lifecycle or timing |
| Unit checks for the scan guard (6 cases) | milliseconds | judging logic for `/scan` publisher counts | a real duplicate publisher |
| Runtime gates G1–G8 | ~6 minutes with simulation | TF, bridging, Nav2, agent, vision, refusal, compound commands | long-run degradation |
| Campaigns (Chapter 9) | 20–45 minutes per run | statistical effects on coverage and stability | anything semantic |
| Scripted live sessions | minutes | concurrency, preemption, terminal interaction | visual quality |

The order matters: cheap layers run first every time, and the expensive layer is reserved for
questions the cheap ones cannot answer.

## 13.2 The offline suite

```
$ cd mak_demo && .venv/bin/python dev/check_*.py        # 11/11
```

| Check | Asserts |
|---|---|
| `check_checks` | the post-run assertion block itself behaves correctly |
| `check_intent` | ACT/ASK classification over real utterances |
| `check_tools` | tool argument handling and refusal paths |
| `check_refusal` | unresolvable target refuses without motion |
| `check_planner` | multi-clause plans and their ordering |
| `check_passability` (19 cases) | laser sector tiers, self-echo exclusion |
| `check_vlm_guard` (3 cases) | VLM answer validation, closed-option parsing |
| `check_vision_options` | the 28-class option list is complete and stable |
| `check_look_route` (9 cases) | observation-pose selection and failure reporting |
| `check_standoffs` | standoff geometry against the real semantic map |
| `check_preempt` (40 assertions, 5 groups) | judgment table, segmentation, fake-link cancellation, `to_thread` heartbeat, **source-level guardrails** |

Two things are unusual and worth copying:

* **`check_preempt` reads the source.** It greps `agent_bridge.py` for `self.goal_seq += 1`
  before the handle release, for the three callbacks accepting a sequence, and for `navc.VERBOSE`
  being set from `--progress`. A behavioural test cannot see "the ordering is wrong"; a source
  assertion can, cheaply.
* **The suite runs without ROS, ollama or a simulator.** That is why it can be run on every
  edit, which is why it finds things.

## 13.3 The eight runtime gates

Each gate is a script; each prints a verdict and exits non-zero on failure, so the whole ladder
can be run unattended.

```bash
# terminal 1 — the stack (isolated domain)
ROS_DOMAIN_ID=161 IGN_PARTITION=ab161 ros2 launch ./dan_house.launch.py \
  lidar:=3d nav:=true loc:=truth headless:=true \
  map:=config/maps/dan_house_lighthouse3d.yaml

# terminals 2–3 — bridge path (system python, both)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
/usr/bin/python3 agent_bridge.py

# terminal 4 — gates
cd mak_demo
.venv/bin/python dev/check_link.py            # G1
.venv/bin/python dev/check_loc.py             # G1b
.venv/bin/python dev/test_nav.py              # G2
.venv/bin/python dev/test_semantic_nav.py 冰箱 # G3
.venv/bin/python agent_nav.py "带我去冰箱看看"    # G4
.venv/bin/python agent_nav.py "前面能过去吗"     # G5
.venv/bin/python agent_nav.py "洗手间在哪"       # G6
.venv/bin/python agent_nav.py "带我去地下室"     # G7
.venv/bin/python main.py "先去客厅，然后到冰箱旁边" # G8
```

Baseline versus last full run (2026-10-26, domain 161):

| Gate | Baseline | Last run | Verdict |
|---|---|---|---|
| G1 link | 25 pose frames, status idle | 24 frames, status idle | pass |
| G1b localisation | 0.00 m / 0.0° | 0.00 m / 0.0° | pass |
| G2 drive (3.0, 0.0) | 57.0 s, 0.197 m residual | 13.3 s, 0.19 m residual | pass (faster: different start pose) |
| G3 semantic 冰箱 | — | 63.0 s, 0.15 m off plan | pass |
| G4 mode D | 1.79 m, VLM 58.5 s | 1.79 m, VLM 13.1 s warm | pass |
| G5 mode E | — | laser 1.12 m, vision "blocked", agree | pass |
| G6 mode B | 41.7 s, "south ~9.2 m" | 41.1 s, 9.2 m | pass |
| G7 refusal | 0.0 s | 0.0 s | pass |
| G8 compound | 91.6 + 58.0 s | 55.6 + 58.2 s | pass |

The G8 result is the one that mattered most in the last regression: the fix for bug 4 of Chapter
12 touched exactly the segment-filtering path that compound commands use, so a passing G8 was
the evidence that the fix did not damage the older behaviour.

## 13.4 Isolation, or why the numbers can be trusted

Every measurement in this book was produced with:

```
ROS_DOMAIN_ID = fresh value per run (87–104, 120–129, 130, 140, 150/151, 160, 161 …)
IGN_PARTITION = matching value (lhL104, ab120 … ab161)
map prefix    = tag_arm_rep (e.g. ..._ab_1025_2047_0.40_1)
log name      = tag_arm_rep_d{domain}.log
```

Consequences: no run can read or overwrite another's map; a failed run's log is never overwritten
by its retry (the domain is in the filename); and after the fact, every number in the CSV can be
traced to one log and one map prefix.

## 13.5 The scan-guard scenarios (reproducible in five minutes)

```
3D normal            ✅  /scan publisher 1: /pointcloud_to_laserscan
3D negative          ❌  /scan has 2 publishers: /_ros2cli_12764, /pointcloud_to_laserscan
    (inject the second publisher:)
    ros2 topic pub /scan sensor_msgs/msg/LaserScan "{header: {frame_id: base_scan},
      angle_min: -3.14159, angle_max: 3.14159, angle_increment: 0.01745,
      range_min: 0.12, range_max: 12.0, ranges: [1.0]}" -r 5
rogue publisher gone ✅  recovered without relaunch
2D normal            ✅  /scan publisher 1: /gz_bridge
```

## 13.6 Six rules of evidence used throughout

1. **Verify the effect, not the call.** "The publisher exists" is not "the subscriber received
   it"; status topics are latched for this reason, and gates assert arrival, not intent.
2. **Numbers carry a domain and a map prefix.** An unattributable number is a rumour.
3. **A failed run is recorded, not deleted.** It is the only evidence of the failure mode.
4. **A single run supports no claim.** Chapter 9 §9.6.
5. **Distinguish "loading" from "broken".** The 210 s boot self-check, and preflight's
   45-second heuristic.
6. **A cancellation must never look like an arrival.** This one sentence summarises half of
   Chapter 12.

> **Takeaways.** Five test layers, cheapest first. The offline suite is unusual in that it reads
> the source for ordering invariants. Gates are scripts that exit non-zero. Every number is
> traceable to a domain, a map prefix and a log.
