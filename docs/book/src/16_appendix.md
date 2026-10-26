# Appendix A — Topics and nodes

## A.1 Topics

| Topic | Type | Producer | Consumers |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | gz bridge | everything (`use_sim_time:=true`) |
| `/odom` | `nav_msgs/Odometry` | gz bridge | `odom_tf_node`, Nav2 |
| `/odom_wheels` | `nav_msgs/Odometry` | gz bridge | EKF experiments |
| `/imu` | `sensor_msgs/Imu` | gz bridge | EKF experiments |
| `/joint_states` | `sensor_msgs/JointState` | gz bridge | `robot_state_publisher` |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2, tour, teleop | gz bridge |
| `/scan` | `sensor_msgs/LaserScan` | bridge (2D) / `pointcloud_to_laserscan` (3D) | `slam_toolbox`, costmaps, exploration, passability, scan guard |
| `/scan/points` | `sensor_msgs/PointCloud2` | gz bridge (3D only) | *(nothing, today — Chapter 14 §14.4)* |
| `/camera/image_raw` | `sensor_msgs/Image` | gz bridge | `mak_demo/vision.py`, camera dev checks |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | gz bridge | camera dev checks |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` or `map_server` | Nav2, exploration, standoff checks |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `slam_toolbox`, `odom_tf_node`/EKF, `robot_state_publisher`, static publishers | everything |
| `/agent/nav_goal` | `geometry_msgs/PoseStamped` | agent (websocket → bridge) | `agent_bridge` |
| `/agent/nav_status` | `std_msgs/String` (JSON, latched) | `agent_bridge` | agent |
| `/agent/pose` | `geometry_msgs/PoseStamped` | `agent_bridge` (5 Hz) | agent |
| `/agent/cancel` | `std_msgs/Empty` | agent | `agent_bridge` |

## A.2 Nodes

| Node | Package / file | Role |
|---|---|---|
| `gz_bridge` | `ros_gz_bridge/parameter_bridge` | the ten-topic contract of Chapter 5 §5.3 |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan` | 3D → 2D projection (3D mode only) |
| `robot_state_publisher` | `robot_state_publisher` | URDF → static and dynamic TF |
| `odom_tf_node` | project | integrates ground-truth pose into `odom→base_footprint` |
| `ekf_node` | `robot_localization` | alternative `odom→base_footprint` (`ekf:=true`) |
| `slam_toolbox` | `slam_toolbox/async_slam_toolbox_node` | mapping and `map→odom` |
| `map_server`, `lifecycle_manager` | `nav2_map_server`, `nav2_lifecycle_manager` | map loading when `loc:=truth` |
| `controller_server`, `planner_server`, `bt_navigator`, `behavior_server` | `nav2_*` | navigation |
| `tour_node` | project (521 lines, frozen) | 32-waypoint tour publisher |
| `frontier_explore` | project (943 lines) | exploration |
| `scan_pub_guard` | `tools/scan_pub_guard.py` | `/scan` single-publisher assertion |
| `rosbridge_websocket` | `rosbridge_server` | DDS ⇄ websocket (:9090) |
| `agent_bridge` | `agent_bridge.py` (287 lines) | `/agent/*` ⇄ Nav2 actions, goal sequencing |
| agent side | `mak_demo/main.py`, `agent_nav.py` | REPL, routing, LLM tools, preemption |

# Appendix B — Parameter reference

## B.1 Localisation and TF

| Parameter | Value | File |
|---|---|---|
| `loc` | `amcl` \| `truth` | launch |
| `ekf` | `false` \| `true` | launch |
| `transform_publish_period` | 0.02 s | `slam_toolbox.yaml` |

## B.2 SLAM

| Parameter | Value |
|---|---|
| `mode` | `mapping` |
| `scan_topic` | `/scan` |
| `resolution` | 0.05 m |
| `map_update_interval` | 2.0 s |
| `min_laser_range` / `max_laser_range` | 0.12 / 12.0 m |
| `minimum_travel_distance` | 0.3 m |
| `minimum_travel_heading` | 0.3 rad |
| `use_scan_matching` | true |

## B.3 Scan projection (3D mode)

| Parameter | Value |
|---|---|
| `target_frame` | `base_scan` |
| `min_height` / `max_height` | −0.75 / +0.05 m |
| `angle_min` / `angle_max` | ±π |
| `angle_increment` | 0.0174533 rad (360 rays) |
| `range_min` / `range_max` | 0.12 / 12.0 m |
| `use_inf` | true |

## B.4 Navigation

| Parameter | `nav2_params.yaml` (mapping) | `nav2_params_nav.yaml` (acceptance) |
|---|---|---|
| `max_vel_x` | 0.15 | 0.26 |
| `min_vel_x` | 0 | −0.15 |
| `robot_radius` | 0.23 | 0.23 |
| `inflation_radius` | 0.45 | 0.45 |
| `cost_scaling_factor` | 3.0 | 3.0 |
| `progress_checker` | 0.25 m / 6 s | 0.25 m / 6 s |
| `xy_goal_tolerance` | 0.20 m | 0.20 m |
| planner | NavFn | NavFn |
| controller | DWB (`FollowPath`) | DWB |
| `observation_sources` | `scan` (LaserScan) | `scan` (LaserScan) |

## B.5 Exploration

| Parameter | Default | Meaning |
|---|---|---|
| `explore_max_goals` | 120 | hard cap |
| `explore_max_minutes` | 60.0 | wall/sim cap |
| `explore_min_cluster` | 12 | frontier cells to be a candidate |
| `explore_revisit_gap` | 0.4 | penalty when re-targeting nearby |
| `explore_goal_clearance` | 0.40 | must be ≥ `inflation_radius` in practice |
| `explore_min_goal_dist` | 0.60 | skip too-close frontiers |
| `explore_stall_secs` | 12.0 | watchdog (18 s measured worse) |
| `min_gain` | 40 cells | fake-frontier blacklist threshold |
| plan pre-check | `ComputePathToPose`, 1.5 s | reject unreachable goals early |
| soft-blacklist TTL | 30 s (no path) / 60 s (detour) | |

## B.6 Agent layer

| Item | Value |
|---|---|
| LLM endpoint | `http://127.0.0.1:11434/v1` (ollama, OpenAI-compatible) |
| Agent model | `qwen3-vl:8b` |
| Reporter model | `qwen2.5:7b` (1.9 s vs 31.7 s hot) |
| Framework | `agent-framework` 1.22.0 |
| rosbridge | `ws://127.0.0.1:9090` |
| Standoff gap / clearance | 0.55 m / 0.45 m |
| Observation distance | object radius + 1.40 m |
| Camera height | 0.70 m |
| Passability `MIN_R` | 0.12 m |

# Appendix C — Glossary (Chinese ↔ English)

| 中文 | English | Where |
|---|---|---|
| 巡游 / 航点 | tour / waypoint | `tour_node.py`, `route.json` |
| 建图 | mapping | Chapter 7 |
| 探索 / 边界 | frontier exploration | `frontier_explore.py` |
| 卡死 | stall | watchdog, §8.4 |
| 假边界 | fake frontier | §9.2 |
| 膨胀半径 / 膨胀税区 | inflation radius / inflation tax region | §8.2 |
| 站位 | standoff pose | §11.3 |
| 观察站位 | observation pose | §11.4 |
| 语义地图 | semantic map | Chapter 11 |
| 词表 / 别名 | lexicon / alias | §11.2 |
| 可通行性 | passability | §11.6 |
| 抢占 / 中途改主意 | preemption | Chapter 12 |
| 叫停词 | cancel keyword | §12.2 |
| 判定表 | decision table | §12.2 |
| 序号（目标序号） | goal sequence number | §10.2 |
| 验收闸门 | acceptance gate | Chapter 13 |
| 离线套件 | offline suite | §13.2 |
| 真值定位 | truth-pose localisation | §7.4 |
| 冻结 / 交付快照 | freeze / delivery snapshot | §14.2 |
| 货位 / 槽位地址 | slot address | §15.3 |

# Appendix D — File index

Approximately 10,000 lines of Python, measured at delivery time.

| File | Lines | Purpose |
|---|---|---|
| `frontier_explore.py` | 943 | exploration node (Chapter 9) |
| `build_world.py` | 898 | world + semantics + route generator (Chapter 3) |
| `tour_node.py` | 521 | frozen tour publisher (Chapter 9) |
| `dan_house.launch.py` | 495 | runtime wiring (Chapter 5) |
| `tools/explore_ab.py` | 454 | experiment driver (§9.5) |
| `mak_demo/agent_nav.py` | 422 | intent routing, modes A/D/E/F/ASK (§10.4) |
| `mak_demo/main.py` | 323 | REPL, preemption, `--progress` (Chapter 12) |
| `mak_demo/tools.py` | 292 | the seven LLM tools (§10.5) |
| `agent_bridge.py` | 287 | `/agent/*` ⇄ Nav2, goal sequencing (§10.2) |
| `mak_demo/dev/check_preempt.py` | 258 | 40 assertions incl. source guardrails (§13.2) |
| `mak_demo/semantic_map.py` | 256 | resolve / standoff (Chapter 11) |
| `mak_demo/vision.py` | 230 | camera capture, VLM prompt, closed options (§11.5) |
| `tools/explore_progress.py` | 194 | read-only progress viewer |
| `tools/map_compare.py` | 181 | map comparison / scoring |
| `mak_demo/passability.py` | 155 | laser sector tiers (§11.6) |
| `mak_demo/nav_client.py` | 143 | Nav2 client, in-flight registry, `stop_all` (Chapter 12) |
| `tools/scan_pub_guard.py` | 140 | `/scan` assertion (§6.4) |
| `mak_demo/rosbridge_link.py` | 139 | websocket transport |
| `tools/map_quality.py` | 123 | the single PGM decoder (§7.2) |
| `gen_textures.py` | 120 | procedural floor/wall textures |
| `mak_demo/preempt.py` | 101 | judgment table, segmentation (§12.2) |
| `odom_tf_node.py` | 78 | `odom→base_footprint` from truth pose (§4.3) |
| `mak_demo/planner.py` | 57 | multi-clause splitting |
| `mak_demo/bearing.py` | 43 | relative direction phrasing |

## D.2 Configuration and data

| Path | Size | Notes |
|---|---|---|
| `config/semantics.json` | 11 KB | 8 rooms, 61 objects, 33 classes (generated) |
| `config/route.json` | 2.1 KB | 32 waypoints (generated) |
| `mak_demo/data/lexicon.json` | 3.3 KB | 35 alias entries |
| `config/slam_toolbox.yaml` | — | Chapter 7 |
| `config/nav2_params*.yaml` | — | four roles (§8.1) |
| `config/nav_to_pose_explore.xml` | 48 lines | exploration behaviour tree |
| `config/maps/` | 390 files | live + experimental + preserved maps |
| `experiments/ab_explore.csv` | 17 columns | every campaign run, valid or not |
| `docs/book/` | — | this book's sources and figures |

# Appendix E — Experiment log

| Tag | Question | Arms | Runs | Verdict |
|---|---|---|---|---|
| — | goal pre-check (§9.2) | off / on | — | adopted: converts stalls into immediate blacklists |
| Recovery a | BackUp-first BT | stock / backup-first | — | rejected: 37.6 vs 45.0 m², 44 % slower |
| — | stall watchdog | 12 s / 18 s | — | kept 12 s: 18 s gave no benefit (4 vs 2 stalls, worse coverage) |
| — | planner c | NavFn / SmacPlanner2D | 40 goals | NavFn adopted: 35/40 vs 28/40, 18.0 vs 36.2 m² |
| `1025_1647` | clearance | 0.40 / 0.50 / 0.55 | 3 × 3, 40 goals | no effect above noise → clearance is a floor, not a knob |
| `1025_2047` | budget | 0.40 | 2 × 80 goals | knee at ≈48 goals; 80 goals buys 2 m² in 41 min |
| `gc55` | clearance 0.55, single run | 0.55 | 1 | superseded by `1025_1647` (single-run artefact) |
| `smoke`, `smoke2` | driver validation | 0.40 | 2 | driver works; one `no_goals` run invalidated a policy |

The full per-run rows (domain, map prefix, log path, eight metrics) are in
`experiments/ab_explore.csv`; every row can be traced to a log file and a map prefix
(Chapter 13 §13.4).
