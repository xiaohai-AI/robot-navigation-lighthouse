# The runtime: one launch file, fifteen switches

## 5.1 Why one launch file

`dan_house.launch.py` (495 lines) starts everything: the simulator, the bridge, the scan
projection, the TF nodes, SLAM, Nav2, exploration, the tour and RViz. A single entry point is a
deliberate trade: the file is long, but there is exactly one place where the *wiring order*
lives, and the wiring order is the part that breaks (Chapter 9 records two separate incidents
caused by a lifecycle bring-up race).

Everything else is a switch. Nothing is deleted when a subsystem is superseded; it is gated.

| Argument | Default | Meaning |
|---|---|---|
| `lidar` | `2d` | `2d` → single-line model + `/scan` bridged directly; `3d` → 64-line model + projection |
| `slam` | `false` | start `slam_toolbox` (mapping mode) |
| `nav` | `false` | start Nav2 (`nav2_bringup`, or the truth-pose variant) |
| `loc` | `amcl` | `amcl` → AMCL owns `map→odom`; `truth` → static identity, `/odom` is trusted |
| `ekf` | `false` | `odom→base_footprint` from `robot_localization` instead of `odom_tf_node.py` |
| `tour` | `false` | run the frozen 32-waypoint tour (`tour_speed`, `tour_spin_rate`) |
| `explore` | `false` | run `frontier_explore.py` (see its own six parameters below) |
| `rviz`, `headless`, `nvidia` | `false` | display, no-GUI server mode, GPU offload |
| `map` | (2D map) | map YAML to load when navigating |
| `use_sim_time` | `true` | almost never changed |

Exploration owns six more parameters, all of which are *measurement devices* rather than
preferences: `explore_max_goals` (120), `explore_max_minutes` (60), `explore_min_cluster` (12),
`explore_revisit_gap` (0.4), `explore_goal_clearance` (0.40), `explore_min_goal_dist` (0.60),
`explore_stall_secs` (12.0).

## 5.2 The order matters: a timetable, not a list

Subsystems are not started simultaneously. Each delay exists because of an observed failure:

```
t = 0 s    gz server (and gz GUI unless headless)
t = 2 s    robot_state_publisher        (needs the URDF serviced by the world)
t = 3 s    pointcloud_to_laserscan      (harmless early; it only works when subscribed)
t = 4 s    slam_toolbox
t = 4 s    static map→odom + map_server + lifecycle (only when loc:=truth)
t = 6 s    Nav2 bringup                 (after SLAM has published its first map)
t = 10 s   tour_node
t = 20 s   nav_live (exploration's Navigator client — needs Nav2 action servers)
t = 30 s   frontier_explore
```

The 20 s gap before `nav_live` is not superstition: Nav2's `lifecycle_manager` can sit in
*Configuring* for many seconds on a loaded machine, and an exploration node that calls
`NavigateToPose` before the action server exists fails instantly and (in an early version)
retried forever against a dead server. The same race produced a 25-minute run that issued zero
goals; Chapter 9 §9.5 shows the guard that now turns that failure into a loud error instead of
an empty map.

## 5.3 The bridge contract

`ros_gz_bridge` bridges ten topics. In 3D mode exactly one of them changes:

| Topic | Type | 2D | 3D |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | ✓ | ✓ |
| `/odom` | `nav_msgs/Odometry` | ✓ | ✓ |
| `/odom_wheels` | `nav_msgs/Odometry` | ✓ | ✓ |
| `/imu` | `sensor_msgs/Imu` | ✓ | ✓ |
| `/joint_states` | `sensor_msgs/JointState` | ✓ | ✓ |
| `/cmd_vel` | `geometry_msgs/Twist` | ✓ | ✓ |
| `/scan` | `sensor_msgs/LaserScan` | ✓ (direct) | **✗ never** |
| `/scan/points` | `sensor_msgs/PointCloud2` | ✗ | ✓ |
| `/camera/image_raw` | `sensor_msgs/Image` | ✓ | ✓ |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ✓ | ✓ |

The 3D row marked "never" is a rule with a mechanism behind it (Chapter 6 §6.5) — the gz
64-line sensor emits *both* a LaserScan and a point cloud, and bridging the wrong one would
feed SLAM a second, silently conflicting source.

## 5.4 Reality check before every launch

`preflight.sh` is the gate between "I think nothing is running" and "I am about to start a
second physics engine". It checks three things and refuses to be optimistic:

```
[1] count gz processes, print age + partition + domain of each
    → an "orphan gz" (server alive, bridge dead) burns CPU and VRAM while publishing nothing
[2] publisher count of /odom, /scan, /clock must each be exactly 1
    /cmd_vel publisher count is printed (>4 usually means a leftover tour or teleop node)
[3] no duplicate node names (two full stacks look exactly like one broken stack)
```

Its most useful output is the distinction between *just starting* (gz younger than 45 s, no
data yet — wait 20–30 s and re-run) and *orphaned* (gz older than 45 s with no data — the
bridge died and the physics engine is idling).

## 5.5 Isolation: how experiments avoid touching the live map

Every experiment driver run allocates a fresh DDS domain and a fresh Gazebo partition:

```
ROS_DOMAIN_ID=120..129, 130, 140, 150/151, 160, 161 …      IGN_PARTITION=ab120, ab160, …
```

Two consequences, both required for trustworthy measurements: (1) a run cannot accidentally
subscribe to the live stack or overwrite the live map, and (2) two runs can be inspected
side-by-side afterwards, because their logs and map prefixes are distinct
(`..._ab_1025_2047_0.40_1_checkpoint_48`). The rule is enforced socially — it is the first
thing the driver does, and every acceptance log prints its domain so ambiguity is impossible
after the fact.

> **Takeaways.** One launch file, all behaviour gated by switches. Timers encode observed
> failure modes, not taste. The bridge table is a contract with one forbidden cell. Preflight
> distinguishes "loading" from "orphaned". Isolation is the first line of defence for every
> measurement.
