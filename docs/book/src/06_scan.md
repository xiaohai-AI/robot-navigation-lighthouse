# The laser contract: from photons to `/scan`

## 6.1 Two sensors, one message

![The scan pipeline in both modes. Note the forbidden edge: in 3D mode the gz LaserScan is
never bridged.](figures/scan_chain.png){width=92%}

The entire stack above Nav2 consumes one message type. Everything in this chapter exists to
guarantee that `/scan` means exactly one thing.

**2D mode.** The gz single-line lidar publishes `LaserScan` as topic `scan`; the bridge maps it
straight to ROS `/scan`. Nothing else touches the topic.

**3D mode.** The gz 64-line lidar publishes `PointCloudPacked` on `scan` and — this is the trap
— *also* a derived `LaserScan`. The bridge is configured to accept **only** the point cloud,
renamed `/scan/points`. A separate node projects 3D into 2D:

```yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: base_scan
    min_height: -0.75       # below the caster contact patch
    max_height: +0.05       # a horizontal slice just above the floor
    angle_min: -3.14159
    angle_max: +3.14159
    angle_increment: 0.0174533    # 360 rays
    range_min: 0.12
    range_max: 12.0
    use_inf: true
```

`range_min`/`range_max` are deliberately identical to `slam_toolbox`'s
`min_laser_range`/`max_laser_range` (0.12 / 12.0) and to the sensor's own limits: a mismatch
here produces a map that is subtly scaled, and scaled maps are the hardest maps to debug
because they look fine.

Measured end-to-end in 3D mode: `/scan` at **8.50 Hz**, `/scan/points` at **8.40 Hz**, **359 of
360** rays returning an echo at the spawn pose, RTF **0.98**, and exactly **one** publisher on
`/scan` (`/pointcloud_to_laserscan`). The corresponding 2D numbers are 0.99 RTF and 8.5+ Hz.

## 6.2 Why a 2D slice, and what it costs

Projecting 23,040 points per frame to 360 rays throws away 98.4 % of the information. That is
the price of a contract: every consumer above works in 2D, the costmaps are 2D, and
`slam_toolbox` is a 2D graph SLAM.

What the slice cannot see:

* anything entirely above +0.05 m and below the next structure (e.g. a table top, a shelf,
  an overhanging pallet);
* anything below −0.75 m (nothing, by construction);
* therefore the costmap is blind to overhangs, and a robot could in principle drive under an
  object it cannot pass beneath in 3D.

This is not a bug, it is a documented boundary of the design, and it is the reason a warehouse
world would force an architectural change (Chapter 15 §15.3). The 3D cloud is already being
published and bridged; nothing consumes it *yet*.

## 6.3 The failure that motivated an assertion

During phase 3, a launch edit briefly bridged the gz LaserScan in 3D mode as well. The result
was not an error message. It was:

* SLAM receiving two `/scan` streams whose timestamps and ray counts differed slightly;
* the map ghosting (double walls) in the corridors;
* Nav2 planning through geometry that one of the streams did not see;
* an afternoon of hypothesis-testing the wrong subsystem.

The failure mode is uniquely nasty because DDS happily delivers both, and which one a
subscriber uses depends on arrival order. Nothing in ROS warns about it.

## 6.4 The guard: one publisher, and it must be the right one

`tools/scan_pub_guard.py` runs as a short-lived node inside the launch (resident with
`--watch`), and judges the topic, not the count alone:

| Observation | Verdict | Why |
|---|---|---|
| 0 publishers | fail | projector or bridge never started; but silent during the startup grace window |
| 1 publisher, node name matches `--expect` | pass | the normal case |
| 1 publisher, wrong node name | fail | right count, wrong source — e.g. the bridge when the projector was expected |
| ≥ 2 publishers | fail **immediately** | never transient; always a wiring error |

The three design choices worth keeping if you port this elsewhere:

1. **Check identity, not just count.** A count-only guard passes the most likely real
   mistake (wrong bridge line), because it still yields exactly one publisher.
2. **Never delay the ≥2 verdict.** Startup grace exists for "not yet", never for
   "impossible".
3. **Stay resident.** With `--watch` the guard also catches a rogue publisher started later
   from another terminal, and it recovers by itself when that publisher exits, so nobody has
   to relaunch the stack.

Evidence from the four live scenarios (Chapter 13 §13.5 reproduces the commands):

```
3D normal            ✅ /scan publisher 1: /pointcloud_to_laserscan
3D negative          ❌ /scan has 2 publishers: /_ros2cli_12764, /pointcloud_to_laserscan
                       (a second publisher was injected on purpose; caught within 30 s)
rogue publisher gone ✅ recovered without relaunch
2D normal            ✅ /scan publisher 1: /gz_bridge
```

Plus six offline unit cases for the judging function, so the logic can be changed without a
simulator (0 publishers, correct single, wrong single, mixed pair, namespaced node, and the
2D expectation).

> **Takeaways.** One message type is the spine of the stack. The 3D sensor is degraded to a 2D
> slice by contract, and that boundary is documented rather than denied. Duplicate publishers
> fail silently by nature, so the project asserts both *how many* and *which one*.
