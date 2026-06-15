# increase_clearance

Nav2 recovery behavior plugin. Moves the robot away from nearby obstacles using laser scans and the dynamic costmap footprint.

## Build

```bash
cd ~/ament_ws
colcon build --symlink-install --packages-select increase_clearance
source install/setup.bash
```

## Nav2 setup

Add to `behavior_server` params:

```yaml
behavior_plugins: [..., "increase_clearance"]
increase_clearance:
  plugin: "increase_clearance::IncreaseClearance"
  scan_topic: "/scan_filtered"
  footprint_topic: "local_costmap/published_footprint"
  influence_radius: 0.5
  safe_clearance_threshold: 0.35
  max_linear_vel: 0.25
  linear_accel_limit: 0.5
  force_gain: 0.1
  min_force_distance: 0.05
  time_allowance: 30.0
  publish_debug_markers: true
  motion_model: "omni"   # diff/ackermann not implemented yet
```

`behavior_server` must be **active** (lifecycle).

## Behavior tree recovery

Add to the `RoundRobin` recovery block in your BT XML:

```xml
<IncreaseClearance error_code_id="{increase_clearance_error_code}"/>
```

Register the BT node and point `bt_navigator` at your XML in `nav2_param.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - increase_clearance_action_bt_node
    default_nav_to_pose_bt_xml: "/path/to/increase_clearance/config/navigate_to_pose_w_replanning_and_recovery.xml"
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
      - increase_clearance_error_code
```

After install, the XML is at:
`share/increase_clearance/config/navigate_to_pose_w_replanning_and_recovery.xml`

Recovery order in the bundled XML: clear costmaps → **increase_clearance** → spin → wait → backup.

## Run

```bash
ros2 action send_goal /increase_clearance increase_clearance/action/IncreaseClearance "{}"
```

Succeeds when `min_clearance > safe_clearance_threshold`. Fails on timeout or missing scan/footprint.

## How it works

1. Read laser scan + footprint polygon
2. For each scan point near the footprint, add repulsive force (`1/d²`)
3. Sum forces → `cmd_vel` (omni, `omega_z = 0`)
4. Stop when clearance is safe

## Debug (RViz)

Subscribe to `/behavior_server/increase_clearance/debug_markers` (MarkerArray, frame `base_link`):

- Green — footprint
- Yellow — scan hits in range
- Blue — net force
- Cyan — commanded velocity

## Params

| Param | Default | Meaning |
|---|---|---|
| `influence_radius` | 0.5 | Ignore points farther than this from footprint |
| `safe_clearance_threshold` | 0.35 | Exit when min clearance exceeds this |
| `force_gain` | 0.1 | Force → velocity scale |
| `max_linear_vel` | 0.25 | Speed cap |
| `time_allowance` | 30.0 | Max run time (s) |
| `motion_model` | `omni` | `diff` / `ackermann` = TODO, falls back to omni |

## Topics

- **Sub:** `/scan_filtered`, `local_costmap/published_footprint`
- **Pub:** `cmd_vel` (via behavior_server), `~/debug_markers`

## Notes

- Pure repulsion pushes **away from obstacles**, not toward open gaps.
- Footprint should update when the arm moves (`robot_footprint_publisher`).
- Tune `influence_radius` and `force_gain` if motion is too aggressive.

## Video of calling the action server.


https://github.com/user-attachments/assets/ba3884b9-7a06-4253-baa1-a4dc099b532f


