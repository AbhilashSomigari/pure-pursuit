# Lab 4: Pure Pursuit

Waypoint following for the F1TENTH car with the pure pursuit algorithm,
implemented as the ROS 2 Python package `pure_pursuit_pkg`.

## Repository structure

```
.
├── pure_pursuit_pkg/                  # ROS 2 ament_python package
│   ├── pure_pursuit_pkg/
│   │   ├── pure_pursuit_node.py       # ROS node: odom in, /drive + RViz markers out
│   │   └── pure_pursuit.py            # pure pursuit math (no ROS, testable on its own)
│   ├── waypoints/data.csv             # Levine loop waypoints (x, y, heading)
│   ├── config/params.yaml             # tunable parameters
│   ├── launch/pure_pursuit.launch.py
│   ├── test/                          # ament flake8 / pep257 / copyright tests
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── maps/                              # Levine map for f1tenth_gym_ros
    ├── levine.png
    └── levine.yaml
```

## Algorithm

### Path preparation (once, at start-up)

The waypoints in `data.csv` were recorded by driving the car, so they contain
noise. At the start of the recording the car moves back and forth about 1 m
before setting off, and localization jitter makes some points step
backwards. `prepare_path` cleans this up in three steps:

1. **Forward-only filter**: a point is kept only if it is at least 0.1 m
   from the last kept point and ahead of it along the direction of travel.
   This removes every backward step. The cleaned loop is 63.6 m long.
2. **Uniform resampling** every `waypoint_spacing` (0.05 m) of arc length.
   Nearest-waypoint search windows are then measured in a fixed distance.
3. **Circular moving average** over `smoothing_window` waypoints, so the
   smoothing wraps around the closed loop.

**Speed profile** (`speed_profile`): the speed at each waypoint is limited
by lateral acceleration, `v = sqrt(max_lateral_accel / |κ|)`, and clipped to
`[min_speed, max_speed]`. The curvature κ comes from three points 0.5 m apart.
A backward pass then enforces `v_i² ≤ v_{i+1}² + 2·max_decel·Δs`. The car
starts braking before a corner instead of reacting only once it is in it.

### Every pose message (`pose_callback`)

1. **Current waypoint**: the waypoint closest to the car. After the first
   call the search only covers a window just ahead of the previous index, so
   the car cannot jump to a part of the loop that is close in space but far
   along the path.
2. **Goal point**: walking forward from the current waypoint, find the
   first path segment that crosses the circle of radius L around the car.
   The goal is the exact circle–segment intersection, not just the nearest
   waypoint. The lookahead grows slightly with speed:
   `L = lookahead_distance + lookahead_speed_gain · speed`.
3. **Vehicle frame**: rotate the goal by the car's yaw to get `(x, y)`
   (forward, left).
4. **Steering**: curvature of the arc through the goal,
   $\gamma = 2y / L^2$, converted to a front-wheel angle with the bicycle
   model, `δ = atan(wheelbase · γ)`, and clipped to `max_steering`.
5. **Speed**: profile speed at the current waypoint × `speed_scale`.
6. **Output**: `AckermannDriveStamped` on `/drive`. RViz markers show all
   waypoints on `/env_viz` (blue, republished at 1 Hz) and the current goal
   point on `/dynamic_viz` (red).

## Build and run (simulator)

```bash
# inside the sim container / ROS 2 workspace
cp -r pure_pursuit_pkg ~/sim_ws/src/
cd ~/sim_ws
colcon build --packages-select pure_pursuit_pkg
source install/setup.bash

# terminal 1: simulator on the Levine map
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# terminal 2: pure pursuit
ros2 launch pure_pursuit_pkg pure_pursuit.launch.py   # uses config/params.yaml
# or
ros2 run pure_pursuit_pkg pure_pursuit_node
```

The waypoints are installed with the package and found through
`get_package_share_directory`, so no absolute paths need editing. To use a
different file, set `waypoints_path` in `params.yaml`. The file needs at
least `x, y` columns; extra columns are ignored.

In RViz, add two **Marker** displays on `/env_viz` and `/dynamic_viz` to see
the path and the goal point.

Run the tests with `colcon test --packages-select pure_pursuit_pkg`.

### Map

`maps/levine.png` / `levine.yaml` is the map the waypoints were recorded on.
It is the default map of `f1tenth_gym_ros`; if yours was changed, copy these
files into `f1tenth_gym_ros/maps/` and set `map_path` in
`f1tenth_gym_ros/config/sim.yaml`. The loop runs counter-clockwise from the
default spawn `(0, 0, 0)`.

## Running on the car

Use the particle filter for localization and slow the profile down:

```yaml
odom_topic: /pf/pose/odom
speed_scale: 0.4     # start low, raise step by step
```

Waypoints for a new map (for example `pure_pursuit.pgm` from
`slam_toolbox`) go in `waypoints/` or are given through `waypoints_path`.

## Parameters

All parameters live in `pure_pursuit_pkg/config/params.yaml`.

| Parameter | Default | Meaning |
|---|---|---|
| `odom_topic` | `/ego_racecar/odom` | pose source (`/pf/pose/odom` on the car) |
| `drive_topic` | `/drive` | Ackermann command topic |
| `waypoints_path` | `''` | CSV path; empty = `waypoints/data.csv` in the package |
| `waypoint_spacing` | 0.05 | resampling step (m) |
| `smoothing_window` | 9 | moving-average window over resampled waypoints |
| `search_window` | 100 | waypoints searched ahead of the last nearest index |
| `lookahead_distance`, `lookahead_speed_gain` | 0.9, 0.05 | `L = 0.9 + 0.05 · speed` (m) |
| `wheelbase` | 0.3302 | f1tenth_gym wheelbase (m) |
| `max_steering` | 0.4189 | steering limit (rad) |
| `max_speed`, `min_speed` | 5.0, 1.5 | speed profile limits (m/s) |
| `max_lateral_accel`, `max_decel` | 5.0, 5.0 | speed profile limits (m/s²) |
| `speed_scale` | 1.0 | multiplies the whole profile |

## Results (offline simulation)

The parameters were tuned in a lightweight offline simulator before running
in ROS. It uses a kinematic bicycle model with f1tenth_gym geometry and
steering-rate limits, a full-footprint collision check against `levine.png`,
and an artificial actuation delay.

- About 14.6 s per lap at an average of 4.25 m/s: 5 m/s on the straights,
  2.1–2.3 m/s in the four corners. There is a brief dip to about 3 m/s at
  x ≈ −2.2 on the bottom straight, where the recorded line has a small
  kink.
- Mean distance from the path 4–5 cm. The footprint stays at least 0.31 m
  from the walls, and at least 0.20 m when spawned off the path.
- Completes laps with up to 80 ms of actuation delay. Lookaheads shorter
  than about 0.8 m start to oscillate once delay is added. A large
  `lookahead_speed_gain` (≥ 0.15) cuts the inside of corners. That is why
  the lookahead is mostly constant.

The kinematic model has no tyre slip. If the car slides wide in
f1tenth_gym, lower `max_lateral_accel` or `speed_scale`.
