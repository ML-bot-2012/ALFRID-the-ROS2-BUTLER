# ALFRID Nav2 Configuration Guide

## Overview

Nav2 is the navigation stack that provides autonomous path planning, collision avoidance, and motion control. This guide covers configuration for ALFRID's 200×200m global costmap and MPPI controller.

## Configuration File Location

```
/home/malharlabade/nav2_params_200x200.yaml
```

## Global Costmap Configuration

### Size and Resolution

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      width: 200          # meters
      height: 200         # meters
      resolution: 0.05    # meters per cell
```

**Explanation:**
- **Width/Height:** Total area the robot can plan within (200m × 200m = 40,000 m²)
- **Resolution:** 0.05m = 5cm per grid cell
  - Smaller = more accurate but slower computation
  - Larger = faster but less precise

**Current Setup:** 200m × 200m with 0.05m resolution = 4,000 × 4,000 cells = 16 million cells (manageable)

### Inflation Radius

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.7    # meters
```

**What it does:**
- Creates a safety buffer around obstacles
- Robot won't get closer than `inflation_radius` to walls
- Current: 0.7m = robot stays 70cm away from walls

**Tuning:**
- **Increase if:** Robot hits obstacles or seems too aggressive
- **Decrease if:** Robot stays too far from walls

### Obstacle Detection

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True      # Clears old obstacles
    marking: True       # Marks new obstacles
    obstacle_max_range: 2.5    # Only detect within 2.5m
    obstacle_min_range: 0.0
```

**Key Parameters:**
- **max_obstacle_height:** Ignores anything above 2.0m (good for detecting furniture)
- **obstacle_max_range:** Only considers obstacles within 2.5m (ignores distant walls)
- **clearing:** If lidar doesn't see it, mark it as free

## Local Costmap Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3
      height: 3
      resolution: 0.05
      rolling_window: true    # Moves with robot
```

**Purpose:** Local planning window around the robot
- **3×3m rolling window** centered on robot
- Moves as robot moves
- Used for immediate collision avoidance

**Why small:** Faster computation, local obstacle avoidance only

## Controller Configuration (MPPI)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0    # 20 Hz control updates
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5     # 0.5 m/s
      lookahead_dist: 0.6         # Look ahead 60cm
      use_collision_detection: true
```

**Parameters:**

| Parameter | Value | Purpose |
|-----------|-------|---------|
| controller_frequency | 20.0 Hz | How often controller updates |
| desired_linear_vel | 0.5 m/s | Target speed |
| lookahead_dist | 0.6 m | How far ahead to plan |
| max_angular_accel | 3.2 rad/s² | Max turning acceleration |

## Planner Configuration

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5              # Goal tolerance in meters
      allow_unknown: false        # Don't plan through unknown areas
```

**GridBased (Dijkstra) Planner:**
- Finds shortest path in costmap
- `tolerance: 0.5` = goal must be within 50cm
- `allow_unknown: false` = won't go through unexplored areas

## Velocity Smoother

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    filter_duration: 0.1
    max_vel: [0.5, 0.0, 2.0]      # [linear_x, linear_y, angular_z]
    max_accel: [2.5, 0.0, 3.2]    # Max accelerations
    max_decel: [-2.5, 0.0, -3.2]  # Max decelerations
```

**Purpose:** Smooth velocity commands to prevent jerky motion

## Common Tuning Scenarios

### Robot Stops Before Obstacles (Too Conservative)

**Reduce inflation_radius:**
```yaml
inflation_radius: 0.5  # from 0.7
```

### Robot Hits Obstacles (Too Aggressive)

**Increase inflation_radius:**
```yaml
inflation_radius: 1.0  # from 0.7
```

### Robot Moves Too Slowly

**Increase desired_linear_vel:**
```yaml
desired_linear_vel: 1.0  # from 0.5 (1.0 m/s)
```

### Robot Oscillates (Unstable)

**Increase lookahead_dist:**
```yaml
lookahead_dist: 1.0  # from 0.6
```

### Robot Overshoots Goals

**Decrease controller_frequency:**
```yaml
controller_frequency: 10.0  # from 20.0
```

## AMCL (Localization)

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2        # Drift in translation
    alpha2: 0.2        # Drift in rotation
    max_particles: 2000
    min_particles: 500
```

**What it does:**
- Estimates robot position using particle filter
- Matches lidar scans to map
- Current: 500-2000 particles (good balance)

**Tuning:**
- **More particles = more accurate but slower** (max 5000)
- **Fewer particles = faster but less accurate** (min 100)

## SLAM Toolbox Parameters

Located in launch file:

```python
{'odom_topic': 'encoder_odom'},
{'map_frame': 'map'},
{'base_frame': 'base_link'},
{'scan_topic': '/scan'}
```

**Important:** 
- Must match actual encoder odometry topic name
- `base_frame` must be `base_link` (consistent with URDF)

## Launch Nav2

### Quick Start
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/home/malharlabade/nav2_params_200x200.yaml
```

### With Custom Parameters
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/malharlabade/nav2_params_200x200.yaml \
  autostart:=true
```

## Testing in RViz

1. **Open RViz2:**
   ```bash
   rviz2
   ```

2. **Set up display:**
   - Fixed Frame: `map`
   - Add displays: Map, Local Costmap, Global Costmap, RobotModel, TF

3. **Send navigation goal:**
   - Click "2D Goal Pose" button
   - Click on map to set goal
   - Robot should plan and navigate

## Monitoring

### Check costmap:
```bash
ros2 topic echo /global_costmap/costmap
```

### Check planned path:
```bash
ros2 topic echo /plan
```

### Check robot position:
```bash
ros2 topic echo /amcl_pose
```

## Common Issues

### Robot doesn't move
- Check if `/cmd_vel` is being published
- Check if motors are receiving commands
- Verify odometry is publishing

### Costmap is empty
- Check if `/scan` topic exists
- Verify SLAM is running and publishing `/map`
- Check if static transforms are published

### Robot goes in circles
- Increase `inflation_radius`
- Reduce `desired_linear_vel`
- Check encoder odometry accuracy

### Poor localization
- Increase AMCL `max_particles`
- Check lidar mounting angle
- Ensure SLAM map quality is good

## Performance Metrics

**Current Setup Benchmarks:**
- Global costmap: 200×200m @ 0.05m resolution
- Control frequency: 20 Hz
- Planning frequency: 20 Hz
- Typical planning time: <100ms

**Expected Performance:**
- Max speed: 0.5 m/s (configurable)
- Min turning radius: ~0.3m
- CPU usage: ~30-40% on Pi5

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [Costmap 2D Plugin Docs](https://docs.nav2.org/configuration/packages/configuring-costmap-2d.html)
- [Controller Server Docs](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
