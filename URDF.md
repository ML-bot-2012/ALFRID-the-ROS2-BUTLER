# ALFRID Robot URDF Documentation

## Overview

The URDF (Unified Robot Description Format) file describes the robot's structure, geometry, and kinematic chain. ALFRID's URDF defines:

- **Base link** (chassis) - 0.30m × 0.20m × 0.15m blue box
- **Laser scanner** - 0.06m radius red cylinder on top
- **Left/Right wheels** - 0.0325m radius dark gray cylinders
- **Caster wheel** - 0.025m radius orange sphere for stability

## File Location

```
~/butler_ros2_ws/src/butler_control/urdf/butler.urdf
```

## URDF Structure

### 1. Materials (Colors)

```xml
<material name="blue">
  <color rgba="0.1 0.3 0.8 1.0"/>  <!-- R G B Alpha -->
</material>
```

**Color Format:** RGBA (Red, Green, Blue, Alpha)
- Range: 0.0 to 1.0
- 1.0 = fully opaque, 0.0 = transparent

### 2. Links (Physical Parts)

Each link represents a rigid body:

```xml
<link name="base_link">
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.30 0.20 0.15"/>
    </geometry>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.30 0.20 0.15"/>
    </geometry>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </collision>
</link>
```

**Key Elements:**
- **inertial** - Mass and inertia tensor (for physics simulation)
- **visual** - What RViz renders
- **collision** - What Nav2 uses for planning
- **origin** - Position relative to parent link (x, y, z in meters; roll, pitch, yaw in radians)

### 3. Joints (Connections)

Joints connect links together:

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <origin xyz="0.08 0 0.23" rpy="0 0 0"/>
</joint>
```

**Types:**
- **fixed** - No movement (laser, wheels)
- **revolute** - Rotates around axis (not used in ALFRID)
- **prismatic** - Slides along axis (not used in ALFRID)

## Geometry Types

### Box
```xml
<geometry>
  <box size="width depth height"/>  <!-- in meters -->
</geometry>
```

### Cylinder
```xml
<geometry>
  <cylinder length="0.08" radius="0.0325"/>
</geometry>
```

### Sphere
```xml
<geometry>
  <sphere radius="0.025"/>
</geometry>
```

## Current Robot Dimensions

| Part | Dimension | Value |
|------|-----------|-------|
| Chassis (box) | Length × Width × Height | 0.30m × 0.20m × 0.15m |
| Laser (cylinder) | Radius × Length | 0.06m × 0.08m |
| Wheels (cylinder) | Radius × Length | 0.0325m × 0.08m |
| Caster (sphere) | Radius | 0.025m |
| Wheel base (left-right) | Distance | 0.20m |
| Wheel base (front-back) | Distance | 0.10m |

## How to Modify the URDF

### Change Robot Size

To make the robot bigger, scale the chassis:

```xml
<!-- Original -->
<box size="0.30 0.20 0.15"/>

<!-- Make it 2x bigger -->
<box size="0.60 0.40 0.30"/>
```

### Change Wheel Size

```xml
<!-- Original wheels -->
<cylinder length="0.08" radius="0.0325"/>

<!-- Larger wheels for rough terrain -->
<cylinder length="0.10" radius="0.05"/>
```

### Change Colors

Update material RGBA values:

```xml
<!-- Make laser green instead of red -->
<material name="red">
  <color rgba="0.0 1.0 0.0 1.0"/>  <!-- Green -->
</material>
```

### Add New Parts

Example: Add a bumper

```xml
<!-- Add to <link name="base_link"> -->
<visual name="bumper">
  <geometry>
    <box size="0.32 0.02 0.05"/>
  </geometry>
  <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
  <material name="yellow"/>
</visual>
```

### Update Joint Positions

Move the laser higher:

```xml
<!-- Original -->
<origin xyz="0.08 0 0.23" rpy="0 0 0"/>

<!-- Move 0.1m higher -->
<origin xyz="0.08 0 0.33" rpy="0 0 0"/>
```

## Loading the URDF in ROS2

### Option 1: Command Line (Direct XML)
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 run robot_state_publisher robot_state_publisher --ros-args \
-p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
```

### Option 2: Launch File
```python
from launch_ros.actions import Node

Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
)
```

## RViz Visualization

1. **Set Fixed Frame** to `base_link`
2. **Add RobotModel display**
3. **Add TF display** to see coordinate frames

## Common Issues

**Issue:** Robot doesn't appear in RViz
- **Solution:** Check that `/tf_static` is publishing (all links should be in transform tree)

**Issue:** Wheels are too small/large
- **Solution:** Adjust cylinder `length` and `radius` in URDF

**Issue:** Colors don't match
- **Solution:** Verify material RGBA values (should be 0.0-1.0, not 0-255)

**Issue:** Laser in wrong position
- **Solution:** Adjust joint origin `xyz` values

## Recommended Modifications for ALFRID

### For Better Nav2 Performance
- Increase `robot_radius` in nav2_params.yaml if robot seems too small in costmap
- Sync collision geometry with actual robot dimensions

### For Better Visualization
- Add a top-mounted camera mount (box geometry)
- Add sensor visualizations (RPLidar point cloud)
- Add motor/servo mounts

### For Simulation
- Add proper inertia values (currently set to defaults)
- Add friction coefficients for wheels
- Add damping for realistic physics

## Reference

- [ROS URDF Documentation](http://wiki.ros.org/urdf)
- [URDF Geometry Reference](http://wiki.ros.org/urdf/XML/geometry)
- [RViz URDF Tutorial](http://wiki.ros.org/rviz/UserGuide/Display%20Types/RobotModel)

---

Property of 5KROBOTICS & MALHAR LABADE © 2026
