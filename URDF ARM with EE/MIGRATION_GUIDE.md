# ROS1 to ROS2 Migration Guide for GP7 Robot

## Overview

This document outlines the changes made to convert the GP7 Robot package from ROS1 to ROS2.

## Package Structure Changes

### Directory Structure

**ROS1:**
```
GP7 Robot.SLDASM/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   └── GP7 Robot.SLDASM.urdf
├── meshes/
├── launch/
│   ├── display.launch
│   └── gazebo.launch
└── config/
```

**ROS2:**
```
gp7_robot_description/
├── package.xml
├── CMakeLists.txt
├── README.md
├── setup.sh
├── urdf/
│   └── gp7_robot.urdf
├── meshes/
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py
└── rviz/
    └── display.rviz
```

## Key File Changes

### 1. package.xml

**Changes:**
- Format updated from `format="2"` to `format="3"`
- Build tool changed from `catkin` to `ament_cmake`
- Package name simplified: `GP7 Robot.SLDASM` → `gp7_robot_description`
- Dependencies updated to ROS2 equivalents
- Added test dependencies

**ROS1:**
```xml
<package format="2">
  <n>GP7 Robot.SLDASM</n>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roslaunch</depend>
  <depend>rviz</depend>
</package>
```

**ROS2:**
```xml
<package format="3">
  <n>gp7_robot_description</n>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2. CMakeLists.txt

**Changes:**
- Minimum CMake version: `2.8.3` → `3.8`
- Build system: `catkin` → `ament_cmake`
- Project name simplified
- Updated install directives

**ROS1:**
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(GP7 Robot.SLDASM)
find_package(catkin REQUIRED)
catkin_package()
```

**ROS2:**
```cmake
cmake_minimum_required(VERSION 3.8)
project(gp7_robot_description)
find_package(ament_cmake REQUIRED)
ament_package()
```

### 3. Launch Files

**Major Change:** XML format → Python format

**ROS1 (display.launch):**
```xml
<launch>
  <arg name="model" />
  <param name="robot_description"
    textfile="$(find GP7 Robot.SLDASM)/urdf/GP7 Robot.SLDASM.urdf" />
  <node name="joint_state_publisher_gui"
    pkg="joint_state_publisher_gui"
    type="joint_state_publisher_gui" />
</launch>
```

**ROS2 (display.launch.py):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = PathJoinSubstitution([
        FindPackageShare('gp7_robot_description'),
        'urdf', 'gp7_robot.urdf'
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    return LaunchDescription([robot_state_publisher_node])
```

### 4. URDF File

**Changes:**
- Package references updated: `package://GP7 Robot.SLDASM/` → `package://gp7_robot_description/`
- Robot name simplified: `GP7 Robot.SLDASM` → `gp7_robot`
- File path: `GP7 Robot.SLDASM.urdf` → `gp7_robot.urdf`

**No structural changes to robot model itself**

## Command Equivalents

### Building

**ROS1:**
```bash
cd ~/catkin_ws
catkin_make
# or
catkin build GP7\ Robot.SLDASM
```

**ROS2:**
```bash
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description
```

### Sourcing

**ROS1:**
```bash
source ~/catkin_ws/devel/setup.bash
```

**ROS2:**
```bash
source ~/ros2_ws/install/setup.bash
```

### Launching

**ROS1:**
```bash
roslaunch GP7\ Robot.SLDASM display.launch
roslaunch GP7\ Robot.SLDASM gazebo.launch
```

**ROS2:**
```bash
ros2 launch gp7_robot_description display.launch.py
ros2 launch gp7_robot_description gazebo.launch.py
```

### Topics

**ROS1:**
```bash
rostopic list
rostopic echo /joint_states
```

**ROS2:**
```bash
ros2 topic list
ros2 topic echo /joint_states
```

## ROS2-Specific Features Added

### 1. Launch Arguments
```bash
# Use simulation time
ros2 launch gp7_robot_description display.launch.py use_sim_time:=true

# Specify Gazebo world
ros2 launch gp7_robot_description gazebo.launch.py world:=/path/to/world.world
```

### 2. Parameter Format
ROS2 uses a different parameter format in launch files and code.

**ROS1:**
```xml
<param name="use_sim_time" value="true"/>
```

**ROS2:**
```python
parameters=[{'use_sim_time': True}]
```

### 3. Static Transform Publisher

**ROS1:**
```xml
<node pkg="tf" type="static_transform_publisher"
      args="0 0 0 0 0 0 base_link base_footprint 40" />
```

**ROS2:**
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
)
```

## Dependency Changes

| ROS1 Package | ROS2 Package |
|--------------|--------------|
| catkin | ament_cmake |
| roslaunch | (built into ROS2) |
| rviz | rviz2 |
| tf | tf2_ros |
| gazebo_ros | gazebo_ros_pkgs |
| rostopic | (built into ros2 CLI) |

## Backward Compatibility Notes

### What Doesn't Work

1. **XML launch files**: Must be converted to Python
2. **roslaunch**: Use `ros2 launch` instead
3. **ROS1 parameter server**: ROS2 uses per-node parameters
4. **Package names with spaces/dots**: Should use underscores

### What Requires Updates

1. **Custom messages**: Need to be rebuilt with ament_cmake
2. **Python nodes**: Update imports (`rospy` → `rclpy`)
3. **C++ nodes**: Update includes and client library calls
4. **Launch files**: Complete rewrite to Python

## Testing the Conversion

### 1. Build Test
```bash
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description
```

### 2. Display Test
```bash
ros2 launch gp7_robot_description display.launch.py
```
- Should open RViz2 with robot model
- Joint state publisher GUI should allow joint manipulation
- Robot should update in real-time

### 3. Gazebo Test
```bash
ros2 launch gp7_robot_description gazebo.launch.py
```
- Should launch Gazebo with robot spawned
- Robot should be visible at origin
- Static TF should be published

### 4. Topic Test
```bash
# In another terminal
ros2 topic list
ros2 topic echo /joint_states
```

## Common Issues and Solutions

### Issue 1: Meshes Not Loading
**Solution:** Copy meshes folder to package and rebuild

### Issue 2: Package Not Found
**Solution:** Ensure workspace is built and sourced:
```bash
colcon build --packages-select gp7_robot_description
source install/setup.bash
```

### Issue 3: Python Import Errors
**Solution:** Check Python3 and ROS2 packages are installed

### Issue 4: Launch File Permission Denied
**Solution:** Make launch files executable:
```bash
chmod +x launch/*.py
```

## Additional Resources

- [ROS2 Migration Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ament_cmake Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)

## Summary

The conversion maintains full functionality while modernizing the package for ROS2:
- ✅ Robot model unchanged
- ✅ All visualization features preserved
- ✅ Gazebo simulation compatible
- ✅ Improved launch file flexibility
- ✅ Better dependency management
- ✅ ROS2 best practices followed
