# GP7 Robot Description for ROS2

This package contains the URDF description and launch files for the GP7 Robot, converted for ROS2.

## Package Contents

- **urdf/**: Contains the robot URDF description file
- **meshes/**: Contains STL mesh files for robot visualization (you need to copy these from your original package)
- **launch/**: ROS2 launch files (Python)
- **rviz/**: RViz2 configuration files

## Prerequisites

- ROS2 (Humble, Iron, or Rolling)
- Required ROS2 packages:
  ```bash
  sudo apt install ros-${ROS_DISTRO}-robot-state-publisher
  sudo apt install ros-${ROS_DISTRO}-joint-state-publisher
  sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui
  sudo apt install ros-${ROS_DISTRO}-rviz2
  sudo apt install ros-${ROS_DISTRO}-xacro
  sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
  ```

## Installation

1. Copy this package to your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/gp7_robot_description .
   ```

2. **Important**: Copy the meshes folder from your original ROS1 package:
   ```bash
   cp -r /path/to/original/meshes ~/ros2_ws/src/gp7_robot_description/
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gp7_robot_description
   source install/setup.bash
   ```

## Usage

### Visualize in RViz2

Launch the robot model with RViz2 and joint state publisher GUI:

```bash
ros2 launch gp7_robot_description display.launch.py
```

This will:
- Start the robot state publisher
- Open the joint state publisher GUI (to control joint positions)
- Launch RViz2 with the robot model

### Simulate in Gazebo

Launch the robot in Gazebo simulator:

```bash
ros2 launch gp7_robot_description gazebo.launch.py
```

To launch with a specific world file:

```bash
ros2 launch gp7_robot_description gazebo.launch.py world:=/path/to/world.world
```

### Use simulation time

If you want to use Gazebo's simulation time:

```bash
ros2 launch gp7_robot_description display.launch.py use_sim_time:=true
```

## Robot Description

The GP7 Robot is an 8-link robotic arm with:
- 6 revolute joints (joint_1 through joint_6)
- 2 prismatic joints (joint_7 and joint_8)

### Joint Configuration

You can view and control the joint states using:

```bash
# View joint states
ros2 topic echo /joint_states

# Publish to joint states (example)
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: ['joint_1'], position: [0.5]}"
```

## File Structure

```
gp7_robot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── gp7_robot.urdf
├── meshes/
│   ├── base_link.STL
│   ├── link_1.STL
│   ├── link_2.STL
│   ├── link_3.STL
│   ├── link_4.STL
│   ├── link_5.STL
│   ├── link_6.STL
│   ├── link_7.STL
│   └── link_8.STL
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py
└── rviz/
    └── display.rviz
```

## Key Changes from ROS1

1. **Launch files**: Converted from XML to Python format (`.launch.py`)
2. **Package format**: Updated to `ament_cmake` build system
3. **Dependencies**: Updated to ROS2 package names
4. **Executables**: Updated node executable names (e.g., `rviz` → `rviz2`)
5. **Parameters**: Using ROS2 parameter format

## Troubleshooting

### Meshes not loading

If you see errors about missing meshes:
1. Ensure the `meshes/` folder is copied to the package
2. Check that mesh file paths in the URDF match your file names
3. Rebuild the package after adding meshes

### Joint limits warning

The current URDF has zero joint limits. You may want to update these in `urdf/gp7_robot.urdf` with appropriate values for your robot.

### RViz2 not showing robot

1. Check that the Fixed Frame is set to `base_link`
2. Ensure RobotModel display is enabled
3. Verify the robot description topic is `/robot_description`

## License

BSD License

## Maintainer

Update the maintainer information in `package.xml`
