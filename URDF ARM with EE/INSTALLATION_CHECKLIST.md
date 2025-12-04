# GP7 Robot ROS2 - Installation Checklist

## Pre-Installation

### Environment Check
- [ ] ROS2 is installed (Humble/Iron/Rolling)
  ```bash
  echo $ROS_DISTRO
  ```
- [ ] ROS2 is sourced
  ```bash
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```
- [ ] Workspace exists or can be created
  ```bash
  mkdir -p ~/ros2_ws/src
  ```

### Dependencies Check
- [ ] robot_state_publisher installed
- [ ] joint_state_publisher installed
- [ ] joint_state_publisher_gui installed
- [ ] rviz2 installed
- [ ] xacro installed
- [ ] gazebo_ros_pkgs installed (for simulation)

**Quick install all:**
```bash
sudo apt install \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

## Installation Steps

### 1. Copy Package
- [ ] Copy `gp7_robot_description` folder to workspace
  ```bash
  cp -r gp7_robot_description ~/ros2_ws/src/
  ```

### 2. Copy Meshes (CRITICAL!)
- [ ] Locate original meshes folder
- [ ] Copy to new package
  ```bash
  cp -r /path/to/original/meshes ~/ros2_ws/src/gp7_robot_description/
  ```
- [ ] Verify 8 STL files exist:
  - [ ] base_link.STL
  - [ ] link_1.STL
  - [ ] link_2.STL
  - [ ] link_3.STL
  - [ ] link_4.STL
  - [ ] link_5.STL
  - [ ] link_6.STL
  - [ ] link_7.STL
  - [ ] link_8.STL (if applicable)

### 3. Make Scripts Executable
- [ ] Set execute permissions
  ```bash
  cd ~/ros2_ws/src/gp7_robot_description
  chmod +x setup.sh
  chmod +x launch/*.py
  ```

### 4. Build Package
- [ ] Navigate to workspace root
  ```bash
  cd ~/ros2_ws
  ```
- [ ] Build the package
  ```bash
  colcon build --packages-select gp7_robot_description
  ```
- [ ] Check for build errors
- [ ] Build completed successfully ✓

### 5. Source Workspace
- [ ] Source the installation
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- [ ] Verify package is found
  ```bash
  ros2 pkg list | grep gp7_robot_description
  ```

## Testing

### Test 1: Package Found
- [ ] Run: `ros2 pkg list | grep gp7_robot`
- [ ] Expected: Package appears in list

### Test 2: Launch Files Listed
- [ ] Run: `ros2 launch gp7_robot_description` (tab complete)
- [ ] Expected: Shows `display.launch.py` and `gazebo.launch.py`

### Test 3: Display Launch
- [ ] Run: `ros2 launch gp7_robot_description display.launch.py`
- [ ] RViz2 opens ✓
- [ ] Robot model visible ✓
- [ ] Joint state publisher GUI opens ✓
- [ ] Can move joints with sliders ✓
- [ ] Robot updates in RViz ✓

### Test 4: Topics Working
In a new terminal while display.launch.py is running:
- [ ] Run: `ros2 topic list`
- [ ] `/joint_states` topic exists ✓
- [ ] `/robot_description` topic exists ✓
- [ ] `/tf` topic exists ✓
- [ ] Run: `ros2 topic echo /joint_states`
- [ ] Joint data streams when moving sliders ✓

### Test 5: Gazebo Launch
- [ ] Run: `ros2 launch gp7_robot_description gazebo.launch.py`
- [ ] Gazebo opens ✓
- [ ] Robot spawns in world ✓
- [ ] Robot is visible (meshes loaded) ✓
- [ ] No errors in terminal ✓

### Test 6: Robot State Publisher
- [ ] Run: `ros2 node list`
- [ ] `/robot_state_publisher` node running ✓
- [ ] Run: `ros2 node info /robot_state_publisher`
- [ ] Publishes to `/tf` and `/tf_static` ✓

## Troubleshooting

### Build Fails
- [ ] Check all dependencies installed
- [ ] Check ROS2 is sourced
- [ ] Check package.xml is valid XML
- [ ] Read error messages carefully

### Meshes Not Visible
- [ ] Verify meshes folder exists
- [ ] Check 8 STL files are present
- [ ] Rebuild package after adding meshes
- [ ] Source workspace again

### Launch File Won't Execute
- [ ] Check file has execute permissions
- [ ] Check shebang line: `#!/usr/bin/env python3`
- [ ] Check Python syntax

### Package Not Found
- [ ] Check workspace is built
- [ ] Source the workspace: `source install/setup.bash`
- [ ] Check package name matches in all files

### RViz2 Doesn't Show Robot
- [ ] Check Fixed Frame is set to `base_link`
- [ ] Check RobotModel display is enabled
- [ ] Check robot_description topic
- [ ] Try resetting RViz config

## Post-Installation

### Configuration
- [ ] Update maintainer email in package.xml
- [ ] Review and update joint limits in URDF
- [ ] Customize RViz configuration
- [ ] Add to source command in ~/.bashrc (optional)
  ```bash
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
  ```

### Documentation Review
- [ ] Read README.md
- [ ] Review QUICK_START.md
- [ ] Check MIGRATION_GUIDE.md if coming from ROS1
- [ ] Review CONVERSION_SUMMARY.md

### Optional Enhancements
- [ ] Convert URDF to xacro format
- [ ] Add ros2_control configuration
- [ ] Create custom Gazebo worlds
- [ ] Add sensors to robot
- [ ] Configure physics parameters
- [ ] Add custom RViz visualizations

## Verification Complete ✅

When all items are checked, your installation is complete and verified!

### Quick Reference Commands

**Build:**
```bash
cd ~/ros2_ws && colcon build --packages-select gp7_robot_description
```

**Source:**
```bash
source ~/ros2_ws/install/setup.bash
```

**Launch RViz:**
```bash
ros2 launch gp7_robot_description display.launch.py
```

**Launch Gazebo:**
```bash
ros2 launch gp7_robot_description gazebo.launch.py
```

**List Topics:**
```bash
ros2 topic list
```

**View Joint States:**
```bash
ros2 topic echo /joint_states
```

---

## Need Help?

If you encounter issues not covered in this checklist:

1. Check the README.md for detailed documentation
2. Review the MIGRATION_GUIDE.md for ROS1 vs ROS2 differences
3. Check ROS2 documentation: https://docs.ros.org/
4. Verify all dependencies are installed
5. Check terminal output for specific error messages

**Remember:** The most common issue is forgetting to copy the meshes folder!
