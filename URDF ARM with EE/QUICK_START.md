# GP7 Robot - Quick Start Guide

## Installation (5 minutes)

1. **Copy package to your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/gp7_robot_description .
   ```

2. **Copy mesh files (IMPORTANT!):**
   ```bash
   cp -r /path/to/original/GP7_Robot.SLDASM/meshes ~/ros2_ws/src/gp7_robot_description/
   ```

3. **Build:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gp7_robot_description
   source install/setup.bash
   ```

## Quick Launch Commands

### Visualize Robot in RViz2
```bash
ros2 launch gp7_robot_description display.launch.py
```

### Simulate in Gazebo
```bash
ros2 launch gp7_robot_description gazebo.launch.py
```

## What Changed from ROS1?

| Aspect | ROS1 | ROS2 |
|--------|------|------|
| **Package Name** | `GP7 Robot.SLDASM` | `gp7_robot_description` |
| **Launch Format** | XML (`.launch`) | Python (`.launch.py`) |
| **Build System** | catkin | ament_cmake |
| **Launch Command** | `roslaunch` | `ros2 launch` |
| **RViz** | `rviz` | `rviz2` |
| **Build Command** | `catkin_make` | `colcon build` |

## File Structure

```
gp7_robot_description/
├── CMakeLists.txt          ← ROS2 build configuration
├── package.xml             ← ROS2 package metadata
├── README.md               ← Detailed documentation
├── MIGRATION_GUIDE.md      ← Conversion details
├── setup.sh                ← Automated setup script
├── urdf/
│   └── gp7_robot.urdf      ← Robot description (paths updated)
├── meshes/                 ← YOU MUST COPY THESE!
│   ├── base_link.STL
│   ├── link_1.STL
│   └── ... (8 STL files total)
├── launch/
│   ├── display.launch.py   ← RViz visualization (Python)
│   └── gazebo.launch.py    ← Gazebo simulation (Python)
└── rviz/
    └── display.rviz        ← RViz2 configuration
```

## Troubleshooting

### "Package not found"
```bash
source ~/ros2_ws/install/setup.bash
```

### "Meshes not loading"
Copy meshes folder from original package and rebuild.

### "Launch file not executable"
```bash
chmod +x ~/ros2_ws/src/gp7_robot_description/launch/*.py
```

## Next Steps

1. ✅ Install and test basic visualization
2. 📝 Update joint limits in URDF (currently set to 0)
3. 🎮 Add controllers for motion control
4. 🔧 Customize RViz configuration
5. 🌍 Create custom Gazebo worlds

## Need Help?

- See `README.md` for detailed documentation
- See `MIGRATION_GUIDE.md` for ROS1 → ROS2 conversion details
- Check ROS2 documentation: https://docs.ros.org/

---

**Pro Tip:** Use the included `setup.sh` script for automated installation:
```bash
cd gp7_robot_description
./setup.sh
```
