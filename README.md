# Car\_ROS2 — Custom Car Example Project for ROS 2

An example **custom robotic car** project for ROS 2.

Includes:

* Robot model generated in **URDF/.xacro**
* **Launch files** for **RViz2** visualization and **Gazebo** simulation
* **Custom plugins** for **differential** and **Ackermann** drive
* **Teleop nodes** for joystick and keyboard
* **Dockerfiles** to run the environment in containers



---

## Features

* 🔩 **Parametric `.xacro` model** (easy to extend with sensors and parts)
* 🛰️ **Visualization** using `robot_state_publisher` and `joint_state_publisher(_gui)`
* 🧪 **Gazebo simulation** with example worlds
* ⚙️ **Custom Gazebo plugins**:

  * **Differential** (diff-drive control)
  * **Ackermann** (steering geometry for car-like robots)
* 🎮 **Teleoperation** nodes:

  * Keyboard and joystick
* 🐳 **Dockerfiles** for reproducible setups

---

## Requirements


> Recommended: ROS 2 **Humble** (Ubuntu 22.04) or compatible.

---

## Quick Installation

```bash
# 1) Create a workspace and clone the repo
mkdir -p ~/ws_car/src
cd ~/ws_car/src
git clone https://github.com/Rhobtor/Car_ROS2.git
cd ..

# 2) Install dependencies (as specified by this project)
rosed install -y src

# (Typical ROS 2 alternative, if needed)
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

# 3) Build
colcon build --symlink-install

# 4) Source the workspace
source install/setup.bash   # or setup.zsh
# fish: source install/setup.fish
```

---

## How to Run

### RViz2 Visualization

```bash
ros2 launch car display.launch.py
```

### Gazebo Simulation

```bash
# Default world
ros2 launch car gazebo.launch.py

# Other examples (if present in the launch/ folder)
ros2 launch car gazebo_alone.launch.py
ros2 launch car gazebo_mountain.launch.py
```

### Teleoperation

**Keyboard**

```bash
ros2 run car teleop_keyboard
```

**Joystick**

```bash
# Ensure /dev/input/js0 is accessible to your user
ros2 run car teleop_joy
```

> You may need `udev` rules or user permissions for input devices.

---

## Custom Plugins

This project ships **custom Gazebo plugins**:

* **Differential drive**: diff-drive traction control
* **Ackermann**: car-like kinematics and steering

Plugins are loaded from the robot’s `.xacro` / `.gazebo` configuration via the Gazebo launch files.

---

## Docker

The docker run in low fps , still in progress

**Dockerfiles** are provided for running the project in containers (useful for CI/CD and reproducible environments).

Example:

```bash
# Build image
docker build -t car_ros2 -f dockerfile .

# Run with X11 graphics (Linux)
xhost +local:root
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  car_ros2
# For NVIDIA GPUs, add: --gpus all
```

---

## Project Layout (summary)

```
Car_ROS2/
├─ launch/              # display.launch.py, gazebo*.launch.py, etc.
├─ urdf/                # *.xacro, materials, plugin inserts
├─ worlds/              # Gazebo worlds
├─ src/ | car/          # nodes (teleop keyboard/joystick, utilities)
├─ plugins/             # custom plugin sources (if applicable)
├─ Dockerfile(s)/       # container setup
└─ package.xml          # ROS 2 dependencies
```

---

## Troubleshooting

* Run `source install/setup.bash` in **every new terminal**
* Ensure `gazebo_ros` is installed and GPU drivers support rendering
* For joystick, verify `ls /dev/input/js*` and user permissions
* If you modify plugins, rebuild clean:

  ```bash
  colcon build --symlink-install --cmake-clean-first
  ```

---

## Contributing

PRs and Issues are welcome! Please open an **Issue** describing your bug/feature and include logs or screenshots where helpful.

---

## License

See the repository’s `LICENSE` file (or add your chosen license).
