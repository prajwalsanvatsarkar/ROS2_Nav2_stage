# ROS2 Nav2 Stage Project

This project demonstrates **ROS 2 Navigation (Nav2)** using the TurtleBot3 in **Gazebo Stage4**.  
It includes a custom package `my_nav2` with a `goal_pub` node and an optional `init_node` that automatically sets the initial pose for AMCL.

---

## Features
- Launches **Gazebo Stage4** with TurtleBot3
- Brings up **Nav2 (Navigation2) stack** with your own map
- Publishes navigation goals automatically (`goal_pub.py`)
- Optionally auto-publishes the robot’s initial pose (`init_node.py`) so AMCL initializes without manual RViz clicks
- Clean launch file with environment setup (ROS Domain ID, RMW, etc.)

---

## Requirements
- ROS 2 Humble (or compatible)
- `turtlebot3_gazebo` and `nav2_bringup` packages installed
- A valid TurtleBot3 model (`burger`, `waffle`, or `waffle_pi`)

---

## Build

```bash
# 1) Create a workspace if not already
mkdir -p ~/stage_project/src
cd ~/stage_project/src

# 2) Clone this repo
git clone https://github.com/prajwalsanvatsarkar/ROS2_Nav2_stage.git

# 3) Build with colcon
cd ..
colcon build --symlink-install

# 4) Source
source install/setup.bash
```
## Run
Launch everything (Gazebo + Nav2 + RViz + goal publisher)
```bash
# from workspace root
cd ~/stage_project
source install/setup.bash

# set your TB3 model
export TURTLEBOT3_MODEL=burger

# set ROS env (avoids multicast warning and DDS conflicts)
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7

# launch
ros2 launch my_nav2 goal_pub.launch.py
```
## Repo Layout
```bash
ROS2_Nav2_stage/
├─ src/
│  └─ my_nav2/
│     ├─ launch/
│     │   └─ goal_pub.launch.py
│     ├─ maps/
│     │   └─ myfirstmap.yaml
│     ├─ my_nav2/
│     │   ├─ goal_pub.py
│     │   └─ init_node.py
│     ├─ package.xml
│     ├─ setup.py
│     ├─ setup.cfg
│     └─ resource/
│         └─ my_nav2
├─ .gitignore
└─ README.md
```

## License
```bash
This project is licensed under the MIT License.
```

## Author
```bash
Built By Prajwal Sanvatsarkar
```
