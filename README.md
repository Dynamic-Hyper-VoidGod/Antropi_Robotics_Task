# Antropi Robotics — Robotics Software Intern Task

An autonomous navigation demonstration using **ROS 2 Humble, Gazebo Classic, TurtleBot3 Burger and Nav2**.

The robot navigates an obstacle-filled simulation world using a saved occupancy-grid map, AMCL localisation and a custom ROS 2 mission controller.

## Demonstration Video

A screen recording of the TurtleBot3 autonomous navigation demonstration is available below:

[Watch the autonomous navigation demonstration](Screencast%20from%2010-31-2026%2001:04:00%20PM.webm)
https://github.com/Dynamic-Hyper-VoidGod/Antropi_Robotics_Task/blob/main/Screencast%20from%2010-31-2026%2001%3A04%3A00%20PM.webm

## Features

- TurtleBot3 Burger simulation in Gazebo Classic.
- Occupancy-grid map generation using SLAM Toolbox.
- Autonomous navigation using the Nav2 stack.
- TF-based robot pose retrieval (`map` to `base_footprint`).
- Random selection of up to three separated navigation waypoints from free map space.
- Basic waypoint clearance checks to avoid selecting targets close to occupied or unknown cells.
- Sequential goal execution through Nav2's `NavigateToPose` action interface.

## Software requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 packages
- Nav2
- SLAM Toolbox
- Python 3 and colcon

Install the ROS packages if they are not already installed:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-navigation2 ros-humble-slam-toolbox

Build the workspace
Clone this repository into a ROS 2 workspace's src directory, or use the existing workspace layout.
cd ~/Antropi_Robotics/Antropi_Robotics_Task
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash


Set the robot model:
export TURTLEBOT3_MODEL=burger


Run the simulation
Start the TurtleBot3 world in Gazebo:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


Generate a map
In a separate terminal, start SLAM Toolbox:
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True


Drive the robot around the environment using keyboard teleoperation:
ros2 run turtlebot3_teleop teleop_keyboard


Once the environment has been mapped, save the map:
ros2 run nav2_map_server map_saver_cli -f ~/factory_map


This produces factory_map.pgm and factory_map.yaml in the home directory.
Start autonomous navigation
Launch Nav2 using the saved map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/factory_map.yaml


In RViz, use 2D Pose Estimate to initialise the robot's approximate position and orientation on the map. Allow localisation to settle before starting the mission controller.
Run the mission controller
In another terminal:
cd ~/Antropi_Robotics/Antropi_Robotics_Task
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run factory_mission_controller mission_controller


The controller waits for the map and TF localisation, identifies candidate free-space cells, and selects up to three waypoints. It sends each target to Nav2 sequentially and reports navigation feedback in the terminal.
Mission controller approach
The factory_mission_controller package is implemented in Python using rclpy.
1. Map input: subscribes to /map using reliable, transient-local QoS so it can receive the published occupancy grid.
2. Pose estimation: obtains the robot's current position from the TF transform between map and base_footprint.
3. Waypoint selection: scans free occupancy-grid cells, applies a six-cell clearance check, excludes targets within 0.8 m of the robot, and selects up to three targets separated by at least 0.9 m.
4. Goal orientation: calculates a heading from the robot's current position towards each selected target.
5. Navigation: sends each target to Nav2 using the NavigateToPose action and processes feedback and results before advancing.
The clearance and separation values are expressed relative to the map resolution where applicable. With the demonstrated map resolution of 0.05 m/cell, the six-cell clearance radius is approximately 0.30 m.
Known limitations
- Waypoints are selected randomly from the currently available free map cells; this is not a globally optimised coverage or inspection route.
- The controller currently selects up to three targets per mission.
- Nav2 may occasionally abort a waypoint even when its reported remaining distance approaches 0.00 m. Therefore, successful completion of every waypoint is not guaranteed in the current implementation.
- The demonstration uses the standard TurtleBot3 simulation world and a map generated from that environment.

Antropi_Robotics_Task/
├── README.md
└── src/
    └── factory_mission_controller/
        ├── factory_mission_controller/
        │   ├── __init__.py
        │   └── mission_controller.py
        ├── resource/
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py

Author
Aneesh Manu
B.Tech — Electrical and Computer Science Engineering
VIT Chennai
EOF
cat > .gitignore
