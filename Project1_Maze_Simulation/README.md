# MTRX3760 Project 1 Maze Simulation

This bundle contains the completed Gazebo world and launch files for the
initial Project 1 TurtleBot3 right-wall-following maze.

## Included files

```text
turtlebot3_gazebo/
├── launch/
│   └── project1_maze.launch.py
└── worlds/
    └── project1_maze.world
```

No `CMakeLists.txt` edit is required in the official ROBOTIS Jazzy package.
Its existing install rule copies the complete `launch`, `models`, `params`,
`rviz`, `urdf` and `worlds` directories.

## Assumptions and current limitations

- ROS 2 Jazzy and the `jazzy` branch of `turtlebot3_simulations` are used.
- The source workspace is assumed to be `~/turtlebot3_ws`.
- The default start pose is `(-0.75, -0.80)` m and the robot initially faces
  Gazebo's positive x direction, placing the south wall on its right.
- The maze wall height is 0.25 m and simulated thickness is 0.01 m.
- The nominal external footprint is 2.00 m square. Because the wall centre
  lines are at ±0.995 m, the clear distance between opposite inner faces is
  1.98 m.
- The initial two-baffle maze is suitable for incremental controller testing.
  Add further boundary-connected walls only after this version works.
- Gazebo has ideal straight walls and clean sensor geometry. Successful
  simulation does not prove tolerance to warped cardboard, wheel slip or real
  LiDAR noise.

## Install the files

From the extracted bundle directory, copy the two supplied files into the
editable source checkout:

```bash
cp turtlebot3_gazebo/worlds/project1_maze.world \
  ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/

cp turtlebot3_gazebo/launch/project1_maze.launch.py \
  ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/
```

Do not copy them into `/opt/ros/jazzy/share`; that directory is an installed
system location rather than the team's editable source repository.

## Build

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select turtlebot3_gazebo

source install/setup.bash
```

## Choose the simulated robot

For a Burger-sized robot with a simulated camera, first check that the course
repository contains the model:

```bash
ls ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```

If `turtlebot3_burger_cam` exists:

```bash
export TURTLEBOT3_MODEL=burger_cam
```

Otherwise, use the camera-equipped Waffle Pi:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

The Waffle Pi has a larger footprint, so verify its clearance before treating
the simulation as representative of the physical maze.

## Launch

```bash
ros2 launch turtlebot3_gazebo project1_maze.launch.py
```

The start position can be overridden without modifying the file:

```bash
ros2 launch turtlebot3_gazebo project1_maze.launch.py \
  x_pose:=-0.75 \
  y_pose:=-0.80
```

## Verify the required sensor topics

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

Expected core topics include `/cmd_vel`, `/odom`, `/scan` and
`/camera/image_raw`. If the camera topic is absent, check that a camera-equipped
model was selected before launching Gazebo.

## Manual geometry test

Before starting the autonomous controller, drive through the maze manually:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Verify that:

1. Every visible wall appears in `/scan`.
2. The robot does not spawn inside collision geometry.
3. Collision and visible wall surfaces coincide.
4. The robot can turn around both baffle ends.
5. The camera continues publishing while the robot moves.

Then stop teleoperation and run the Project 1 controller:

```bash
ros2 run turtlebot3_gazebo turtlebot3_drive
```

Open RViz in another terminal if required:

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Use `odom` as the fixed frame, add a `LaserScan` display using `/scan`, and add
an `Image` display using `/camera/image_raw`.

## Version control

Add the new files to the team's repository rather than modifying the original
empty-world files:

```bash
cd ~/turtlebot3_ws/src/turtlebot3_simulations
git add turtlebot3_gazebo/worlds/project1_maze.world
git add turtlebot3_gazebo/launch/project1_maze.launch.py
git commit -m "Add initial Project 1 Gazebo maze"
```

## Failure checks

### Launch file not found

Rebuild and source the workspace:

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source install/setup.bash
```

### TurtleBot3 model environment variable is missing

Set it in the same terminal used for launch:

```bash
export TURTLEBOT3_MODEL=burger_cam
```

### Camera topic is missing

The ordinary `burger` model does not supply the required camera. Select
`burger_cam` when available, or `waffle_pi` after checking its larger footprint.

### Walls appear but LiDAR or camera topics do not

Launch the world through `project1_maze.launch.py`, not directly with
`gz sim project1_maze.world`. The launch file also starts the robot state
publisher, robot spawner and ROS-Gazebo bridges.
