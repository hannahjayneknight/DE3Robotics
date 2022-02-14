# DE NIRO Simulation
A ROS Workspace With Robot DE NIRO Simulation

## Prerequisite

### Operating System
Ubuntu 16.04

### ROS Distribution
ROS Kinetic

### Package Installation

```sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser git-core python-argparse python-wstool python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers```


## Launch Robot DE NIRO Simulation

1. Launch the environment

```roslaunch deniro_gazebo deniro_world.launch```

2. Spawn robot DE NIRO to the environment

```roslaunch deniro_gazebo deniro_spawn.launch```

## Tutorial

1. Launch DE NIRO Simulation on Gazebo
2. Enable DE NIRO Robot and Move the Arms to the Untuck Position
3. Enable or disable arms via python code
4. DE NIRO gripper control
5. Joint position control
6. Inverse kinematics solver for DE NIRO robot
7. Simple pick and place demo


## Contributor
1. Marion Tormento
2. Gor Nersisyan
3. Roni Permana Saputra
