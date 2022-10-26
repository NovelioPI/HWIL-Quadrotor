# HWIL-Quadrotor

Hardware-in-the-loop Simulation for Quadrotor using ROS and Gazebo

## Install
ROS and Gazebo: http://wiki.ros.org/noetic/Installation/Ubuntu

Installing Dependencies
```
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-message-to-tf
sudo apt-get install ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers
```
Clone from repository
```
git clone --recurse-submodules https://github.com/NovelioPI/HWIL-Quadrotor.git
```
Compile ROS
```
cd HWIL-Quadrotor
catkin_make
source devel/setup.bash
```

## Launch simulator
```
roslaunch hwil_sim start.launch
```
