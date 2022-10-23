# HWIL-Quadrotor

Hardware-in-the-loop Simulation for Quadrotor using ROS and Gazebo

## Install
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
roslaunch hwil_sim start_env.launch
```
