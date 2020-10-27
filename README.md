# Beginner_tutorials
ROS Beginner tutorials as part of ENPM808x

## Overview
* This repo demos a simple publisher and subscriber. 
* References were used from http://wiki.ros.org/ROS/Tutorials. 
* The only delta is a change in string message at the publisher. 

## Build Requirements
* CMake
* catkin
* ROS - Melodic
* Ubuntu 18.04

## Build Instruction
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/vishnuu95/beginner_tutorials.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run Instruction
```
roscore
```
In a new terminal
```
rosrun beginner_tutorials talker
```
In a new terminal 
```
rosrun beginner_tutorials listener
```
