# Beginner_tutorials
ROS Beginner tutorials as part of ENPM808x

## Overview
* This repo demos a simple publisher and subscriber. 
* References were used from http://wiki.ros.org/ROS/Tutorials. 
* This repo also demos the use of Ros service.
* On running demo.launch, after 7 seconds, the client(present in listener.cpp) 
   sends a request to the server to change the message in the publisher ( in talker.cpp).
   This changes the message being published which can be seen in terminal prints
* User can also change frequency of publish during launch.
* Note the use of 5 levels of logging as well. 
* Added a unit testing file as well
* Added a TF broadcaster
* Added rosbag recording

## rqt_console screenshot
![image](https://github.com/vishnuu95/beginner_tutorials/blob/Week9_HW/rqt_console.png)

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
git checkout main
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run Instruction

```
roslaunch beginner_tutorials demo.launch
```
The above will launch the 2 nodes and the talker will publish at 10hz default. 
```
roslaunch beginner_tutorials demo.launch pub_freq:=<input your frequency value here as an Integer>
```
* The above will launch the 2 nodes and the talker will publish at the input frequency chosen by you. Also, a rosbag is recorded in the bags folder. The duration of record can be specified in the launch file. 
* The above will also publish to /tf
* To check the published transform, do a simple
```
rostopic echo /tf
```
You should see the transform with parent as "world" and child as "talk"
```
cd ~/catkin_make
catkin_make run_tests_beginner_tutorials
```
The above will run unit tests to test the ros service that the talker has.