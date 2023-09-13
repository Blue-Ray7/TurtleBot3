# TurtleBot3
Create different robot applications using ROS framework. The robot kit used in this project is the Turtlebot3 burger.


This repository consists of a master branch and a basic_functionalities branch.

The basic_funtionalities branch contains the core functions and features that are necessary to a robot to perform its intended purpose.

For instance: * Simple publisher and subscriber
	      * Tf2_broadcaster; Tf2 refers to the Transform library which keeps track of coordinate frames 
	      * Tf2_listener
	      * Laserscan_listener
	      * Odometry_listener
	      *	Computer_vision: Processes images using OpenCV library 
	      * Teleoperation: Controls the robot using keyboard or joystick input


The master branch will include advanced project tasks using a mixture of the algorithms developed in the basic_functionalities branch.

Some of the tasks will be: 
	      * Vision-module: Scanning a map, detecting target objects and marking them
	      * Robot navigation: Autonomous driving using Path planning algorithms
	      * SLAM: Self localization and mapping 

Potential projects to be made in the future:
	      * Industrial Pick and Place robot 
              * Medical surgery on a 3D Human model 
	      * Ping-Pong Robot player

All results will be visualized in Rviz graphical interface and Gazebo simulation environment.
