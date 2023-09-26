
<!-- PROJECT SHIELDS -->
[![LinkedIn](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/raouf-ayadi-a0a142223/)


<!-- ABOUT THE PROJECT -->
## About The Project

This repository was made to Create different robot applications using ROS framework. It consists of a master branch and a basic_functionalities branch.

The basic_funtionalities branch contains the core functions and features that are necessary for a robot to perform its intended purpose.

For instance: 
* Simple publisher and subscriber
* Tf2_broadcaster (Tf2 refers to the transform library which keeps track of coordinate frames) 
* Tf2_listener
* Laserscan_listener
* Odometry_listener
* Computer_vision: Processes images using OpenCV library 


The master branch will include advanced project tasks using a mixture of the algorithms developed in the basic_functionalities branch.

The main task will be creating a firefighter assistance robot that alerts the rescue team of the exact location of people inside a burning building by marking them on a map and displaying them on the rescue team's tablets.

This consists of:
* Vision-module: Scanning a map, detecting target objects and marking them
* Robot navigation module: Autonomous driving using Path planning algorithms
* SLAM module: Self localization and mapping 


All results will be visualized in Rviz graphical interface and Gazebo simulation environment.
