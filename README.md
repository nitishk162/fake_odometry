
## Overview

This package generates fake odometery and a ground truth pose that can be used in conjunction with scan_from_image package,
amcl and map_server to simulate a robot motion on a map.pgm. This is done through the fake_amcl.launch.
The pre-requistes packages to be already built to run the fake_amcl.launch file are:
scan_from_image
key_teleop
tf2_ros
amcl
map_server

Instructions to build scan_from_image and key_teleop are mentioned below. Install map_server and amcl from [here](https://github.com/ros-planning/navigation/tree/melodic-devel)
The PACKAGE NAME package has been tested under [ROS] Kinetic and Melodic on respectively Ubuntu 16.04, and 18.04.


## Installation

### Installing dependencies
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

	sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/nitishk162/fake_odometry.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

Similary install the other packages: [scan_from_image](https://github.com/nitishk162/scan_from_image) and [key_teleop](https://github.com/nitishk162/teleop_tools)

### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:melodic bash
	
This downloads the `ros:melodic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/nitishk162/fake_odometry.git
	git clone https://github.com/nitishk162/scan_from_image.git
	git clone https://github.com/nitishk162/teleop_tools

	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch fake_odometry fake_amcl.launch


Run the main node with

	roslaunch fake_odometry fake_amcl.launch
It will automatically open up rviz with pre-configured settings. Publish a 2D pose from rviz, it will call a service and set the initial pose for the simulation and then from the window the launch file was run, move the robot.
