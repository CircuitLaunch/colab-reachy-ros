# colab-reachy-ros

### ROS structure
This directory is a ROS package. If you want to use it in your own ROS environment, clone it into your `catkin_ws/src` directory. Alternatively, if you use the development Docker-Compose setup, it will automatically be mounted as `/home/reachy/catkin_ws/src/colab-reachy-ros`, and other packages associated with the Reachy will automatically be cloned.

### Native setup

- You must install ros noetic using the standard wiki procedure
- Then install other needed ROS packages with "sudo apt-get install -y ros-noetic-moveit ros-noetic-cv-camera"
- Then build your catikn_ws
- Navigate to the src folder in your catkin_ws, you'll need to clone three repos there:
- git clone https://github.com/pollen-robotics/reachy_description
- cd reachy_description
- git checkout -b ros1 a51b576
- cd ..
- git clone https://github.com/pollen-robotics/reachy_moveit_config
- git clone https://github.com/CircuitLaunch/colab_reachy_ros
- cd colab_reachy_ros
- pip3 install -r requirements.txt
- cd ..
- catkin_make
- source devel/setup.bash

### Docker setup
#### With VSCode
To set up the Docker development container using VSCode, simply install the Remote-Containers extension on VSCode, start Docker, open this directory in VSCode and click the "Reopen in Container" prompt when it appears. VSCode will handle building and opening the environment, and will also install relevant extensions for Python and ROS development into the container.

#### Without VSCode
To build the container yourself, enter the .devcontainer directory in your terminal and run the command `docker-compose up` to create and start the container.

#### Docker Ports

The development container holds the following ports for the following reasons:

* Port 6171: Used to connect a Python `Reachy` object to [Pollen's online Reachy simulator](http://playground.pollen-robotics.com).

#### Using simulator
- Open in your browser, and click connect once the unity window loads: https://playground.pollen-robotics.com/
- Run `roslaunch colab_reachy_ros moveit_demo.launch simulator:=true`
- Open a new terminal to use the arm compliant mode service, and enter `rosservice call /right_arm_controller/set_arm_compliant False`


# Node Descriptions

## NAME
### Screenshot
<!--<img src="assets/screnshot.png" width="400">-->
#### Description
```text
blah blah blah
```
##### <DESCRIPTION> 
#### Commandline Example
```bash
echo foo
```
#### Hardware Requirments
- foo
- bar 
##### <DESCRIPTION>
#### Message Format
#### [trajectory_msgs/JointTrajectoryPoint Documentation](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)
```cpp
  
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
```
#### Required Packages (python)
##### <DESCRIPTION>
```text
requests==3.5.2
```
#### Ros packages Instilation (apt-get install)
##### <DESCRIPTION>
```bash
ls -hal
```
#### Topics/Services/Actions Defintion 
##### [REQUIRED/NOREQUIRED] <DESCRIPTION>
```bash
/EXAMPLE_TOPIC : Int16
```
### Ros parameters
```bash
```
#### Notes
#####
