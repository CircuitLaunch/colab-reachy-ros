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
