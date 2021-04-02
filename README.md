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

-----------------------
# Node Descriptions

-----------------------
## Head Node
### Screenshot
<img src="https://i.imgur.com/QWHPGBd.jpg" width="400">
<img src="https://i.imgur.com/Ru0sPze.png" width="400">

#### Description
```text
this node controls the head of reachy its two servo motors which control the tilt and yaw of the head
TODO: add control of reachy's 2x antenna's (ears)
```

#### Commandline Example
##### this sends a animation for the head in this case its dummy data
##### but could be conerted to something that shakes the head back and forth
```bash
roslaunch colab_reachy_ros head.launch
rostopic pub /head/position_animator trajectory_msgs/JointTrajectory '{points:[{positions:[1.5,1.5]},{positions:[0.8,1.2]},{positions:[0.7,0.6]},{positions:[0.3,0.1]}]}'
# single position (move tilt and yaw) debug statements
rostopic pub /head/position_animator/debug_point trajectory_msgs/JointTrajectoryPoint '{positions: [0.78, -0.7853982]}'
rostopic pub /head/position_animator/debug_point trajectory_msgs/JointTrajectoryPoint '{positions: [0.0, 0.0]}'
rostopic pub /head/position_animator/debug_point trajectory_msgs/JointTrajectoryPoint '{positions: [0.0, 1.308997]}'
rostopic pub /head/position_animator/debug_point trajectory_msgs/JointTrajectoryPoint '{positions: [0.0, 2.007129]}'
# Animation example
rostopic pub /head/position_animator trajectory_msgs/JointTrajectory '{points:[{positions:[1.5,1.5]},{positions:[1.5,1.308997]},{positions:[1.5,1.5]},{positions:[1.5,1.308997]},{positions:[1.5,1.5]},]}'

rostopic pub /head/position_animator trajectory_msgs/JointTrajectory '{points:[{positions:[1.5,1.745329]},{positions:[1.5,1.5]},{positions:[1.5,1.745329]},{positions:[1.5,1.5]},{positions:[1.5,1.5]},]}'

```
#### how to use degree mode debug topic 
#### usecase : i just want to move the motors to one position for testing
#### NOTE: `the angles are in degrees not! radians`
```bash
rostopic pub /head/position_animator/debug_point_degrees trajectory_msgs/JointTrajectoryPoint '{positions: [0, 75]}'
```
#### Hardware Requirments
- 2 hobby servo motors
#### Message Format
#### [trajectory_msgs/JointTrajectory Documentation](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html)
#### [trajectory_msgs/JointTrajectoryPoint Documentation](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)
 
```python
  
Header header
string[] joint_names
JointTrajectoryPoint[] points <- im only using this one

example of a JointTrajectoryPoint

float64[] positions <- im only using this one
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

example if it was in json format 
{
    points:
        [
            { positions: [1.5, 1.5] },
            { positions: [0.8, 1.2] },
            { positions: [0.7, 0.6] },
            { positions: [0.3, 0.1] }
        ]
}
```
#### Required Packages (python)
##### im not using any special packages besides the normal rospython client
```text
rospy==1.15.9
```
#### Ros packages Instilation (apt-get install)
##### ros serial for noetic is required to communicat with the arduino over serial
```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
#### Topics Defintion 
##### <DESCRIPTION>
```bash
---public topics---
"/head/position_animator": JointTrajectory

---private topics ---
"/head/position_animator/debug_point": JointTrajectoryPoint used for debugging only
"/head/neck_pan_goal": UInt16
"/head/neck_tilt_goal": UInt16
```
#### Ros parameters
##### file is located in launch/head.launch
```bash
value="/dev/ttyACM0"/ must be changed to reflect the arduino your using

to find your arduino port run this command
1) 
ls /dev/ttyACM*
2) if that doesnt work try
ls /dev/serial/by-id
```
#### Notes
#### The arduino code has a manual offset of -90 to account for how the servo motors are mounted that can be changed to 0 if you dont have any offsets
### filename: `arduino_code/ServoControl/ServoControl.ino`
### ` int head_yaw_offset  = -90`
### to 
### `int head_yaw_offset  = 0;`
### title range is between 115 degrees (90+25d ) <->  75 degrees (90-15d ) with no offset
### ----- where 90 degrees is the head facing forward (virtical)
### how to create a head nodding animation
#### yes guesture
#### NOTE: (0, 90) is a nutral head position head facing straight forward and neck stright upright
### commandline pub version of yes guesture
```bash
94,90
94,115
94,90
94,115
94,90

rostopic pub /head/position_animator_debug_degrees trajectory_msgs/JointTrajectory '{points:[{positions:[94,90]},{positions:[94,115]},{positions:[94,90]},{positions:[94,115]},{positions:[94,90]}]}' 
#### no guesture
```bash
94,90
65,90
115,90
65,90
115,90
94,90

rostopic pub /head/position_animator_debug_degrees trajectory_msgs/JointTrajectory '{points:[{positions:[94,90]},{positions:[65,90]},{positions:[115,90]},{positions:[65,90]},{positions:[115,90]},{positions:[94,90]}]}'  
```
### Look right
```bash
94,80
65,80
65,80
94,80

rostopic pub /head/position_animator_debug_degrees trajectory_msgs/JointTrajectory '{points:[{positions:[94,80]},{positions:[65,80]},{positions:[65,80]},{positions:[94,80]}]}'  
```

### Look left
```bash
94, 80
115,80
115,80
94, 80

rostopic pub /head/position_animator_debug_degrees trajectory_msgs/JointTrajectory '{points:[{positions:[94,80]},{positions:[115,80]},{positions:[115,80]},{positions:[94,80]}]}'  
```



-----------------------
# EXAMPLE FORMAT
-----------------------
## NAME
### Screenshot
<!--<img src="assets/screnshot.png" width="400">-->
#### Description
```text
blah blah blah
```
#### Commandline Example
##### <DESCRIPTION> 
```bash
echo foo
```
#### Hardware Requirments
- foo
- bar 
##### <DESCRIPTION>
#### Message Format
#### [trajectory_msgs/JointTrajectoryPoint Documentation](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)
```python
  
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
##### <DESCRIPTION>
```bash
/EXAMPLE_TOPIC : Int16
```
#### Ros parameters
```bash
```
#### Notes
#####

-----------------
# Contributors

[![](https://contrib.rocks/image?repo=CircuitLaunch/colab_reachy_ros)](https://github.com/CircuitLaunch/colab_reachy_ros/graphs/contributors)

##### Made with [contributors-img](https://contrib.rocks).

-----------------
# License
#### MIT © wisehackermonkey
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
