# colab-reachy-ros

### ROS structure
This directory is a ROS package. If you want to use it in your own ROS environment, clone it into your `catkin_ws/src` directory. Alternatively, if you use the development Docker-Compose setup, it will automatically be mounted as `/home/reachy/catkin_ws/src/colab-reachy-ros`, and other packages associated with the Reachy will automatically be cloned.

### Native setup

- You must install ros noetic using the standard wiki procedure
- Then install moveit with "sudo apt-get install -y ros-noetic-moveit"
- Then build your catikn_ws
- Navigate to the src folder in your catkin_ws, you'll need to clone three repos there:
- git clone https://github.com/pollen-robotics/reachy_description
- cd reachy_descrpition
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
- Open in your browser, and click connect once the unitiy window loads: https://playground.pollen-robotics.com/
- Open launch/test.launch and change line 34 to 'ws'
- Run "colab_reachy_ros test.launch
- Open a new terminal to use the arm compliant mode service, and enter "rosservice call right_arm_controller/set_arm_compliant False"
