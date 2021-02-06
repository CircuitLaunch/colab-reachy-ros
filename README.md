# colab-reachy-ros

### ROS structure
This directory is a ROS package. If you want to use it in your own ROS environment, clone it into your `catkin_ws/src` directory. Alternatively, if you use the development Docker-Compose setup, it will automatically be mounted as `/home/reachy/catkin_ws/src/colab-reachy-ros`, and other packages associated with the Reachy will automatically be cloned.
### Docker setup
#### With VSCode
To set up the Docker development container using VSCode, simply install the Remote-Containers extension on VSCode, start Docker, open this directory in VSCode and click the "Reopen in Container" prompt when it appears. VSCode will handle building and opening the environment, and will also install relevant extensions for Python and ROS development into the container.

#### Without VSCode
To build the container yourself, enter the .devcontainer directory in your terminal and run the command `docker-compose up` to create and start the container.

#### Docker Ports
The development container holds the following ports for the following reasons:

* Port 6171: Used to connect a Python `Reachy` object to [Pollen's online Reachy simulator](http://playground.pollen-robotics.com).