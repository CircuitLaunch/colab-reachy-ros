FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y git git-lfs python3-pip
RUN apt-get install -y ros-noetic-moveit

RUN groupadd reachy
RUN useradd --create-home --shell /bin/bash reachy --gid reachy
RUN adduser reachy sudo
RUN printf "reachy ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

USER reachy
WORKDIR /home/reachy
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /home/reachy/catkin_ws/src
RUN cd /home/reachy/catkin_ws/src && git clone https://github.com/pollen-robotics/reachy_description
RUN cd /home/reachy/catkin_ws/src && git clone https://github.com/pollen-robotics/reachy_moveit_config
RUN cd /home/reachy/catkin_ws && source /opt/ros/noetic/setup.bash && /opt/ros/noetic/bin/catkin_make

RUN printf "\\nsource /home/reachy/catkin_ws/devel/setup.bash\\n" >> /home/reachy/.bashrc

COPY --chown=reachy:reachy requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt
RUN rm /tmp/requirements.txt