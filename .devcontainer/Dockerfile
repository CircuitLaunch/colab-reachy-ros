FROM ros:noetic-ros-base-focal
# FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y git python3-pip

RUN groupadd reachy
RUN useradd --create-home --shell /bin/bash reachy --gid reachy
RUN adduser reachy sudo
RUN printf "reachy ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

USER reachy
WORKDIR /home/reachy
SHELL ["/bin/bash", "-c"]

# RUN cd /home/reachy/colab-reachy-ros && /opt/ros/noetic/bin/catkin_make

RUN printf "\\nsource /home/reachy/colab-reachy-ros/devel/setup.bash\\n" >> /home/reachy/.bashrc

COPY --chown=reachy:reachy requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt
RUN rm /tmp/requirements.txt