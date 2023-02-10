FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
  vim git tmux \
  ros-noetic-rosserial ros-noetic-foxglove-bridge ros-noetic-microstrain-inertial-driver

RUN pip3 install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc