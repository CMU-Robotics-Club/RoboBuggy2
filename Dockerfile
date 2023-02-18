FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
  vim git tmux \
  ros-noetic-rosserial ros-noetic-foxglove-bridge ros-noetic-microstrain-inertial-driver

RUN pip3 install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
RUN echo 'set -g mouse 1' >> ~/.tmux.conf
RUN echo 'cd rb_ws' >> ~/.bashrc
RUN echo 'catkin_make >/dev/null' >> ~/.bashrc
RUN echo 'source devel/setup.bash' >> ~/.bashrc
