FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
  vim git tmux tree sl htop

RUN apt-get install -y -qq \
  ros-noetic-rosserial \
  ros-noetic-foxglove-bridge \
  ros-noetic-microstrain-inertial-driver \
  ros-noetic-realsense2-camera

RUN pip3 install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
RUN echo 'cd rb_ws' >> ~/.bashrc
RUN echo 'catkin_make >/dev/null' >> ~/.bashrc
RUN echo 'source devel/setup.bash' >> ~/.bashrc

# add mouse to tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf
