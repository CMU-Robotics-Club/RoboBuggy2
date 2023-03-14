FROM osrf/ros:noetic-desktop-full-focal


RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
  python3-tk \
  vim git tmux tree sl htop x11-apps

RUN apt-get install -y -qq \
  ros-noetic-rosserial \
  ros-noetic-foxglove-bridge \
  ros-noetic-microstrain-inertial-driver \
  ros-noetic-realsense2-camera \
  ros-noetic-realsense2-description

COPY python-requirements.txt python-requirements.txt
RUN pip3 install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
RUN echo 'cd rb_ws' >> ~/.bashrc
RUN echo 'catkin_make >/dev/null' >> ~/.bashrc
RUN echo 'source devel/setup.bash' >> ~/.bashrc

# RUN echo "exec firefox" > ~/.xinitrc && chmod +x ~/.xinitrc
# CMD ["x11vnc", "-create", "-forever"]

# add mouse to tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf

