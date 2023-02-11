FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
<<<<<<< HEAD
  x11-apps \
  xauth \
  vim \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3 \
  ros-noetic-rosserial \
  ros-noetic-foxglove-bridge \
  ros-noetic-microstrain-inertial-driver
  
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
=======
  x11-apps xauth \
  vim git tmux \
  ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-rosserial ros-noetic-foxglove-bridge

RUN pip3 install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
>>>>>>> a5cd1d113ed66b8b027e01f3b50037afa0699934
