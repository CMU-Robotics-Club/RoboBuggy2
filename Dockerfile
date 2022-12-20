FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y python3-pip x11-apps xauth vim ros-noetic-rosserial ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
RUN pip install -r python-requirements.txt
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc