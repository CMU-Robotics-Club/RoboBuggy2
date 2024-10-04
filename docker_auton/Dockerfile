# FROM nvidia/cuda:11.6.2-base-ubuntu20.04 as CUDA

#FROM arm64v8/ros:noetic-perception-focal
#FROM ros:noetic-ros-base-focal
# COPY --from=CUDA /usr/local/cuda /usr/local/
#FROM arm64v8/ros:noetic-ros-base-focal


FROM --platform=linux/aarch64 ubuntu:focal

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y \
  vim git tmux tree sl htop x11-apps curl lsb-release apt-utils && \
  apt-get install -y python3-pip python3-tk && \
  apt-get clean all -y

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt-get update && \
	apt-get install -y \
		ros-noetic-desktop-full\
		ros-noetic-rosserial \
		ros-noetic-foxglove-bridge \
		ros-noetic-microstrain-inertial-driver \
		ros-noetic-realsense2-camera \
		ros-noetic-realsense2-description \
		ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
		

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
#   RUN apt-key add -


# RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
RUN 	echo 'source "/opt/ros/noetic/setup.bash" --' >> ~/.bashrc && \
	echo 'cd rb_ws' >> ~/.bashrc && \
	echo 'pip3 install -r python-requirements.txt' >> ~/.bashrc && \
	echo 'catkin_make >/dev/null' >> ~/.bashrc && \
	echo 'source devel/setup.bash' >> ~/.bashrc

# shoestring fix since installing numba from pythonrequirements 
# smh breaks
RUN pip3 install numba

# RUN echo "exec firefox" > ~/.xinitrc && chmod +x ~/.xinitrc
# CMD ["x11vnc", "-create", "-forever"]

# add mouse to tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf
