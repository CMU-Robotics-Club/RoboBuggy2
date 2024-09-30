# FROM nvidia/cuda:11.6.2-base-ubuntu20.04 as CUDA

#FROM arm64v8/ros:noetic-perception-focal
#FROM ros:noetic-ros-base-focal
# COPY --from=CUDA /usr/local/cuda /usr/local/
#FROM arm64v8/ros:noetic-ros-base-focal


FROM ubuntu:focal@sha256:420b6f4cc783dc199667eae3316d9e066bd6e19931448c1e4b6f9a3759e52106

ENV DEBIAN_FRONTEND noninteractive
RUN apt update
RUN apt-get install -y -qq \
  python3-pip \
  python3-tk \
  vim git tmux tree sl htop x11-apps curl

RUN apt-get update && apt-get install -y lsb-release && apt-get clean all

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
# RUN apt-key add -

# https://wiki.ros.org/noetic/Installation/Ubuntu

# RUN apt-get install -y -qq \
#   ros-noetic-desktop-full \ 
  ros-noetic-rosserial \
  ros-noetic-foxglove-bridge \
  ros-noetic-microstrain-inertial-driver \
  ros-noetic-realsense2-camera \
  ros-noetic-realsense2-description \
  ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs

# Run this now to cache it separately from other requirements
# COPY cuda-requirements_TEMP_DO_NOT_EDIT.txt cuda-requirements.txt
# RUN pip3 install -r cuda-requirements.txt

# COPY ./python-requirements_TEMP_DO_NOT_EDIT.txt python-requirements.txt
# RUN pip3 install -r python-requirements.txt

# RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
RUN echo 'cd rb_ws' >> ~/.bashrc
RUN echo 'catkin_make >/dev/null' >> ~/.bashrc
RUN echo 'source devel/setup.bash' >> ~/.bashrc

# shoestring fix since installing numba from pythonrequirements 
# smh breaks
RUN pip3 install numba

# RUN echo "exec firefox" > ~/.xinitrc && chmod +x ~/.xinitrc
# CMD ["x11vnc", "-create", "-forever"]

# add mouse to tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf
