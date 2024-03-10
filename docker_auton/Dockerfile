FROM nvidia/cuda:11.6.2-base-ubuntu20.04 as CUDA

FROM osrf/ros:noetic-desktop-full-focal

COPY --from=CUDA /usr/local/cuda /usr/local/


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

# Run this now to cache it separately from other requirements
COPY cuda-requirements_TEMP_DO_NOT_EDIT.txt cuda-requirements.txt
# RUN pip3 install -r cuda-requirements.txt


COPY python-requirements_TEMP_DO_NOT_EDIT.txt python-requirements.txt
RUN pip3 install -r python-requirements.txt

RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> ~/.bashrc
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
