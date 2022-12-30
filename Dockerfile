FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

COPY python-requirements.txt python-requirements.txt

RUN apt update

### gzweb ###
# dependencies
RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
RUN apt-get install -y git curl python-dev
ENV NODE_VERSION=11.14.0
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
RUN node --version
RUN npm --version

# install
WORKDIR /root
RUN source .bashrc
RUN git clone https://github.com/osrf/gzweb
WORKDIR /root/gzweb
RUN source /usr/share/gazebo/setup.sh
RUN export GAZEBO_MODEL_PATH=/rb_ws/src/buggy/sim_models
RUN ./deploy.sh -m -c
WORKDIR /

### Buggy ###
# dependencies
RUN apt-get install -y python3-pip
RUN apt-get install -y x11-apps xauth
RUN apt-get install -y vim 
RUN apt-get install -y ros-noetic-rosserial ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
RUN pip install -r python-requirements.txt

# ROS Setup
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> /root/.bashrc
