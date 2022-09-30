FROM osrf/ros:noetic-desktop-full-focal

COPY python-requirements.txt python-requirements.txt

RUN apt update
RUN apt-get install -y python3-pip
RUN pip install -r python-requirements.txt
RUN source ros_entrypoint.sh