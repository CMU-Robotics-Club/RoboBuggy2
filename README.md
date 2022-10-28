# RoboBuggy2
Clowns leading clowns. (TM)
A complete re-write of the old RoboBuggy.

## Installation
### Docker
- You will need Docker dependencies installed. Please install using the following link https://docs.docker.com/engine/install/. 

- Once installed, the Docker image can be built as follows from the repository's root directory:

        docker compose build --no-cache
        docker compose up

- Then in another terminal window, in order to access the running Docker's CLI, run:

        docker exec -it robobuggy2-main-1 bash

- Now you should be presented with a bash CLI as you're used to.

### Setting up X11 Fowarding
- On the host machine, run `xhost local:docker`.

### ROS
- Navigate to the `rb_ws` workspace. This is where all ROS and Python programs will live.
- To build the ROS workspace and source it, run:
        
        catkin_make
        source devel/setup.bash  
        
### Running Example Publisher and Subcriber Scripts
- Ensure you've sourced the `rb_ws` workspace.
- Make your workspace by running `catkin_make`.
- Run `roscore` in one terminal.
- Navigate to `rb_ws/src/buggy/scripts/` and run `test_publisher.py` and `test_subscriber.py` in two separate terminals. (Example command would be `python3 test_publisher.py`)
- Understand what's going on in the publisher terminal and the subscriber terminal.
- Open another terminal window and use `rostopic` (http://wiki.ros.org/rostopic) to check everything is working correctly.

### Simulator
- We're using the Robotis's TurtleBot for our simulator because I (Christian) don't want to create a URDF of the Buggy from our CAD model.

- Run the following to run Gazebo and the Command Server:

        export TURTLEBOT3_MODEL=burger
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

- Publishing messages to `/cmd_vel` as in `test_gazebo_publisher.py` will command the turtle bot to move.

### Notes about our ROS Workspace
/rb_ws/src/buggy is the package for our project
("catkin_create_pkg buggy std_msgs rospy roscpp geometry_msgs")

"rospack find buggy" will find our package

"rosrun buggy hello_world.py" will run the python scripts
(!!! Make sure hello_world.py is executable via "chomod +x hello_world.py")

Publisher: test_publisher.py
Subscriber: test_subscriber.py
Test Python File: hello_world.py

Published Messages is "Robobuggy > Atlas! PublishedTime: %f"
