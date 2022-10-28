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

        docker exec -it robobuggy2-main-1

- Now you should be presented with a bash CLI as you're used to. Then run `source ros_entrypoint.sh`.

### ROS
- Navigate to the `rb_ws` workspace. This is where all ROS and Python programs will live.
- To build the ROS workspace and source it, run:
<<<<<<< HEAD
=======

        catkin_make
        source devel/setup.bash
        
### Running Example Publisher and Subcriber Scripts
- Ensure you've sourced the `rb_ws` workspace.
- Make your workspace by running `catkin_make`.
- Run `roscore` in one terminal.
- Navigate to `rb_ws/src/buggy/scripts/` and run `test_publisher.py` and `test_subscriber.py` in two separate terminals. (Example command would be `python3 test_publisher.py`)
- Understand what's going on in the publisher terminal and the subscriber terminal.
- Open another terminal window and use `rostopic` (http://wiki.ros.org/rostopic) to check everything is working correctly.



### Notes for Christian
/rb_ws/src/buggy is the package for our project
("catkin_create_pkg buggy std_msgs rospy roscpp geometry_msgs")
>>>>>>> parent of 575e33e... Update README.md

        catkin_make
        source devel/setup.bash
