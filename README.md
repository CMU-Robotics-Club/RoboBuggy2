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

### ROS
- Navigate to the `rb_ws` workspace. This is where all ROS and Python programs will live.
- To build the ROS workspace and source it, run:

        catkin_make
        source devel/setup.bash


### Notes for Christian
/rb_ws/src/buggy is the package for our project
("catkin_create_pkg buggy std_msgs rospy roscpp geometry_msgs")

"rospack find buggy" will find our package

"rosrun buggy hello_world.py" will run the python scripts
(!!! Make sure hello_world.py is executable via "chomod +x hello_world.py")

Publisher: test_publisher.py
Subscriber: test_subscriber.py
Test Python File: hello_world.py

Published Messages is "Robobuggy > Atlas! PublishedTime: %f"