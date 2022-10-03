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
