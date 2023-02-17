# RoboBuggy2
A complete re-write of the old RoboBuggy.

---
## Table of Contents
 - Installation
 - Quickstart
 - Development


---
## Installation
### Docker
- You will need [Docker](https://docs.docker.com/get-docker/) installed.

- Once installed, the Docker image can be built as follows from the repository's root directory:

        docker compose build --no-cache
        docker compose up

- Then in another terminal window, in order to access the running Docker's CLI, run:

        docker exec -it robobuggy2-main-1 bash

- Now you should be presented with a bash CLI as you're used to.

### ROS
- Navigate to `/rb_ws`. This is the catkin workspace where we will be doing all our ROS stuff.
- To build the ROS workspace and source it, run:
        
        catkin_make
        source /rb_ws/devel/setup.bash  # sets variables so that our package is visible to ROS commands

### Alternate Shortcut
- In the main directory, just run `./setup_dev.sh` or `./setup_prod.sh` as appropriate.
- Run `exec_docker` to get into Docker environment and setup all the aliases.
        

---
## Quickstart

### Simulation

Watch [this](https://youtu.be/kEL3-sF9TTE) video to get started with visualizing the simulation.


### Controls

Edit `rb_ws/src/buggy/scripts/controller.py`. Skeleton code for interacting with existing topics is provided. 

To launch the simulation against your controls code, run the following command: `roslaunch buggy main.launch simulation:=true`.

If you get an error about not finding package `buggy`, remember to run `source /rb_ws/devel/setup.bash`. 

---

## Development

TODO: 
 - pusher.py: apply simulated pushing force if buggy is in certain areas  
 - steering_mux.py: Decide where to listen for steering inputs given a state
   - dead-mans (lock right(?))
   - Teleop (Input from foxglove teleop panel)
   - Controls (auton)
 - instrument_*.py
   - INS, 
