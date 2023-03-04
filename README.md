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
        docker compose --env-file .env.dev up

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
## 3D Simulation Quickstart

Watch [this](https://youtu.be/kEL3-sF9TTE) video to get started with visualizing the simulation.


### Controls

Edit `rb_ws/src/buggy/scripts/controller.py`. Skeleton code for interacting with existing topics is provided. 

To launch the simulation against your controls code, run the following command: `roslaunch buggy main.launch simulation:=true`.

If you get an error about not finding package `buggy`, remember to run `source /rb_ws/devel/setup.bash`. 

---
## 2D Simulation Quickstart
Examples (from the same run):
- Foxglove playback of simulated file example: [link](https://youtu.be/dpa5oH69eJI)
- Matplotlib live simulation example: [link](https://youtu.be/6Xji-FtDQfo)

Control Example:
- Foxglove output (via sending steering + velocity commands): [link](https://youtu.be/AOsecwWmqyw)


- Install the appropriate X11 server on your computer for your respective operating systems (Xming for Windows, XQuartz for Mac, etc.).
- Mac: In XQuartz settings, ensure that the "Allow connections from network clients" under "Security" is checked.
- Windows: Make sure that you're using WSL 2 Ubuntu and NOT command prompt.
- While in a bash shell with the X11 server running, run `xhost +localhost`.
- Boot up the docker container using the "Alternate Shortcut" above.
- Run `xeyes` while INSIDE the Docker container to test X11 forwarding. If this works, we're good.
- Run `roslaunch buggy sim_2d.launch` for the simulator.

### Using the Simulator
Feedback:
- Longitude + Latitude for Foxglove visualization on map: `/sim_2d/navsatfix` (sensor_msgs/NavSatFix)
- UTM coordinates (assume we're in Zone 17T): `/sim_2d/utm` (geometry_msgs/Pose - position.x = Easting meters , position.y = Northing meters, position.z = heading in degrees from East axis + is CCW)

Commands:
- Steering angle: `/sim_2d/steering` in degrees (std_msgs/Float32)
- Velocity: `/sim_2d/velocity` in m/s (std_msgs/Float32)

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
