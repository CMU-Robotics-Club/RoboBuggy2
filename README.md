# RoboBuggy2
A complete re-write of the old RoboBuggy.

---
## Table of Contents
 - Installation
 - Quickstart
 - Development


---
## Installation (for Windows)
### Install Softwares: WSL, Ubuntu, Foxglove
- Go to Microsoft Store to install "Windows Subsystem for Linux" and "Ubuntu 20.04.6 LTS".
- Go install Foxglove https://foxglove.dev/.

### Docker
- You will need [Docker](https://docs.docker.com/get-docker/) installed.

### Set up repo in WSL
- To set up ssh key, follow this link: https://docs.github.com/en/authentication/connecting-to-github-with-ssh.
- In the website above, see these two pages: "Generating a new SSH key and adding it to the ssh-agent" and "Adding a new SSH key to your GitHub account".

### Clone
- In your terminal type: `$ git clone git@github.com:CMU-Robotics-Club/RoboBuggy.git`.
- The password above is find in github: code -> local -> Clone SSH.

### ROS
- Navigate to `/rb_ws`. This is the catkin workspace where we will be doing all our ROS stuff.
- To build the ROS workspace and source it, run:
        
        catkin_make
        source /rb_ws/devel/setup.bash  # sets variables so that our package is visible to ROS commands
- To learn ROS on your own, follow the guide on https://wiki.ros.org/ROS/Tutorials. Start from the first and install Ros on your own computer instead of using the Docker.

---
## Open Docker 
- Open WSL.
- Type `cd` to go to the WSL's default directory.
- Type `cd Robobuggy2`.
- Then do `explorer.exe .` to open the file explorer to the /Robobuggy2 directory. 
- Then do `./setup_dev.sh` in the main directory (RoboBuggy2) to launch the docker container.
- Then you can go in the docker container using the `docker exec -it robobuggy2-main-1 bash`.

  ## 2D Simulation
- To run 2D simulation on Foxglove, use `$ roslaunch buggy sim_2d.launch`.
- Open Foxglove, choose the third one to start link.
- On the top, click Layout, then "Import from file".
- Go to RoboBuggy2 and choose the file "telematics layout".
        

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

Instructions:
- Install the appropriate X11 server on your computer for your respective operating systems (Xming for Windows, XQuartz for Mac, etc.).
- Mac: In XQuartz settings, ensure that the "Allow connections from network clients" under "Security" is checked.
- Windows: Make sure that you're using WSL 2 Ubuntu and NOT command prompt.
- While in a bash shell with the X11 server running, run `xhost +local:docker`.
- Boot up the docker container using the "Alternate Shortcut" above.
- Run `xeyes` while INSIDE the Docker container to test X11 forwarding. If this works, we're good.
- Run `roslaunch buggy sim_2d.launch controller:=CONTROLLER_TYPE` for the simulator.

### Using the Simulator
Feedback:
- Longitude + Latitude for Foxglove visualization on map: `/state/pose_navsat` (sensor_msgs/NavSatFix)
- UTM coordinates (assume we're in Zone 17T): `/sim_2d/utm` (geometry_msgs/Pose - position.x = Easting meters , position.y = Northing meters, position.z = heading in degrees from East axis + is CCW)
- INS Simulation: `/nav/odom` (nsg_msgs/Odometry) (**Noise** is implemented to vary ~1cm)

Commands:
- Steering angle: `/buggy/steering` in degrees (std_msgs/Float64)
- Velocity: `/buggy/velocity` in m/s (std_msgs/Float64)

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
