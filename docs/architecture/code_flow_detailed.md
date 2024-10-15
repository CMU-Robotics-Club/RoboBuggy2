# Code Flow Document
## Scope
The purpose of this file is to outline the desired framework of the auton stack (and the simulator + logging capabilities) of the RD25 software on a file level. This document should be referenced when writing new files for this framework.

The large boxes are ROSNodes. Each individual smaller box is a file. Data within ROSNodes are transferred via function calls, while data between node are transferred via publishers and subscribers.

## Viewing On VSCode
Install `bierne
r.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## Chart!


```mermaid
flowchart LR

    subgraph "ROS Topic - Raw State"
        self/raw_state
        other/raw_state
        self/gps
        self/encoder_speed
        self/stepper_steering
    end

    subgraph "ROS Topic - Clean State"
        self/state
        other/state
    end

    subgraph "ROS Topic - Trajectory"
        self/global_traj
        self/cur_traj
    end

    subgraph "ROS Topic - Output"
        self/steering
    end

    subgraph "ROS Topic - Debug"
        self/ins_topics 
        self/gnss1
        self/gnss2
        self/firmware_debug
        self/sanity
    end
    
    subgraph "Serial Node"
        ROS_serial_translator
        serial_port_parser
    end

    subgraph "UKF Node"
        UKF
    end

    subgraph "Buggy State Node"
        buggy_state_converter
    end

    subgraph "Debug Telem Node"
        telemetry
        watchdog
    end

    subgraph "Pathplanner Node - 10 Hz"
        initialize_trajectory
        path_planner
        trajectory_wrapper
    end

    subgraph "Controller Node - 100 Hz"
        controller
    end

    self/encoder_speed --> |"10 Hz"| UKF
    self/stepper_steering --> |"10 Hz"| UKF
    self/gps --> |"10 Hz"| UKF
    UKF --> |"100 Hz"| self/raw_state
    INS --> |"100 Hz"| self/raw_state
    INS --> |"100 Hz"| self/ins_topics
    INS --> |"10 Hz"| self/gnss1
    INS --> |"10 Hz"| self/gnss2

    ROS_serial_translator --> |"10 Hz - SC"| self/encoder_speed
    ROS_serial_translator --> |"10 Hz - SC"| self/stepper_steering
    ROS_serial_translator --> |"10 Hz - SC"| self/gps
    ROS_serial_translator --> |"10 Hz - SC"| other/raw_state
    ROS_serial_translator --> |"100 Hz - NAND"| self/raw_state
    ROS_serial_translator --> |"100 Hz"| self/firmware_debug
    self/steering --> |"100 Hz"| ROS_serial_translator

    other/raw_state --> |"10 Hz"| buggy_state_converter
    buggy_state_converter --> |"10 Hz"| other/state
    self/raw_state --> |"100 Hz"| buggy_state_converter
    buggy_state_converter --> |"100 Hz"| self/state

    initialize_trajectory --> |"once"| self/global_traj
    initialize_trajectory --> |"once"| self/cur_traj
    self/global_traj --> path_planner
    self/cur_traj --> path_planner
    self/state --> path_planner
    other/state --> path_planner
    path_planner --> self/cur_traj

    self/cur_traj --> controller
    self/state --> controller
    controller --> self/steering

    self/state --> telemetry
    other/state --> telemetry
    self/cur_traj --> telemetry
    watchdog --> self/sanity
    
```