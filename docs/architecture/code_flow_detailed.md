# Code Flow Document
## Scope
The purpose of this file is to outline the desired framework of the auton stack (and the simulator + logging capabilities) of the RD25 software on a file level. This document should be referenced when writing new files for this framework.

## Viewing On VSCode
Install `bierne
r.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## Legend
- Green Boxes are marking ros topic gropus, labeled at the top
- Orange boxes are marking ros nodes and their functions
- Hexagonal black boxes are marking the ros topics that the ros nodes subscribe and publish to 
- Rectangular black boxes are marking individual files within the ros node and what topics they interact with

## Chart


```mermaid
flowchart TB

    subgraph RT-RS["ROS Topic - Raw State"]
        self/raw_state{{"self/raw_state"}}
        other/raw_state{{"other/raw_state"}}
        self/gps{{"self/gps"}}
        self/encoder_speed{{"self/encoder_speed"}}
        self/stepper_steering{{"self/stepper_steering"}}
    end

    subgraph RT-CS["ROS Topic - Clean State"]
        self/state{{"self/state"}}
        other/state{{"other/state"}}
    end

    subgraph RT-TRAJ["ROS Topic - Trajectory"]
        self/global_traj{{"self/global_traj"}}
        self/cur_traj{{"self/cur_traj"}}
    end

    subgraph RT-OUT["ROS Topic - Output"]
        self/steering{{"self/steering"}}
    end

    subgraph RT-DEB["ROS Topic - Debug"]
        self/ins_topics{{"self/ins_topics "}}
        self/gnss1{{"self/gnss1"}}
        self/gnss2{{"self/gnss2"}}
        self/firmware_debug{{"self/firmware_debug"}}
        self/sanity{{"self/sanity"}}
    end
    
    subgraph RN-SER["Serial Node"]
        ROS_serial_translator
        serial_port_parser
    end

    subgraph RN-UKF["UKF Node"]
        UKF - SC ONLY
    end

    subgraph RN-STATE["Buggy State Node"]
        buggy_state_converter
    end

    subgraph RN-DEB["Debug Telem Node"]
        telemetry
        watchdog
    end

    subgraph RN-PATH["Pathplanner Node - 10 Hz"]
        initialize_trajectory
        path_planner
        trajectory_wrapper
    end

    subgraph RN-CTRL["Controller Node - 100 Hz"]
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
    INS --> |"10 Hz - SC"| self/gps

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

    ROS_serial_translator --> |"10 Hz - SC"| self/encoder_speed
    ROS_serial_translator --> |"10 Hz - SC"| self/stepper_steering
    ROS_serial_translator --> |"10 Hz - SC"| other/raw_state
    ROS_serial_translator --> |"100 Hz - NAND"| self/raw_state
    ROS_serial_translator --> |"100 Hz"| self/firmware_debug
    self/steering --> |"100 Hz"| ROS_serial_translator

    RT-RS:::RT
    RT-CS:::RT
    RT-TRAJ:::RT
    RT-OUT:::RT
    RT-DEB:::RT

    RN-SER:::RN
    RN-UKF:::RN
    RN-STATE:::RN
    RN-DEB:::RN
    RN-PATH:::RN
    RN-CTRL:::RN

    classDef RT color:#000,fill:#74a57f;
    classDef RN color:#000,fill:#ee5622;
    linkStyle default stroke-width:3px;
    
```