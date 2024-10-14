# Code Flow Document
## Scope
The purpose of this file is to outline the desired framework of the auton stack (and the simulator + logging capabilities) of the RD25 software on a file level. This document should be referenced when writing new files for this framework.

The large boxes are ROSNodes. Each individual smaller box is a file. Data within ROSNodes are transferred via function calls, while data between node are transferred via publishers and subscribers.

## Viewing On VSCode
Install `bierner.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## Chart!
```mermaid
flowchart LR

    subgraph I/O Translation Node
        Serial
        BuggyState_Converter
    end

    subgraph Controller Node
        Controller
    end

    subgraph Pathplanner Node
        JSON_to_Trajectory
        Pathplanner
        Trajectory
    end

    subgraph Telemetry Node
        Sanity_Check

    end

    INS --> |"ekf/*"|Serial
    BuggyState_Converter --> |"(unit conversions)"|Serial
    Serial --> |"all the data"| Sanity_Check

    Pathplanner --> |"traj/follow_path"| Controller
    Controller --> |"SC/state"| Pathplanner
    Simulator --> Pathplanner
    Controller --> Simulator
    Controller --> |"self/steering_angle"| Serial

    Serial --> |"NAND/state"| Pathplanner
    Serial --> |"self/state"| Controller




    JSON_to_Trajectory --> |"get_init_traj()"|Pathplanner

```


