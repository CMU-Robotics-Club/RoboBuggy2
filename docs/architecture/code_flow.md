# Code Flow Document
## Scope
The purpose of this file is to outline the desired framework of the auton stack (and the simulator + logging capabilities) of the RD25 software on a file level. This document should be referenced when writing new files for this framework.

## Viewing On VSCode
Install `bierner.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## Chart!
```mermaid
flowchart LR

    subgraph I/O Translation Node
        Serial
        BuggyState_Republisher
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


    Serial --> BuggyState_Republisher
    BuggyState_Republisher --> Sanity_Check
    Controller --> Serial

    BuggyState_Republisher --> Pathplanner
    BuggyState_Republisher --> Controller


    Controller --> Pathplanner
    Simulator --> Pathplanner
    Controller --> Simulator


    JSON_to_Trajectory --> Pathplanner

```