# Code Flow Document
## Scope
The purpose of this file is to outline the desired framework of the auton stack (and the simulator + logging capabilities) of the RD25 software on a file level. This document should be referenced when writing new files for this framework.

## Viewing On VSCode
Install `bierner.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## Chart!
```mermaid
flowchart LR
    Serial --> Sanity_Check

    subgraph Control Loop
        Controller --> Pathplanner
    end

    Simulator --> Pathplanner
    Controller --> Simulator




    Serial --> BuggyState_Republisher

    BuggyState_Republisher --> Pathplanner
    BuggyState_Republisher --> Controller



    Controller --> Serial

```