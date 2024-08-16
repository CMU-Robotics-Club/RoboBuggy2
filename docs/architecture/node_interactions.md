# ROS Node Interaction Document
## Scope
The purpose of this file is to descirbe the architecture of the RD25 stack from a ROS node level. This document should be updated whenever any ROS topics are added, removed or modified.

## Viewing On VSCode
Install `bierner.markdown-mermaid` from the extension marketplace to render the charts in VSCode markdown preview.

## List of topics
| Topic Name | Type                    | is custom message  |
| ---------- | ----------------------- | ------------------ |
| ego/state  | BuggyState              | yes |
| other_buggy/state | BuggyState       | yes |
| ego/trajectory | BuggyTrajectory     | yes |
| ego/steering_cmd | [Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) | no |
| add new topic here | | |

## ROS Nodes Graph
Auton Loop [Sequence Diagram](https://en.wikipedia.org/wiki/Sequence_diagram#:~:text=A%20sequence%20diagram%20shows%2C%20as,order%20in%20which%20they%20occur.)

```mermaid
sequenceDiagram
A->>B: "some/topic"
```

means A publishes a message of `some/topic` and B receives the message.

```mermaid
sequenceDiagram
    participant I as Ego State Estimator Node
    participant P as Planner Node
    participant C as Controller Node
    participant S as Firmware Comms Node

    box Grey HBK 3DM-GQ7 INS
        participant I
    end

    box Purple Auton Core
        participant P
        participant C
    end

    box Blue Communication with Sensors, Actuators and Other Buggy
        participant S
    end

    loop 100hz
        I->>P: "ego/state"
        I->>C: "ego/state"
    end

    loop 10hz
        S->>P: "other_buggy/state"
    end

    loop 10hz
        P->>C: "ego/trajectory"
    end

    loop 100hz
        C->>S: "ego/steering_cmd"
    end
```

