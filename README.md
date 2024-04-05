# ROS2 Actions

This repository demonstrates the usages of ROS2 actions.

I used move_base as example because it is a very popular package in ROS.
However, it is no longer available in ROS2, hence, we would not be getting any
package naming conflicts as we learn.

## Action Definition

```bash
#goal definition
geometry_msgs/Pose goal
---
#result definition
int32 result_code
---
#feedback definition
float64 progress
```

## CLI Action Client

```bash
# Terminal 1 (start move base action server)
ros2 run move_base move_base_action_server_node
```

```bash
# Terminal 2 (send goal to move base action server)
# Add --feedback to retrieve progress feedback from action server
ros2 action send_goal /move_base move_base_msgs/action/MoveBase "goal:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"  --feedback
# Alternatively,
ros2 action send_goal /move_base move_base_msgs/action/MoveBase "{}" --feedback
```
