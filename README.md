## Unity Simulation Link

This ROS2 workspace interfaces with [mars_rover_unity_sim](https://github.com/Maxwell44772029/mars_rover_unity_sim), which simulates the rover and terrain in Unity.

This repo:

- Subscribes to Unity’s `/pointcloud`, `/tf`, and `/clock`
- Projects OctoMap into a 2D costmap
- Runs A* planner and publishes to `/cmd_vel`
