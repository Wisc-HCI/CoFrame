# evd_ros_core
Expert View Dashboard (EvD) is an educational environment used to train operators for collaborative robotic workcells.

This package is the ROS backend server that runs an implemented task. Consider this package to be a library to hook into when building your specific applications.

To see the system overview check out the repository [README](../../README.md).

## Core Architecture
The goal of the core package is to centralize Evd behavior between multiple tasks. If one is writing one-off code then it should not go here. Likewise, very low-level cobot control probably should not be implemented in here.

Please refer to the following diagram for the backend architecture.

<img src="https://github.com/Wisc-HCI/Expert_View_Dashboard/evd_ros_backend/evd_ros_core/_docs/_imgs/core_arch.png" width="200" />

The curx of Core is the data_server. This node maintains the current EvDscript state for the entire EvD environment. Access to this data_server is easily achieved with the data client interface. EvD also provides a trace processor and its asociated graders to convert trajectories and waypoint/locations into traces and joint states, respectively. EvD tracks issues and pending jobs, which can be routed into the frontend interface.

To view the ROS topics and services present in Core, refer to [this](./_docs/ROS_INTERFACING.md). To learn more about the underlying EvDScript implementation see [this](./_docs/EVD_SCRIPT.md).


## Future Work
There are several parts that should be addressed in the future but are not on the
critical path toward EvD.

First, EvD does not handle program verification. Implementing this could improve
/ finish the program quality frame feedback. Of course this also requires changes
to EvDscript.

Second, EvDscript needs to implement full control flow. That is branching and
conditional looping. There should also be an exception mechanism and sensor
mechanisms.

Third, EvD should handle grasp verification. This can improve the authenticity of
grasping simulation over the naive assumptions currently baked into EvD.
