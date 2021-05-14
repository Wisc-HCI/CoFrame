# evd_ros_backend
EvD is an education system developed for collaborative manufacturing robots (e.g., Universal Robots, Franka Emika) with the goal of overlaying an expert's view of a task onto the operator's.
We aim to improve understanding of the task such that operators can make informed changes to their program.

These packages compose the ROS backend for the EvD project.

- [evd_ros_backend](./evd_ros_backend/README.md): ROS meta-package
- [evd_ros_core](./evd_ros_core/README.md): ROS Server backend
- [evd_ros_tasks](./evd_ros_tasks/README.md): Specific tasks for experiments and demos
- [evd_ros_ur_bringup](./evd_ros_ur_bringup/README.md): Interface for UR robot

System overview for both Frontend and Backend is [here](../README.md).

At this time only the UR3e is supported. Future work may extend this to other robots.

## Dependencies

Wisc-HCI Specific Dependencies
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)

General ROS Dependencies
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
    - Via `sudo apt-get install ros-noetic-rosbridge-server`
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
    - Please follow their installation instructions!

We also need to install the following python modules.

- [scipy](https://pypi.org/project/scipy/)

## Deploy
EvD's backend is broken into a library `evd_ros_core` and the user's application space `evd_ros_tasks`. 
To runthe backend, just launch the following with the specific task as an arguement.

```
roslaunch evd_ros_tasks main.launch task:="<TASK NAME>"
```

Or just skip the pretext and call the specific task launch file. For example,

```
roslaunch evd_ros_tasks 3d_printer_machine_tending.launch
```

Under the hood, this will hook into the core launch files and will launch task specific nodes.
