# cobots_ros
Cobots is an education system developed for collaborative manufacturing robots (e.g., Universal Robots, Franka Emika) with the goal of overlaying an expert's view of a task onto the operator's understanding.

These packages compose the ROS backend for the Cobots project.

- [cobots_ros](./cobots_ros/README.md): ROS meta-package
- [cobots_core](./cobots_core/README.md): ROS Server backend
- [cobots_tasks](./cobots_tasks/README.md): Specific tasks for experiments and demos
- [cobots_ur_bringup](./cobots_ur_bringup/README.md): Interface for UR robot

For the front-end implementation:

- [cobots_augmented_reality_ui](https://github.com/Wisc-HCI/cobots_augmented_reality_ui)
- [cobots_web_ui]

## Dependencies

### Backend
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
    - First try the package manager `sudo apt install ros-<VERSION>-rosauth`
  - Can try to install via `sudo apt install ros-<VERSION>-rosbridge-suite`
  - If cloning from Github, make sure to checkout `master` branch
  - I have `tornado==4.5.3`, it might work on a different version.
- [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [industrial_core](wiki.ros.org/industrial_core) via `sudo apt install ros-<VERSION>-industrial-core`
- [dwhit/ros-sharp](https://github.com/dwhit/ros-sharp)
- [MoriKen254/timed_roslaunch](https://github.com/MoriKen254/timed_roslaunch)
- [moveit](https://moveit.ros.org/) via `sudo apt install ros-<VERSION>-moveit`
  - Needed to for successful build, though we don't use it directly
- [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher) via 'sudo apt install ros-<VERSION>-joint-state-publisher-gui'
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [fmauch/universal_robot](https://github.com/fmauch/universal_robot)


## Not used right now (but probably will in the future)
- [ros-perception/ar_track_alvar](https://github.com/ros-perception/ar_track_alvar.git)
- [rosserial_arduio](http://wiki.ros.org/rosserial_arduino) via `sudo apt install ros-<VERSION>-rosserial-arduino`
