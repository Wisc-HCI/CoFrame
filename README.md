# cobots_ros
Cobots is an education system developed for collaborative manufacturing robots (e.g., Universal Robots, Franka Emika) with the goal of overlaying an expert's view of a task onto the operator's understanding.

These packages compose the ROS backend for the Cobots project.

- [cobots_ros](./cobots_ros/README.md): ROS meta-package
- [cobots_core](./cobots_core/README.md): ROS Server backend
- [cobots_tasks](./cobots_tasks/README.md): Specific tasks for experiments and demos

For the front-end implementation:

- [cobots_augmented_reality_ui](https://github.com/Wisc-HCI/cobots_augmented_reality_ui)

## System
TODO

## Installation and Dependencies

- [Wisc-HCI/robot_behavior](https://github.com/Wisc-HCI/robot_behavior.git)
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations.git)
- [Wisc-HCI/ros_pop_button](https://github.com/Wisc-HCI/ros_pop_button.git)
- [Wisc-HCI/wiscutils](https://github.com/Wisc-HCI/wiscutils.git)
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
    - First try the package manager `sudo apt install ros-<version>-rosauth`
- [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
- [ros-perception/ar_track_alvar](https://github.com/ros-perception/ar_track_alvar.git)
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
  - Must use URScript Driver!
- [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
  - Need version with UR e-series support
- [industrial_core](wiki.ros.org/industrial_core)
- [dniewinski/ur_modern_driver](https://github.com/dniewinski/ur_modern_driver.git)
- [dwhit/ros-sharp](https://github.com/dwhit/ros-sharp.git)
  - Not needed in the ROS workspace
- [MoriKen254/timed_roslaunch](https://github.com/MoriKen254/timed_roslaunch.git)
- [rosserial_arduion](http://wiki.ros.org/rosserial_arduino)
  - Try package manager `sudo apt install ros-<version>-rosserial-arduino`
