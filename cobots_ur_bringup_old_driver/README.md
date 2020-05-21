# cobots_ur_bringup_old_driver

This version of bringup is going to be deprecated in favor of the UR robot driver.

This bringup configuration can be used to connect the ROS core infrastructure to
either a physical UR Robot or to a URSim application using the UR-Modern driver.

Deploy this launch configuration onto a URSim VM with ROS installed, onto a
dedicated URSim computer, or a remote UR controller computer.

## Installation
Regarding operating system, for Linux installs I have been using
[Lubuntu 16.04 LTS](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release/) which
runs well on older hardware. If you are using the non-linux route then the VM
is already a 32-bit Lubuntu 14.04 LTS instance.

Next if you are using the simulator then you will need to install URSim from
Universal Robots for e-series robots. Currently it can be found
[here](https://www.universal-robots.com/download/?option=53319#section41511).
I have used both the 5.4 and 5.3.1 variants. Specific version doesn't matter for
this application.

Then install ROS.

If not using the VM then install according to your specific OS's version. Otherwise,
install ROS Indigo on the VM. I used the desktop-install instead of the desktop-full-install.

Create a workspace called `catkin_ws` in your home directory. If you name it
something else, the provided script will not work. However you are free to change
it to suit your needs.

Next, clone the [cobots_ros](https://github.com/Wisc-HCI/cobots_ros) repository.

Followed by these dependencies:
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
- [industrial_core](http://wiki.ros.org/industrial_core) via `sudo apt install ros-<VERSION>-industrial-core`
- [AdmiralWall/ur_modern_driver](https://github.com/AdmiralWall/ur_modern_driver) forked from [ros-industrial/ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver) for 5.4 support
- [moveit](https://moveit.ros.org/) via `sudo apt install ros-<VERSION>-moveit`
- [controller_manager](http://wiki.ros.org/controller_manager) via `sudo apt install ros-<VERSION>-controller-manager`
- [roslibpy](https://pypi.org/project/roslibpy/) via `pip install roslibpy`

NOTE: Roslibpy requires an older version of the Twisted library. Install

## Execute
### Option 1
Copy the `run_ros_ur_simulator_subsystem.sh` script in the `scripts` directory to you desktop then once URSim is running, execute this script.

Make sure to update the `ROS_TYPE` variable before executing the script to your systems type.

### Option 2
Via roslaunch run the following,

```
roslaunch cobots_ur_bringup main.launch type:=<TYPE> local:=false
```

### Type
There are three types expected by cobots_core:
- physical : this connects to a real robot or the real robot's simulation.
- simulated : this is used for the interactive robot marker on the front-end interface.
- planner : this is used by cobot_core's planner subsystem for generating and testing trajectories.
