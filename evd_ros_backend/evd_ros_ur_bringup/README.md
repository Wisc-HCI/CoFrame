# evd_ros_ur_bringup
Expert View Dashboard (EvD) is an educational environment used to train operators
for collaborative robotic workcells.

This configuration can be used to connect the EvD core infrastructure to
either a physical UR robot or to a URSim application. The URSim may either be
ran locally, on a URSim VM, or on a separate dedicated machine. ROS in not needed
in either the VM or the separate machine. This configuration requires the ur robot
driver's URCap to be installed.

## URSim Installation
Regarding operating system, for Linux installs I have been using
[Lubuntu 16.04 LTS](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release/) which
runs well on older hardware. If you are using the non-linux route then the VM
is already a 32-bit Lubuntu 14.04 LTS instance.

Next if you are using the simulator then you will need to install URSim from
Universal Robots for e-series robots. Currently it can be found
[here](https://www.universal-robots.com/download/?option=53319#section41511).
I have used both the 5.4 and 5.3.1 variants. Specific version doesn't matter for
this application.

## ROS dependencies

- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [industrial_core](http://wiki.ros.org/industrial_core) via `sudo apt install ros-<VERSION>-industrial-core`
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [fmauch/universal_robot](https://github.com/fmauch/universal_robot)

## Execute

Via roslaunch, run the following,

```
roslaunch evd_ros_ur_bringup main.launch type:=<TYPE-STRING>
```

### Type
There are two types expected by evd_ros_core:
- physical : this connects to a real robot or the real robot's simulation.
- simulated : this is used for the interactive robot marker on the front-end interface.
