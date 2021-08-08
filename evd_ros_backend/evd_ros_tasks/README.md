# evd_ros_tasks
Expert View Dashboard (EvD) is an educational environment used to train operators
for collaborative robotic workcells.

This package provides tasks implemented for experiments and demos when using the EvD system.

To see the system overview check out the repository [README](../README.md).

## Tasks
The following tasks were implemented, click the link to get further documentation.

- [3D Printer Machine Tending](./tasks/3d_printer_machine_tending/README.md) ~ Interacts with a fake CNC machine and performs palletization.

## Task Format
All tasks must contain an `applications` directory. This is where serialized evd_script is stored.
Additionally, all `collision_meshes` and unique `models` are required to be stored in within the task
directory. Lastly, the application should have a URDF that specifies the links between the robot and
all fixtures / static objects within the world.

The `applications` directory should always contain a `meta.json` file. When initialized,
it should look like this:

```
{
    "description": "<Brief note about the task>",
    "name": "<Task Name>",
    "options": []
}
```

The options field will be filled in by the data server when applications are saved. Each option is an
application plus configuration that can be run by EvD.

Lastly, make sure to update the launch file `main.launch` with the task option and include a
custom launch file in the `launch\tasks` directory. The custom launch file should hook into
existing EvD ore infrastructure w/ custom configuration. If nodes external to EvD need to run,
then this is where you should launch them.

## Code
All code should follow standard ROS convention and be placed in either scripts or src depending on the 
generalizability of the thing written.

## Notes
This package ends up serving as a place to store one-off task specific code. As
such there is a fair amount of deprecated and half-baked code here.















### UR Controller stuff
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
