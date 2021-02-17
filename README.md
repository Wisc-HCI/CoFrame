# Expert View Dashbaord
Expert View Dashboard (EvD) is an educational environment used to train operators
for collaborative robotic workcells.

We accomplish this with an authoring tool that incorporates a Blockly-based visual
programming language called EvD-Script, a simulation environment built in Unity,
and an expert checklist that directs the operator to think in four frames:
Safety, Program Quality, Performance, and Buisness Objectives.

## System Architecture
TODO




## Installation
Unfortuntly, EvD relies on many different components to get it working. Below,
installation is described for each subsystem.

### evd_authoring_app
We developed the authoring application using React and Fluent UI. All dependencies
can be installed by navigating into the directory and running:

```
npm install
```

### evd_ros_backend
First install ROS (one) either kinetic or melodic, preferrably full desktop install.

Then clone this repo into your catkin workspace. You will also need to
install the following packages.

- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [industrial_core](http://wiki.ros.org/industrial_core) via `sudo apt install ros-<VERSION>-industrial-core`
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [fmauch/universal_robot](https://github.com/fmauch/universal_robot)
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)

We also need to install the following python modules.

- [scipy](https://pypi.org/project/scipy/)

Lastly, we need to setup the robot simulators (and/or the physical robot). For this
project we targeted a UR3e from Universal Robots. We use the UR robot driver in
ROS (that you already installed) but it does not come with the simulators.

First, you will need to decide whether the simulator will run on hardware or in
a virtual machine. If you have a few physical machines available then you can
install the simulator on each one. Otherwise, the better option is to setup a
VM for each simulator. Note, you will need two/three simulators running.

Regardless, head over to Universal Robots' [website](https://www.universal-robots.com/download/) to download the simulator. Note, you will have to create an account to access the files.

- Pick `software`,`linux simulator`,`robot arm size`, `UR3e`, (`OFFLINE SIMULATOR - E-SERIES - UR SIM FOR LINUX 5.4.3`) if you want to run it directly.
- Pick `software`,`non-linux simulator`,`robot arm size`, `UR3e`, (`OFFLINE SIMULATOR - E-SERIES - UR SIM FOR NON LINUX 5.4.3`) if you want to run in a VM.

Last step is to update the launch files with your specific configuration. If setting up a fresh system it
might be easier to just configure your system is its described in the launch files.

`evd_ros_backend\evd_ros_core\launch\robot_bringup.launch`
- Change physical_ip_robot to your real robot.
- Change physical_ip_sim to your simulated physical robot.
- Change planner_ip to your simulated planner robot.
- Change simulated_ip to your simulated simulated robot.
- Change physical_port, physical_reverse_port, simulated_port, simulated_reverse_port, planner_port, and planner_reverse_port to unique values. (I recommend configuring your simulators/robots with the scheme already described).

`evd_ros_backend\evd_ros_ur_bringup\launch\main.launch`
- Needs to have ip_address changed (if used directly) to either sim or real robot.
- Potentially kinematics_config, port, and reverse_port changed.
- For robotiq driver, may need to adjust comport and baud.

*(Optional)*

`evd_ros_backend\evd_ros_ur_bringup\launch\test.launch`
- Needs to have ip_address changed to sim or real robot.
- Potentially kinematics_config, port, and reverse_port changed.

### evd_unity_sim_app
We developed the Unity application under Windows 10 with Unity 2019.4.5f1. All
dependencies are already included within the directory.



## Deployment

### evd_authoring_app
Just run the following command to load the development server, which should be
sufficient for your purposes.

```
npm start
```

### evd_ros_backend


### evd_unity_sim_app
Please don't run this code directly. There is a test build available but the best
experience is to use the version embedded within the authoring app.

## Future Development
This project is accepting issues, pull-requests, feedback, etc. Our current version
is still a far war away from the aims of this project so any input is appreciated.

For specific development guidence, checkout each subsystem's respective README.
- [evd_authoring_app README](./evd_authoring_app/README.md)
- [evd_ros_backend README](./evd_ros_backend/README.md)
- [evd_unity_sim_app README](./evd_unity_sim_app/README.md)
