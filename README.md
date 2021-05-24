# Expert View Dashbaord
Expert View Dashboard (EvD) is an educational environment used to train operators
for collaborative robotic workcells.

We accomplish this with an authoring tool that incorporates a Blockly-based visual
programming language called EvD-Script, a simulation environment built in Unity,
and an expert checklist that directs the operator to think in four frames:
Safety, Program Quality, Robot Performance, and Buisness Objectives.

## System Architecture
TODO


## Installation
EvD relies on many different components to get it working. Below,
installation is described for each subsystem.

### evd_authoring_app
We developed the authoring application using React and Fluent UI. All dependencies
can be installed by navigating into the `evd_authoring_app` directory and running:

```
npm install
```

### evd_ros_backend
First install ROS (one) noetic, preferrably full desktop install.

Then clone this repo into your catkin workspace's `src` directory. You will also need to
install the following packages.

Wisc-HCI Specific Dependencies
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)

General ROS Dependencies
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
    - Via `sudo apt-get install ros-noetic-rosbridge-server`
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
    - Please follow their installation instructions!
    - Refactoring seems to be occuring in that project and upstream dependencies, for instance, you may need to install [ur_msgs](http://wiki.ros.org/ur_msgs) separately.

Please also install the following python modules.

- [scipy](https://pypi.org/project/scipy/)

### evd_unity_sim_app
We developed the Unity application under Windows 10 with Unity 2019.4.5f1. All
dependencies are already included within the directory.

## Deployment

### evd_authoring_app
Just run the following command to load the development server, which should be
sufficient for your purposes. Make sure to run this within the `evd_authoring_app` directory.

```
npm start
```

### evd_ros_backend
EvD's backend is broken into a library `evd_ros_core` and the user's application space `evd_ros_tasks`. 
To runthe backend, just launch the following with the specific task as an arguement.

```
roslaunch evd_ros_tasks main.launch task:="<TASK NAME>"
```

Or just skip the pretext and call the specific task launch file. For example,

```
roslaunch evd_ros_tasks 3d_printer_machine_tending.launch
```

Under the hood, this will hook into the core launch files and will also launch task specific nodes.

### evd_unity_sim_app
Please don't run this code directly. There is a test build available but the best
experience is to use the version embedded within the authoring app.

## Future Development
This project is accepting issues, pull-requests, feedback, etc. Our current version
is still far away from the aims of this project so any input is appreciated.

For specific development guidence, checkout each subsystem's respective README.
- [evd_authoring_app README](./evd_authoring_app/README.md)
- [evd_ros_backend README](./evd_ros_backend/README.md)
- [evd_unity_sim_app README](./evd_unity_sim_app/README.md)
