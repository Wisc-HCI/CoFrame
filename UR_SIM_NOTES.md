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