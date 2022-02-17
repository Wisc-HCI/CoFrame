export const urdf = `<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from src/universal_robot/ur_description/urdf/ur3.xacro | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="ur3_robot">
  <!--
    Base UR robot series xacro macro.

    NOTE: this is NOT a URDF. It cannot directly be loaded by consumers
    expecting a flattened '.urdf' file. See the top-level '.xacro' for that
    (but note: that .xacro must still be processed by the xacro command).

    For use in '.launch' files: use one of the 'load_urX.launch' convenience
    launch files.

    This file models the base kinematic chain of a UR robot, which then gets
    parameterised by various configuration files to convert it into a UR3(e),
    UR5(e), UR10(e) or UR16e.

    NOTE: the default kinematic parameters (ie: link lengths, frame locations,
    offets, etc) do not correspond to any particular robot. They are defaults
    only. There WILL be non-zero offsets between the Forward Kinematics results
    in TF (ie: robot_state_publisher) and the values reported by the Teach
    Pendant.

    For accurate (and robot-specific) transforms, the 'kinematics_parameters_file'
    parameter MUST point to a .yaml file containing the appropriate values for
    the targetted robot.

    If using the UniversalRobots/Universal_Robots_ROS_Driver, follow the steps
    described in the readme of that repository to extract the kinematic
    calibration from the controller and generate the required .yaml file.

    Main author of the migration to yaml configs: Ludovic Delval.

    Contributors to previous versions (in no particular order):

     - Felix Messmer
     - Kelsey Hawkins
     - Wim Meeussen
     - Shaun Edwards
     - Nadia Hammoudeh Garcia
     - Dave Hershberger
     - G. vd. Hoorn
     - Philip Long
     - Dave Coleman
     - Miguel Prada
     - Mathias Luedtke
     - Marcel Schnirring
     - Felix von Drigalski
     - Felix Exner
     - Jimmy Da Silva
     - Ajit Krisshna N L
     - Muhammad Asif Rana
  -->
  <!--
    NOTE: the macro defined in this file is NOT part of the public API of this
          package. Users CANNOT rely on this file being available, or stored in
          this location. Nor can they rely on the existence of the macro.
  -->
  <!-- links: main serial chain -->
  <link name="base_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0.0"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/base.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0.04"/>
      <geometry>
        <cylinder length="0.09" radius="0.065"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0030531654454" ixy="0.0" ixz="0.0" iyy="0.0030531654454" iyz="0.0" izz="0.005625"/>
    </inertial>
  </link>
  <link name="shoulder_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/shoulder.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.11" radius="0.065"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.008093163429399999" ixy="0.0" ixz="0.0" iyy="0.008093163429399999" iyz="0.0" izz="0.005625"/>
    </inertial>
  </link>
  <link name="upper_arm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.12"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/upperarm.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="-0.127 0 0.12"/>
      <geometry>
        <cylinder length="0.145" radius="0.05"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="0 0 0.125"/>
      <geometry>
        <cylinder length="0.11" radius="0.05"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="-0.245 0 0.125"/>
      <geometry>
        <cylinder length="0.11" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.42"/>
      <origin rpy="0 1.5707963267948966 0" xyz="-0.121825 0.0 0.12"/>
      <inertia ixx="0.021728483221103233" ixy="0.0" ixz="0.0" iyy="0.021728483221103233" iyz="0.0" izz="0.00961875"/>
    </inertial>
  </link>
  <link name="forearm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.027"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/forearm.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="-0.07 0 0.027"/>
      <geometry>
        <cylinder length="0.20" radius="0.04"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="-0.215 0 0.027"/>
      <geometry>
        <cylinder length="0.095" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.26"/>
      <origin rpy="0 1.5707963267948966 0" xyz="-0.1066 0.0 0.027"/>
      <inertia ixx="0.0065445675821719194" ixy="0.0" ixz="0.0" iyy="0.0065445675821719194" iyz="0.0" izz="0.00354375"/>
    </inertial>
  </link>
  <link name="wrist_1_link">
    <visual>
      <!-- TODO: Move this to a parameter -->
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.104"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist1.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.095" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.002084999166" ixy="0.0" ixz="0.0" iyy="0.002084999166" iyz="0.0" izz="0.00225"/>
    </inertial>
  </link>
  <link name="wrist_2_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.08535"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist2.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.008 0.01"/>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.002084999166" ixy="0.0" ixz="0.0" iyy="0.002084999166" iyz="0.0" izz="0.00225"/>
    </inertial>
  </link>
  <link name="wrist_3_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.01 -0.081"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist3.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 -0.01 -0.015"/>
      <geometry>
        <cylinder length="0.04" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.35"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.02"/>
      <inertia ixx="0.00013626661215999998" ixy="0.0" ixz="0.0" iyy="0.00013626661215999998" iyz="0.0" izz="0.0001792"/>
    </inertial>
  </link>
  <!-- joints: main serial chain -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="shoulder_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.1519"/>
    <axis xyz="0 0 1"/>
    <limit effort="56.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin rpy="1.570796327 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="56.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin rpy="0 0 0" xyz="-0.24365 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="28.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_1_link"/>
    <origin rpy="0 0 0" xyz="-0.21325 0 0.11235"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <origin rpy="1.570796327 0 0" xyz="0 -0.08535 -1.750557762378351e-11"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin rpy="1.570796326589793 3.141592653589793 3.141592653589793" xyz="0 0.0819 -1.679797079540562e-11"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <!-- ROS-Industrial 'base' frame: base_link to UR 'Base' Coordinates transform -->
  <link name="base"/>
  <joint name="base_link-base_fixed_joint" type="fixed">
    <!-- Note the rotation over Z of pi radians: as base_link is REP-103
           aligned (ie: has X+ forward, Y+ left and Z+ up), this is needed
           to correctly align 'base' with the 'Base' coordinate system of
           the UR controller.
      -->
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="base"/>
  </joint>
  <!-- ROS-Industrial 'flange' frame: attachment point for EEF models -->
  <link name="flange"/>
  <joint name="wrist_3-flange" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="flange"/>
    <origin rpy="0 -1.5707963267948966 -1.5707963267948966" xyz="0 0 0"/>
  </joint>
  <!-- ROS-Industrial 'tool0' frame: all-zeros tool frame -->
  <link name="tool0"/>
  <joint name="flange-tool0" type="fixed">
    <!-- default toolframe: X+ left, Y+ up, Z+ front -->
    <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/>
    <parent link="flange"/>
    <child link="tool0"/>
  </joint>
</robot>`