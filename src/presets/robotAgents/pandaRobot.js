export const PANDA_ROBOT_DATA = {
  "robot-agent-3290720sfd3950234907450129sfcwesd2": {
    type: "robotAgentType",
    id: "robot-agent-3290720sfd3950234907450129sfcwesd2",
    name: "Panda Robot Agent",
    canDelete: true,
    canEdit: true,
    properties: {
      description:
        "A robot created by Franka Emika. This robot has a payload of 3kg.",
      initialJointState: {
        panda_joint1: 0,
        panda_joint2: 0,
        panda_joint3: 0,
        panda_joint4: 0,
        panda_joint5: 0,
        panda_joint6: 0,
        panda_joint7: 0,
      },
      jointLinkMap: [],
      pinchPointPairLinks: [
        {
          link1: "panda_link1",
          link2: "panda_link2",
        },
        {
          link1: "panda_link1",
          link2: "panda_link3",
        },
        {
          link1: "panda_link1",
          link2: "panda_link4",
        },
        {
          link1: "panda_link1",
          link2: "panda_link5",
        },
        {
          link1: "panda_link1",
          link2: "panda_link6",
        },
        {
          link1: "panda_link1",
          link2: "panda_link7",
        },
        {
          link1: "panda_link1",
          link2: "panda_link8",
        },
        {
          link1: "panda_link2",
          link2: "panda_link1",
        },
        {
          link1: "panda_link2",
          link2: "panda_link3",
        },
        {
          link1: "panda_link2",
          link2: "panda_link4",
        },
        {
          link1: "panda_link2",
          link2: "panda_link5",
        },
        {
          link1: "panda_link2",
          link2: "panda_link6",
        },
        {
          link1: "panda_link2",
          link2: "panda_link7",
        },
        {
          link1: "panda_link2",
          link2: "panda_link8",
        },
        {
          link1: "panda_link3",
          link2: "panda_link1",
        },
        {
          link1: "panda_link3",
          link2: "panda_link2",
        },
        {
          link1: "panda_link3",
          link2: "panda_link4",
        },
        {
          link1: "panda_link3",
          link2: "panda_link5",
        },
        {
          link1: "panda_link3",
          link2: "panda_link6",
        },
        {
          link1: "panda_link3",
          link2: "panda_link7",
        },
        {
          link1: "panda_link3",
          link2: "panda_link8",
        },
        {
          link1: "panda_link4",
          link2: "panda_link1",
        },
        {
          link1: "panda_link4",
          link2: "panda_link2",
        },
        {
          link1: "panda_link4",
          link2: "panda_link3",
        },
        {
          link1: "panda_link4",
          link2: "panda_link5",
        },
        {
          link1: "panda_link4",
          link2: "panda_link6",
        },
        {
          link1: "panda_link4",
          link2: "panda_link7",
        },
        {
          link1: "panda_link4",
          link2: "panda_link8",
        },
        {
          link1: "panda_link5",
          link2: "panda_link1",
        },
        {
          link1: "panda_link5",
          link2: "panda_link2",
        },
        {
          link1: "panda_link5",
          link2: "panda_link3",
        },
        {
          link1: "panda_link5",
          link2: "panda_link4",
        },
        {
          link1: "panda_link5",
          link2: "panda_link6",
        },
        {
          link1: "panda_link5",
          link2: "panda_link7",
        },
        {
          link1: "panda_link5",
          link2: "panda_link8",
        },
        {
          link1: "panda_link6",
          link2: "panda_link1",
        },
        {
          link1: "panda_link6",
          link2: "panda_link2",
        },
        {
          link1: "panda_link6",
          link2: "panda_link3",
        },
        {
          link1: "panda_link6",
          link2: "panda_link4",
        },
        {
          link1: "panda_link6",
          link2: "panda_link5",
        },
        {
          link1: "panda_link6",
          link2: "panda_link7",
        },
        {
          link1: "panda_link6",
          link2: "panda_link8",
        },
        {
          link1: "panda_link7",
          link2: "panda_link1",
        },
        {
          link1: "panda_link7",
          link2: "panda_link2",
        },
        {
          link1: "panda_link7",
          link2: "panda_link3",
        },
        {
          link1: "panda_link7",
          link2: "panda_link4",
        },
        {
          link1: "panda_link7",
          link2: "panda_link5",
        },
        {
          link1: "panda_link7",
          link2: "panda_link6",
        },
        {
          link1: "panda_link7",
          link2: "panda_link8",
        },
        {
          link1: "panda_link8",
          link2: "panda_link1",
        },
        {
          link1: "panda_link8",
          link2: "panda_link2",
        },
        {
          link1: "panda_link8",
          link2: "panda_link3",
        },
        {
          link1: "panda_link8",
          link2: "panda_link4",
        },
        {
          link1: "panda_link8",
          link2: "panda_link5",
        },
        {
          link1: "panda_link8",
          link2: "panda_link6",
        },
        {
          link1: "panda_link8",
          link2: "panda_link7",
        },
      ],
      urdf: `<?xml version="1.0" ?>
          <!-- =================================================================================== -->
          <!-- |    This document was autogenerated by xacro from franka_description/robots/panda_arm_hand.urdf.xacro | -->
          <!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
          <!-- =================================================================================== -->
          <robot name="panda">
            <link name="panda_link0">
            <visual>
              <geometry>
              <mesh filename="package://franka_ros/franka_description/meshes/visual/link0.dae"/>
            </geometry>
          </visual>
          <collision>
            <origin rpy="0 0 0" xyz="-0.04 0 0.06"/>
            <geometry>
              <box size="0.23 0.2 0.15"/>
            </geometry>
          </collision>
            </link>
            <link name="panda_link1">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link1.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="0 0 0" xyz="0 0.0 -0.13"/>
                <geometry>
                  <capsule length="0.06" radius="0.06"/>
                </geometry>
              </collision>
              <collision>
                <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
                <geometry>
                  <capsule length="0.135" radius="0.06"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="3.875e-03 2.081e-03 -0.1750"/>
                <mass value="4.970684"/>
                <inertia ixx="7.0337e-01" ixy="-1.3900e-04" ixz="6.7720e-03" iyy="7.0661e-01" iyz="1.9169e-02" izz="9.1170e-03"/>
              </inertial>
            </link>
            <joint name="panda_joint1" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
              <origin rpy="0 0 0" xyz="0 0 0.333"/>
              <parent link="panda_link0"/>
              <child link="panda_link1"/>
              <axis xyz="0 0 1"/>
              <limit effort="87" lower="-2.8973" upper="2.8973" velocity="2.1750"/>
            </joint>
            <link name="panda_link2">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link2.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="-1.5707963267948966 0 0" xyz="0 -0.18 0.0"/>
                <geometry>
                  <capsule length="0.11" radius="0.06"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="-3.141e-03 -2.872e-02 3.495e-03"/>
                <mass value="0.646926"/>
                <inertia ixx="7.9620e-03" ixy="-3.9250e-03" ixz="1.0254e-02" iyy="2.8110e-02" iyz="7.0400e-04" izz="2.5995e-02"/>
              </inertial>
            </link>
            <joint name="panda_joint2" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7628" soft_upper_limit="1.7628"/>
              <origin rpy="-1.5707963267948966 0 0" xyz="0 0 0"/>
              <parent link="panda_link1"/>
              <child link="panda_link2"/>
              <axis xyz="0 0 1"/>
              <limit effort="87" lower="-1.7628" upper="1.7628" velocity="2.1750"/>
            </joint>
            <link name="panda_link3">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link3.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="1.5707963267948966 0 0" xyz="0.08 0.0 0"/>
                <geometry>
                  <capsule length="0.11" radius="0.055"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="2.7518e-02 3.9252e-02 -6.6502e-02"/>
                <mass value="3.228604"/>
                <inertia ixx="3.7242e-02" ixy="-4.7610e-03" ixz="-1.1396e-02" iyy="3.6155e-02" iyz="-1.2805e-02" izz="1.0830e-02"/>
              </inertial>
            </link>
            <joint name="panda_joint3" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
              <origin rpy="1.5707963267948966 0 0" xyz="0 -0.316 0"/>
              <parent link="panda_link2"/>
              <child link="panda_link3"/>
              <axis xyz="0 0 1"/>
              <limit effort="87" lower="-2.8973" upper="2.8973" velocity="2.1750"/>
            </joint>
            <link name="panda_link4">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link4.dae"/>
                </geometry>
              </visual>
              <inertial>
                <origin rpy="0 0 0" xyz="-5.317e-02 1.04419e-01 2.7454e-02"/>
                <mass value="3.587895"/>
                <inertia ixx="2.5853e-02" ixy="7.7960e-03" ixz="-1.3320e-03" iyy="1.9552e-02" iyz="8.6410e-03" izz="2.8323e-02"/>
              </inertial>
            </link>
            <joint name="panda_joint4" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0718" soft_upper_limit="-0.0698"/>
              <origin rpy="1.5707963267948966 0 0" xyz="0.0825 0 0"/>
              <parent link="panda_link3"/>
              <child link="panda_link4"/>
              <axis xyz="0 0 1"/>
              <limit effort="87" lower="-3.0718" upper="-0.0698" velocity="2.1750"/>
            </joint>
            <link name="panda_link5">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link5.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="0 0 0" xyz="0.0 0 -0.27"/>
                <geometry>
                  <capsule length="0.09" radius="0.055"/>
                </geometry>
              </collision>
              <collision>
                <origin rpy="-0.2 0 0" xyz="0 0.08 -0.1"/>
                <geometry>
                  <box size="0.07 0.07 0.2"/>
                </geometry>
              </collision>
              <collision>
                <origin rpy="1.5707963267948966 0 0" xyz="0.0 0.04 0"/>
                <geometry>
                  <capsule length="0.1" radius="0.045"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="-1.1953e-02 4.1065e-02 -3.8437e-02"/>
                <mass value="1.225946"/>
                <inertia ixx="3.5549e-02" ixy="-2.1170e-03" ixz="-4.0370e-03" iyy="2.9474e-02" iyz="2.2900e-04" izz="8.6270e-03"/>
              </inertial>
            </link>
            <joint name="panda_joint5" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
              <origin rpy="-1.5707963267948966 0 0" xyz="-0.0825 0.384 0"/>
              <parent link="panda_link4"/>
              <child link="panda_link5"/>
              <axis xyz="0 0 1"/>
              <limit effort="12" lower="-2.8973" upper="2.8973" velocity="2.6100"/>
            </joint>
            <link name="panda_link6">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link6.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="1.5707963267948966 0 0" xyz="0.09 0.014 0.0"/>
                <geometry>
                  <cylinder length="0.12" radius="0.04"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="6.0149e-02 -1.4117e-02 -1.0517e-02"/>
                <mass value="1.666555"/>
                <inertia ixx="1.9640e-03" ixy="1.0900e-04" ixz="-1.1580e-03" iyy="4.3540e-03" iyz="3.4100e-04" izz="5.4330e-03"/>
              </inertial>
            </link>
            <joint name="panda_joint6" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-0.0175" soft_upper_limit="3.7525"/>
              <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
              <parent link="panda_link5"/>
              <child link="panda_link6"/>
              <axis xyz="0 0 1"/>
              <limit effort="12" lower="-0.0175" upper="3.7525" velocity="2.6100"/>
            </joint>
            <link name="panda_link7">
              <visual>
                <geometry>
                  <mesh filename="package://franka_ros/franka_description/meshes/visual/link7.dae"/>
                </geometry>
              </visual>
              <collision>
                <origin rpy="0 0 0" xyz="0 0.005 0.07"/>
                <geometry>
                  <cylinder length="0.04" radius="0.045"/>
                </geometry>
              </collision>
              <collision>
                <origin rpy="0 0 0.8" xyz="0.04 0.04 0.08"/>
                <geometry>
                  <box size="0.07 0.07 0.03"/>
                </geometry>
              </collision>
              <inertial>
                <origin rpy="0 0 0" xyz="1.0517e-02 -4.252e-03 6.1597e-02"/>
                <mass value="7.35522e-01"/>
                <inertia ixx="1.2516e-02" ixy="-4.2800e-04" ixz="-1.1960e-03" iyy="1.0027e-02" iyz="-7.4100e-04" izz="4.8150e-03"/>
              </inertial>
            </link>
            <joint name="panda_joint7" type="revolute">
              <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
              <origin rpy="1.5707963267948966 0 0" xyz="0.088 0 0"/>
              <parent link="panda_link6"/>
              <child link="panda_link7"/>
              <axis xyz="0 0 1"/>
              <limit effort="12" lower="-2.8973" upper="2.8973" velocity="2.6100"/>
            </joint>
            <link name="panda_link8">
              <inertial>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <mass value="0.0"/>
                  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
                </inertial>
            </link>
            <joint name="panda_joint8" type="fixed">
              <origin rpy="0 0 0" xyz="0 0 0.107"/>
              <parent link="panda_link7"/>
              <child link="panda_link8"/>
            </joint>
            <joint name="panda_hand_joint" type="fixed">
              <parent link="panda_link8"/>
              <child link="panda_connector"/>
              <origin rpy="0 0 -0.7853981633974483" xyz="0 0 0"/>
            </joint>
            <link name="panda_connector"/>
          </robot>`,
      compileFn: 10,
      updateFields: [
        "initialJointState",
        "position",
        "rotation",
        "relativeTo",
        "urdf",
      ],
      position: {
        x: 0,
        y: 0,
        z: 0.38,
      },
      rotation: {
        w: 1,
        x: 0,
        y: 0,
        z: 0,
      },
      relativeTo: "pedestal",
      status: "VALID",
      compiled: {},
      singleton: true,
      jointLimit: {
        panda_joint1: {
          lower: -2.8973,
          upper: 2.8973,
        },
        panda_joint2: {
          lower: -1.7628,
          upper: 1.7628,
        },
        panda_joint3: {
          lower: -2.8973,
          upper: 2.8973,
        },
        panda_joint4: {
          lower: -3.0718,
          upper: -0.0698,
        },
        panda_joint5: {
          lower: -2.8973,
          upper: 2.8973,
        },
        panda_joint6: {
          lower: -0.0175,
          upper: 3.7525,
        },
        panda_joint7: {
          lower: -2.8973,
          upper: 2.8973,
        },
      },
    },
    dataType: "INSTANCE",
    selected: false,
    editing: false,
  },
};
