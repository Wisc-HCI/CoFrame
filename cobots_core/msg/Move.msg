string CIRCULAR = circular
string JOINT = joint
string LINEAR = linear
string PROCESS = process
string motion_type

# Only needed for joint type move. Uses target pose instead of target_joints
bool use_ur_ik

# Use pose for circular, linear, and process modes and maybe for joint mode if
# use_ur_ik is true otherwise use joints
geometry_msgs/Pose target_pose
float32[] target_joints

# Used for circular movement
geometry_msgs/Pose path_pose

float32 STD_RADIUS = 0
float32 radius

float32 STD_ACCELERATION = 1.2
float32 JOINT_ACCELERATION = 1.4
float32 acceleration

float32 STD_VELOCITY = 0.25
float32 JOINT_VELOCITY = 1.05
float32 velocity

float32 STD_TIME = 0
float32 time

# Only for circular mode, constrains to fixed EE orientation
bool circular_constrained
