
string CIRCULAR = circular
string JOINT = joint
string motion_type

# Only needed for joint type servoing. Uses target pose instead of target_joints
bool use_ur_ik

# Use pose for circular mode and maybe for joint mode if use_ur_ik is true
# otherwise use joints
geometry_msgs/Pose target_pose
trajectory_msgs/JointTrajectoryPoint target_joints

# Only used for circular servoing
float32 STD_RADIUS = 0
float32 radius

float32 STD_ACCELERATION = 1.2
float32 acceleration

float32 STD_VELOCITY = 0.25
float32 velocity
