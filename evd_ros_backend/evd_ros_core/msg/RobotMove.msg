string JOINT = joint
string EE_IK = ee_ik
string motion_type

# fill in target pose if ee_ik otherwise fill in joints 
# (joints must be of appropriate length)
geometry_msgs/Pose target_pose
float32[] target_joints

float32 STD_VELOCITY = 0.25
float32 JOINT_VELOCITY = 1.05
float32 velocity

# if zero attempts to run as fast as it can
# otherwise will try to oblige sampling timestep
float32 STD_TIMESTEP = 0
float32 timestep

# enforce a user manual override to run behavior
bool manual_safety
