
import rospy

from std_msgs.msg import Empty, Bool


class RobotControlInterface:

    def __init__(self, at_start_cb=None, at_end_cb=None, lockout_cb=None):

        self.use_simulated_robot_pub = rospy.Publisher('robot_control_server/use_simulated_robot',Bool,queue_size=10)
        self.use_physical_robot_pub = rospy.Publisher('robot_control_server/use_physical_robot',Bool,queue_size=10)
        self.freedrive_pub = rospy.Publisher('robot_control_server/freedrive',Bool,queue_size=10)

        self.play_pub = rospy.Publisher('robot_control_server/play',Empty,queue_size=10)
        self.stop_pub = rospy.Publisher('robot_control_server/stop',Empty,queue_size=10)
        self.pause_pub = rospy.Publisher('robot_control_server/pause',Empty,queue_size=10)
        self.reset_pub = rospy.Publisher('robot_control_server/reset',Empty,queue_size=10)
        self.step_fwd_pub = rospy.Publisher('robot_control_server/step_forward',Empty,queue_size=10)
        self.step_bkd_pub = rospy.Publisher('robot_control_server/step_backward',Empty,queue_size=10)

        self.at_start_sub = rospy.Subscriber('robot_control_server/at_start',Bool,lambda x: at_start_cb(x) if at_start_cb != None else print x)
        self.at_end_sub = rospy.Subscriber('robot_control_server/at_end',Bool,lambda x: at_end_cb(x) if at_end_cb != None else print x)
        self.lockout_sub = rospy.Subscriber('robot_control_server/lockout',Bool,lambda x: lockout_cb(x) if lockout_cb != None else print x)
