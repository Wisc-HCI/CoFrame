'''
Wraps the interaction with the high-level robot control server.

This interface allows the client to quickly trigger a program to run and switch
allowed agents.
'''


import rospy

from std_msgs.msg import Empty, Bool


class RobotControlInterface:

    def __init__(self, at_start_cb=None, at_end_cb=None, lockout_cb=None):

        self._user_at_start_cb = at_start_cb
        self._user_at_end_cb = at_end_cb
        self._user_lockout_cb = lockout_cb

        self.use_simulated_robot_pub = rospy.Publisher('robot_control_server/use_simulated_robot',Bool,queue_size=10)
        self.use_physical_robot_pub = rospy.Publisher('robot_control_server/use_physical_robot',Bool,queue_size=10)
        self.freedrive_pub = rospy.Publisher('robot_control_server/freedrive',Bool,queue_size=10)

        self.play_pub = rospy.Publisher('robot_control_server/play',Empty,queue_size=10)
        self.stop_pub = rospy.Publisher('robot_control_server/stop',Empty,queue_size=10)
        self.pause_pub = rospy.Publisher('robot_control_server/pause',Empty,queue_size=10)
        self.reset_pub = rospy.Publisher('robot_control_server/reset',Empty,queue_size=10)

        self.at_start_sub = rospy.Subscriber('robot_control_server/at_start',Bool,self._at_start_cb)
        self.at_end_sub = rospy.Subscriber('robot_control_server/at_end',Bool,self._at_end_cb)
        self.lockout_sub = rospy.Subscriber('robot_control_server/lockout',Bool,self._lockout_cb)

    def _at_start_cb(self, msg):
        if self._user_at_start_cb != None:
            self._user_at_start_cb(msg.data)

    def _at_end_cb(self, msg):
        if self._user_at_end_cb != None:
            self._user_at_end_cb(msg.data)

    def _lockout_cb(self, msg):
        if self._user_lockout_cb != None:
            self._user_lockout_cb(msg.data)

    def use_simulated_robot(self, state):
        self.use_simulated_robot.publish(Bool(state))

    def use_physical_robot(self, state):
        self.use_physical_robot_pub.publish(Bool(state))

    def enable_freedrive(self, state):
        self.freedrive_pub.publish(Bool(state))

    def play(self):
        self.play_pub.publish(Empty())

    def stop(self):
        self.stop_pub.publish(Empty())

    def pause(self):
        self.pause_pub.publish(Empty())

    def reset(self):
        self.reset_pub.publish(Empty())
