
class FakeGripperActionServer:

    def __init__(self):
        self._joint_state_pub = rospy.Publisher('gripper/joint_state',JointState,queue_size=5)
        self._server = actionlib.SimpleActionServer('gripper_command', GripperCommandAction, self._execute, False)
        self._server.start()

    def _execute(self, goal):

        jointState = JointState()
        jointState.name = ['robotiq_85_left_knuckle_joint']
        jointState.position = [goal.command.position]
        self._joint_state_pub.publish(jointState)

        result = GripperCommandResult()
        result.position = goal.command.position
        result.stalled = False
        result.reached_goal = True
        self._server.set_succeeded(result)
