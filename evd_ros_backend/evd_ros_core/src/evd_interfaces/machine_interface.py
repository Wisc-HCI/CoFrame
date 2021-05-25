'''
This abstracts controlling a machine in EvD. All machines should follow this interface (for instance 
by using the template).

Machine primitives are specifically written with this interface in mind.
'''

import rospy

from evd_ros_core.msg import MachineAck, MachineInitialize, MachinePause, MachineStart, MachineStop, MachineStatus


class MachineInterface:

    def __init__(self, prefix=None):
        self._prefix = prefix
        prefix_fmt = prefix+'/' if prefix != None else ''
        self._status_table = {}
        self._ack_table = {}

        self.initialize_pub = rospy.Publisher('{0}machine/initialize'.format(prefix_fmt), MachineInitialize, queue_size=10)
        self.start_pub = rospy.Publisher('{0}machine/start'.format(prefix_fmt), MachineStart, queue_size=10)
        self.stop_pub = rospy.Publisher('{0}machine/stop'.format(prefix_fmt), MachineStop, queue_size=10)
        self.pause_pub = rospy.Publisher('{0}machine/pause'.format(prefix_fmt), MachinePause, queue_size=10)

        self.ack_sub = rospy.Subscriber('{0}machine/ack'.format(prefix_fmt), MachineAck, self._ack_cb)
        self.status_sub = rospy.Subscriber('{0}machine/status'.format(prefix_fmt), MachineStatus, self._status_cb)

    def _ack_cb(self, msg):
        self._ack_table[msg.uuid] = msg.ack

    def _status_cb(self, msg):
        self._status_table[msg.uuid] = {'running': msg.running, 'status': msg.status}

    def estop(self, machineUuid):
        msg = MachineStop(machineUuid,True)
        self.stop_pub.publish(msg)

    def pause(self, state, machineUuid):
        msg = MachinePause(machineUuid,state)
        self.pause_pub.publish(msg)

    def initialize(self, machineUuid):
        msg = MachineInitialize(machineUuid)
        self.initialize_pub.publish(msg)

    def start(self, machineUuid):
        msg = MachineStart(machineUuid)
        self.start_pub.publish(msg)

    def stop(self, machineUuid):
        msg = MachineStop(machineUuid,False)
        self.stop_pub.publish(msg)

    def is_acked(self, machineUuid, clearEntry=True):
        if not machineUuid in self._ack_table.keys():
            return None
        else:
            ack = self._ack_table[machineUuid]
            if clearEntry:
                del self._ack_table[machineUuid]
            return ack

    def get_status(self, machineUuid):
        if not machineUuid in self._status_table.keys():
            return None
        else:
            return self._status_table[machineUuid]
