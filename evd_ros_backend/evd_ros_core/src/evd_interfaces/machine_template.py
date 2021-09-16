'''
This abstracts controlling machines (all task machines should use use this!)

Specifically, this template should be extended by task specific software to expose
machines in EvD.
'''

import json
import rospy

from std_msgs.msg import String
from evd_script import NodeParser, Machine
from evd_ros_core.msg import MachineAck, MachineInitialize, MachinePause, MachineStart, MachineStop, MachineStatus


class MachineTemplate:

    VALID_STATES = [
        MachineStatus.STATUS_RUNNING,
        MachineStatus.STATUS_IDLE,
        MachineStatus.STATUS_PAUSED,
        MachineStatus.STATUS_ERROR
    ]

    @classmethod
    def generate_uuid(cls):
        import uuid
        return uuid.uuid1().hex

    def __init__(self, uuid='', prefix=None, frontend_prefix=None, init_fnt=None, start_fnt=None, 
                 stop_fnt=None, pause_fnt=None):
        self.uuid = uuid
        self._prefix = prefix
        prefix_fmt = prefix+'/' if prefix != None else ''
        self._frontend_prefix = frontend_prefix
        frontend_prefix_fmt = frontend_prefix+'/' if frontend_prefix != None else ''
        self._running = False
        self._status = 'unknown'
        self._evdReprentation = None

        self._init_fnt = init_fnt
        self._start_fnt = start_fnt
        self._stop_fnt = stop_fnt
        self._pause_fnt = pause_fnt

        self.update_sub = rospy.Subscriber('{0}program/configure/machines'.format(frontend_prefix_fmt), String, self._update_cb)

        self.ack_pub = rospy.Publisher('{0}machine/ack'.format(prefix_fmt), MachineAck, queue_size=10)
        self.status_pub = rospy.Publisher('{0}machine/status'.format(prefix_fmt), MachineStatus, queue_size=10)

        self.initialize_sub = rospy.Subscriber('{0}machine/initialize'.format(prefix_fmt), MachineInitialize, self._initialize_cb)
        self.start_sub = rospy.Subscriber('{0}machine/start'.format(prefix_fmt), MachineStart, self._start_cb)
        self.stop_sub = rospy.Subscriber('{0}machine/stop'.format(prefix_fmt), MachineStop, self._stop_cb)
        self.pause_sub = rospy.Subscriber('{0}machine/pause'.format(prefix_fmt), MachinePause, self._pause_cb)

    def _update_cb(self, msg):
        machines = json.loads(msg.data)

        for m in machines:
            if m['uuid'] == self.uuid:
                self._evdReprentation = NodeParser(m, enforce_types=[Machine])

    def _initialize_cb(self, msg):
        if self.uuid == msg.uuid:

            # Call machine routine
            ack = True
            if self._init_fnt != None:
                ack = self._init_fnt(msg.emergency) == True

            # Generate ACK/NACK
            retmsg = MachineAck(self.uuid,ack)
            self.ack_pub.publish(retmsg)

    def _start_cb(self, msg):
        if self.uuid == msg.uuid:

            # Call machine routine
            ack = True
            if self._start_fnt != None:
                ack = self._start_fnt() == True

            # Generate ACK/NACK
            retmsg = MachineAck(self.uuid,ack)
            self.ack_pub.publish(retmsg)

    def _stop_cb(self, msg):
        if self.uuid == msg.uuid:

            # Call machine routine
            ack = True
            if self._stop_fnt != None:
                ack = self._stop_fnt(msg.emergency) == True

            # Generate ACK/NACK
            retmsg = MachineAck(self.uuid,ack)
            self.ack_pub.publish(retmsg)

    def _pause_cb(self, msg):
        if self.uuid == msg.uuid:

            # Call machine routine
            ack = True
            if self._pause_fnt != None:
                ack = self._pause_fnt(msg.state) == True

            # Generate ACK/NACK
            retmsg = MachineAck(self.uuid,ack)
            self.ack_pub.publish(retmsg)

    @property
    def is_running(self):
        return self._running

    @property
    def current_status(self):
        return self._status

    @property
    def evdMachine(self):
        return self._evdReprentation

    @current_status.setter
    def current_status(self, value):
        
        if value not in self.VALID_STATES:
            raise Exception('Invalid status state specified: {0}'.format(value))
        
        self._status = value
        if self._status == MachineStatus.STATUS_RUNNING:
            self._running = True

        self.update_status()

    def update_status(self):
        msg = MachineStatus(self.uuid,self.is_running,self.current_status)
        self.status_pub.publish(msg)

