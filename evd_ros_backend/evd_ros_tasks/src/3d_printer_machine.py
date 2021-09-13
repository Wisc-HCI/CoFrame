#!/usr/bin/env python3

'''
Presents a virtual 3D printer machine to EvD.

Behavior implementation hooks are stubbed. Machine and thing type registration is
also handled.
'''

import rospy

from evd_interfaces.machine_template import MachineTemplate


PROCESS_TIME_3D_PRINTER = 5
MACHINE_UUID_3D_PRINTER = '3d-printer-machine-uuid'


class PrinterMachineNode(MachineTemplate):

    def __init__(self, uuid, prefix=None, rate=5, simulated=True):
        self._paused = False
        self._rate = rate
        self._simulated = simulated

        super(PrinterMachineNode,self).__init__(
            uuid, prefix, 
            init_fnt=self._init,
            start_fnt=self._start,
            stop_fnt=self._stop,
            pause_fnt=self._pause)

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            self.update_status()
            rate.sleep()

    def _init(self):
        if not self.is_running:
            self.current_status = 'idle'
            return True
        else:
            self._stop(True)
            return False

    def _start(self):
        if self.current_status == 'idle':
            self.current_status = 'running'
            self._paused = False
            return True
        else:
            return False

    def _stop(self, emergency):
        if emergency:
            self.current_status = 'error'
        else:
            self.current_status = 'idle'
        return True

    def _pause(self, state):

        if self.is_running:
            self._paused = state

            if self._paused:
                self.current_status = 'paused'
            else:
                self.current_status = 'running'
            
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node('3d_printer_node')

    uuid = rospy.get_param('uuid',MACHINE_UUID_3D_PRINTER)
    prefix = rospy.get_param('prefix',None)
    rate = rospy.get_param('rate',5)
    simulated = rospy.get_param('simulated',True)

    node = PrinterMachineNode(uuid,prefix,rate,simulated)
    node.spin()