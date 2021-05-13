#TODO should provide an ID and a set of hooks. Can convert to low-level implementation code
import rospy

from evd_interfaces.machine_template import MachineTemplate


class PrinterMachineNode(MachineTemplate):

    def __init__(self, uuid, prefix=None):
        super(PrinterMachineNode,self).__init__(
            uuid, prefix, 
            init_fnt=self._init,
            start_fnt=self._start,
            stop_fnt=self._stop,
            pause_fnt=self._pause)

    def _init(self):
        pass

    def _start(self):
        pass

    def _stop(self, emergency):
        pass

    def _pause(self, state):
        pass


if __name__ == "__main__":
    rospy.init_node('3d_printer_node')

    uuid = rospy.get_param('uuid','default-3d-printer-uuid')
    prefix = rospy.get_param('prefix',None)

    node = PrinterMachineNode(uuid,prefix)
    rospy.spin()