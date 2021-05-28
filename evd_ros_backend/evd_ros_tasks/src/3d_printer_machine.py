#!/usr/bin/env python3

'''
#TODO should provide an ID and a set of hooks. Can convert to low-level implementation code
'''


import rospy

from evd_script import Machine, MachineRecipe, CubeRegion, Position, Orientation, ThingType

from evd_interfaces.machine_template import MachineTemplate
from evd_interfaces.data_client_interface import DataClientInterface


PROCESS_TIME = 5 #seconds (not realistic but ¯\_(ツ)_/¯)


class PrinterMachineNode(MachineTemplate):

    def __init__(self, position, orientation, uuid, prefix=None, rate=5, simulated=True):
        super(PrinterMachineNode,self).__init__(
            uuid, prefix, 
            init_fnt=self._init,
            start_fnt=self._start,
            stop_fnt=self._stop,
            pause_fnt=self._pause)

        self._data_client = DataClientInterface(
            sub_to_update=False, 
            store_program=False,
            on_program_update_cb=self._on_server_program_push)

        self._evd_thing_type = ThingType(
            type_name='print',
            is_safe=True,
            weight=0,
            mesh_id='package://evd_ros_core/markers/3DBenchy.stl',
            uuid=uuid+'+thing_type'
        )

        self._evd_machine = Machine(
            input_regions={},
            output_regions={
                self._evd_thing_type.uuid: CubeRegion(
                    center_position=Position.from_list(position),
                    center_orientation=Orientation.from_list(orientation),
                    uncertainty_x=0,
                    uncertainty_y=0,
                    uncertainty_z=0)
            },
            recipe=MachineRecipe(
                process_time=PROCESS_TIME,
                input_thing_quantities={},
                output_thing_quantities={
                    self._evd_thing_type.uuid: 1
                }
            ),
            uuid=uuid+'+machine'
        )

        self._paused = False
        self._rate = rate
        self._simulated = simulated

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            self.update_status()
            rate.sleep()

    '''
    Machine Template Methods
    '''

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

    '''
    Data Server Registration
    '''

    def _on_server_program_push(self):
        pass #TODO retrieve UUIDs to check if printer is in list


if __name__ == "__main__":
    rospy.init_node('3d_printer_node')

    position = rospy.get_param('position',[0,0,0]) #xyz
    orientation = rospy.get_param('orientation',[0,0,0,0]) #xyzw
    uuid = rospy.get_param('uuid','default-3d-printer-uuid')
    prefix = rospy.get_param('prefix',None)
    rate = rospy.get_param('rate',5)
    simulated = rospy.get_param('simulated',True)

    node = PrinterMachineNode(position,orientation,uuid,prefix,rate,simulated)
    node.spin()