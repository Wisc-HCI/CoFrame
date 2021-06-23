#!/usr/bin/env python3

'''
Presents a virtual 3D printer machine to EvD.

Behavior implementation hooks are stubbed. Machine and thing type registration is
also handled.
'''

import rospy

from evd_script import Machine, CubeRegion, Position, Orientation, ThingType, \
    CollisionMesh, Placeholder, Thing

from evd_interfaces.machine_template import MachineTemplate
from evd_interfaces.frontend_interface import FrontendInterface


PROCESS_TIME = 5 #seconds (not realistic but ¯\_(ツ)_/¯)


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

        self._evd_thing_type = ThingType(
            name='3D Print',
            is_safe=True,
            weight=0,
            mesh_id='package://evd_ros_core/markers/3DBenchy.stl',
            uuid=uuid+'+thing_type')

        thing = Thing(thing_type_uuid=self._evd_thing_type.uuid, name='3D Print - Placeholder Object')

        self._evd_thing_placeholder = Placeholder(
            pending_node_dct=thing.to_dct(),
            pending_fields=[
                'position',
                'orientation'
            ]
        )

        self._evd_region = CubeRegion(
            uuid=uuid+'+region',
            link='3d_printer_link', 
            center_position=Position(0,0,0),
            center_orientation=Orientation.Identity(),
            uncertainty_x=0.01,
            uncertainty_y=0.01,
            uncertainty_z=0.01)

        self._evd_collision_mesh = CollisionMesh(
            link='3d_printer_link', 
            mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl')

        self._evd_machine = Machine(
            name='3D Printer',
            uuid=uuid+'+machine',
            inputs={},
            outputs={
                self._evd_thing_type.uuid: [
                    {
                        'region_uuid': self._evd_region.uuid,
                        'quantity': 1,
                        'placeholder_uuids': [
                            self._evd_thing_placeholder.uuid
                        ]
                    }
                ]
            },   
            process_time=PROCESS_TIME,
            link='3d_printer_link', 
            mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/visual_meshes/MK2-Printer.stl',
            collision_mesh_uuid=self._evd_collision_mesh.uuid)

        self._program = FrontendInterface(use_registration=True, register_cb=self._call_to_register)


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

    def _call_to_register(self):
        dct_list = []
        
        dct_list.append(self._evd_thing_type.to_dct())
        dct_list.append(self._evd_region.to_dct())
        dct_list.append(self._evd_collision_mesh.to_dct())
        dct_list.append(self._evd_machine.to_dct())

        self._program.register(dct_list)

        print('machine-registered')


if __name__ == "__main__":
    rospy.init_node('3d_printer_node')

    uuid = rospy.get_param('uuid','default-3d-printer-uuid')
    prefix = rospy.get_param('prefix',None)
    rate = rospy.get_param('rate',5)
    simulated = rospy.get_param('simulated',True)

    node = PrinterMachineNode(uuid,prefix,rate,simulated)
    node.spin()