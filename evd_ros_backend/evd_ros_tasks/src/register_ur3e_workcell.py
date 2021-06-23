#!/usr/bin/env python3

'''
Registers robot and environment information with EvD.
'''

import rospy

from evd_script import Position, ReachSphere, PinchPoint, CollisionMesh, OccupancyZone

from evd_interfaces.frontend_interface import FrontendInterface


class URRobotEnvironment:

    def __init__(self, uuid_base):

        self._reach_sphere = ReachSphere(0.8, offset=Position(0,0,0.15), uuid='{0}-reach-sphere'.format(uuid_base))
        self._pinch_points = [
            PinchPoint(link='simulated_shoulder_link', radius=0.075, length=0.2, offset=Position.from_axis('z',-0.05)),
            PinchPoint(link='simulated_upper_arm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
            PinchPoint(link='simulated_forearm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
            PinchPoint(link='simulated_wrist_1_link', radius=0.06, length=0.17, offset=Position.from_axis('z',-0.05)),
            PinchPoint(link='simulated_wrist_3_link', radius=0.1, length=0.16, offset=Position.from_axis('z',0.1))
        ]
        self._collision_meshes = [
            CollisionMesh(link='box_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Box.stl'),
            CollisionMesh(link='table_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Table.stl'),
            CollisionMesh(link='ur3e_pedestal_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Pedestal.stl')
        ]
        self._occupancy_zones = [
            OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=1, sclX=2, height=-0.77),
            OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=-0.8, sclX=2, height=-0.77),
            OccupancyZone(OccupancyZone.ROBOT_TYPE, sclX=1.6, sclZ=1.2, height=-0.77)
        ]

        self._program = FrontendInterface(use_registration=True, register_cb=self._call_to_register)

    def _call_to_register(self):
        dct_list = []

        dct_list.append(self._reach_sphere.to_dct())
        dct_list.extend([p.to_dct() for p in self._pinch_points])
        dct_list.extend([c.to_dct() for c in self._collision_meshes])
        dct_list.extend([o.to_dct() for o in self._occupancy_zones])

        self._program.register(dct_list)

        print('workcell-registered')


if __name__ == "__main__":
    rospy.init_node('ur_robot_environment')

    uuid = rospy.get_param('uuid','default-ur-environment-uuid')

    node = URRobotEnvironment(uuid)
    rospy.spin()