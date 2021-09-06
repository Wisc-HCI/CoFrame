#!/usr/bin/env python3

'''
Registers robot and environment information with EvD.
'''

import rospy

from evd_script import Position, ReachSphere, PinchPoint, CollisionMesh, OccupancyZone

from evd_interfaces.frontend_interface import FrontendInterface


class URRobotEnvironment:

    def __init__(self):

        self._collision_meshes = [
            CollisionMesh(link='table_link', mesh_id='package://evd_ros_tasks/description/meshes/collision/Table.stl'),
        ]
        self._occupancy_zones = [
            OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=1, sclX=2, height=-0.77),
            OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=-0.8, sclX=2, height=-0.77),
        ]

        self._program = FrontendInterface(use_registration=True, register_cb=self._call_to_register)

    def _call_to_register(self):
        dct_list = []

        dct_list.extend([c.to_dct() for c in self._collision_meshes])
        dct_list.extend([o.to_dct() for o in self._occupancy_zones])

        self._program.register(dct_list)

        print('workcell-registered')


if __name__ == "__main__":
    rospy.init_node('register_static_workcell')
    node = URRobotEnvironment()
    rospy.spin()