#!/usr/bin/env python3

'''
This application is used to stress the educational dimension of EvD
'''

from scipy import interpolate
from evd_script import *


def generate():
    
    cache = get_evd_cache_obj()
    prog = Program()

    #===========================================================================
    # Define Program Data
    #===========================================================================

    

    #===========================================================================
    # Define Environment
    #===========================================================================

    prog.environment.reach_sphere = ReachSphere(0.8, offset=Position(0,0,0.15))
    prog.environment.pinch_points = [
        PinchPoint(link='simulated_shoulder_link', radius=0.075, length=0.2, offset=Position.from_axis('z',-0.05)),
        PinchPoint(link='simulated_upper_arm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
        PinchPoint(link='simulated_forearm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
        PinchPoint(link='simulated_wrist_1_link', radius=0.06, length=0.17, offset=Position.from_axis('z',-0.05)),
        PinchPoint(link='simulated_wrist_3_link', radius=0.1, length=0.16, offset=Position.from_axis('z',0.1))
    ]
    prog.environment.collision_meshes = [
        CollisionMesh(link='table_2_link'),
        CollisionMesh(link='assembly_jig_link'),
        CollisionMesh(link='blade_conveyor_link'),
        CollisionMesh(link='knife_conveyor_link'),
        CollisionMesh(link='table_1_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Table.stl'),
        CollisionMesh(link='3d_printer_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl'),
        CollisionMesh(link='ur3e_pedestal_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Pedestal.stl')
    ]
    prog.environment.occupancy_zones = [
        OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=1, sclX=2, height=-0.77),
        OccupancyZone(OccupancyZone.ROBOT_TYPE, sclX=1.6, sclZ=1.2, height=-0.77)
    ]

    #===========================================================================
    # Define Program Primitives
    #===========================================================================

    #TODO

    #===========================================================================
    # Done :)
    #===========================================================================

    prog.late_construct_update() # This will force the entire program to undergo patch/repair
    return prog


if __name__ == "__main__":
    import rospy
    rospy.init_node('test_node_application')

    program = generate()

    import json
    with open('knife_task_exported.json','w+') as f:
        json.dump(program.to_dct(), f, indent=4)