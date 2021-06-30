#!/usr/bin/env python3

'''
This application is used to demonstrate EvDScript's modeling ability.
'''

from scipy import interpolate
from evd_script import *


def generate():

    cache = get_evd_cache_obj()
    prog = Program()

    #===========================================================================
    # Define Program Data
    #===========================================================================

    # Define and add location
    home_loc = Location(Position(0,0.2,0),Orientation(x=0,y=0.7071068,z=0,w=0.7071068),name='home')
    pick_stock_loc = Location(Position(0.1,0.3,0),Orientation(x=0,y=0.7071068,z=0,w=0.7071068), name='pick stock')
    place_stock_loc = Location(Position(0.4,0.4,0.2),Orientation(x=0,y=0.7071068,z=0,w=0.7071068), name='place stock')
    retract_loc = Location(Position(0.3,0.3,0.4),Orientation(x=0,y=0.7071068,z=0,w=0.7071068), name='retract')
    pick_final_loc = Location(Position(0.4,0.4,0.2),Orientation(x=0,y=0.7071068,z=0,w=0.7071068), name='pick_final')
    place_final_loc = Location(Position(-0.1,-0.1,-0.1),Orientation(x=0,y=0.7071068,z=0,w=0.7071068), name='place_final')

    prog.context.add_location(home_loc)
    prog.context.add_location(pick_stock_loc)
    prog.context.add_location(place_stock_loc)
    prog.context.add_location(retract_loc)
    prog.context.add_location(pick_final_loc)
    prog.context.add_location(place_final_loc)

    # Define and add machines
    cnc = Machine(name='cnc',uuid="reserved-machine-cnc")

    prog.context.add_machine(cnc)

    # Define and add statically defined things
    stock_type = ThingType(name='stock', is_safe=True, weight=0.1,
                           mesh_id='package://evd_ros_core/markers/3DBenchy.stl',
                           uuid="reserved-thing-type-stock")
    prog.context.add_thing_type(stock_type)

    raw_stock = Thing(thing_type_uuid=stock_type.uuid,
                      name='boat',
                      uuid="reserved-thing-raw-stock")
    prog.context.add_thing(raw_stock)

    # Define and add trajectories

    traj1 = Trajectory(home_loc.uuid,pick_stock_loc.uuid)
    traj2 = Trajectory(pick_stock_loc.uuid,place_stock_loc.uuid)
    traj3 = Trajectory(place_stock_loc.uuid,retract_loc.uuid)
    traj4 = Trajectory(retract_loc.uuid,pick_final_loc.uuid)
    traj5 = Trajectory(pick_final_loc.uuid,place_final_loc.uuid)
    traj6 = Trajectory(place_final_loc.uuid,home_loc.uuid)

    prog.context.add_trajectory(traj1)
    prog.context.add_trajectory(traj2)
    prog.context.add_trajectory(traj3)
    prog.context.add_trajectory(traj4)
    prog.context.add_trajectory(traj5)
    prog.context.add_trajectory(traj6)

    # Create fake waypoints for trajectories
    for trajUuid in cache.get_uuids('trajectory'):
        trajectory = cache.get(trajUuid,'trajectory')

        startLoc = prog.context.get_location(trajectory.start_location_uuid)
        endLoc = prog.context.get_location(trajectory.end_location_uuid)

        xFunc = interpolate.interp1d([0,1],[startLoc.position.x,endLoc.position.x])
        yFunc = interpolate.interp1d([0,1],[startLoc.position.y,endLoc.position.y])
        zFunc = interpolate.interp1d([0,1],[startLoc.position.z,endLoc.position.z])

        # Put a waypoint midway between locations
        NUM_WPS = 2
        waypoint_uuids = []
        for i in range(0,NUM_WPS+1):
            if i == NUM_WPS:
                break

            w = Waypoint(
                position=Position(
                    x=xFunc((i+1.0)/(NUM_WPS+1)).tolist(),
                    y=yFunc((i+1.0)/(NUM_WPS+1)).tolist(),
                    z=zFunc((i+1.0)/(NUM_WPS+1)).tolist()),
                orientation=Orientation(
                    x=0,
                    y=0.7071068,
                    z=0,
                    w=0.7071068))

            prog.context.add_waypoint(w)
            waypoint_uuids.append(w.uuid)

        trajectory.waypoint_uuids = waypoint_uuids

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
        CollisionMesh(link='box_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Box.stl'),
        CollisionMesh(link='table_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Table.stl'),
        CollisionMesh(link='3d_printer_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl'),
        CollisionMesh(link='ur3e_pedestal_link', mesh_id='package://evd_ros_tasks/description/3d_printer_machine_tending/collision_meshes/Pedestal.stl')
    ]
    prog.environment.occupancy_zones = [
        OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=1, sclX=2, height=-0.77),
        OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=-0.8, sclX=2, height=-0.77),
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
    rospy.init_node('test_node_placeholder_name')

    program = generate()

    import json
    with open('debug_app_exported.json','w+') as f:
        json.dump(program.to_dct(),f, indent=4)