import json
import pprint

from scipy import interpolate
from evd_script import *


def generate():

    cache = get_evd_cache_obj()

    #===========================================================================
    # Define Program
    #===========================================================================
    prog = Program()
    default_objs = {
        "machines": [],
        "things": []
    }

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

    locations = [home_loc,pick_stock_loc,place_stock_loc,retract_loc,pick_final_loc,place_final_loc]

    # Define and add machines
    cnc = Machine(name='cnc',uuid="reserved-machine-cnc")

    prog.context.add_machine(cnc)
    default_objs["machines"].append(cnc)

    # Define and add statically defined things
    raw_stock = Thing('stock',mesh_id='package://evd_ros_core/markers/3DBenchy.stl',name='boat',uuid="reserved-thing-raw-stock")

    prog.context.add_thing(raw_stock)
    default_objs["things"].append(raw_stock)

    # Define and add primitives
    prog.add_primitive(Initialize(
        homeLocUuid=home_loc.uuid,
        machineUuids=[cnc.uuid]))

    prog.add_primitive(Loop(
        primitives=[
            SimplePickAndPlace(
                startLocUuid=home_loc.uuid,
                pickLocUuid=pick_stock_loc.uuid,
                placeLocUuid=place_stock_loc.uuid),
            MoveTrajectory(
                startLocUuid=place_stock_loc.uuid,
                endLocUuid=retract_loc.uuid),
            MachineBlockingProcess(
                machineUuid=cnc.uuid),
            SimplePickAndPlace(
                startLocUuid=retract_loc.uuid,
                pickLocUuid=pick_final_loc.uuid,
                placeLocUuid=place_final_loc.uuid)
        ]
    ))

    for trajUuid in cache.trajectories.keys():
        trajectory = cache.trajectories[trajUuid]

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

        # Put a trace datapoint at each location and waypoint
        datapoints = []
        datapoints.append(TraceDataPoint(startLoc.position,startLoc.orientation))
        for uuid in waypoint_uuids:
            wp = prog.context.get_waypoint(uuid)
            datapoints.append(TraceDataPoint(wp.position,wp.orientation))
        datapoints.append(TraceDataPoint(endLoc.position,endLoc.orientation))

        trajectory.trace = Trace('ee',{'ee': datapoints})

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
        CollisionMesh(link='box_link', mesh_id='package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Box.stl'),
        CollisionMesh(link='table_link', mesh_id='package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Table.stl'),
        CollisionMesh(link='3d_printer_link', mesh_id='package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl'),
        CollisionMesh(link='ur3e_pedestal_link', mesh_id='package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Pedestal.stl')
    ]
    prog.environment.occupancy_zones = [
        OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=1, sclX=2, height=-0.77),
        OccupancyZone(OccupancyZone.HUMAN_TYPE, posZ=-0.8, sclX=2, height=-0.77),
        OccupancyZone(OccupancyZone.ROBOT_TYPE, sclX=1.6, sclZ=1.2, height=-0.77)
    ]

    #===========================================================================
    # Repair all orphans and integrate all context patches
    #===========================================================================

    prog.late_construct_update() # This will force the entire program to undergo patch/repair

    #===========================================================================
    # Debug Logging
    #===========================================================================

    #print '\n\n\n\n'
    #print 'Data Server - Cache Log'
    #pprint.pprint(cache.utility_cache_stats())

    #pprint.pprint(prog.context.to_dct())

    #print 'Data Server - Context Trajectories'
    #pprint.pprint(prog.context.trajectories)

    #print '\n\n\n\n'

    return {"program": prog, "default_objects": default_objs}
