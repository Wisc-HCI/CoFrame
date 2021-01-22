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
    cnc = Machine(name='cnc')

    prog.context.add_machine(cnc)

    # Define and add statically defined things
    raw_stock = Thing('stock',mesh_id='package://evd_ros_core/markers/3DBenchy.stl',name='boat')

    prog.context.add_thing(raw_stock)

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

        startPos = prog.context.get_location(trajectory.start_location_uuid).position
        endPos = prog.context.get_location(trajectory.end_location_uuid).position

        xFunc = interpolate.interp1d([0,1],[startPos.x,endPos.x])
        yFunc = interpolate.interp1d([0,1],[startPos.y,endPos.y])
        zFunc = interpolate.interp1d([0,1],[startPos.z,endPos.z])

        # Put a waypoint midway between locations
        NUM_WPS = 2
        waypoint_uuids = []
        for i in range(0,NUM_WPS+1):
            if i+1 == NUM_WPS:
                break

            w = Waypoint(
                position=Position(
                    x=xFunc((i+1.0)/NUM_WPS).tolist(),
                    y=yFunc((i+1.0)/NUM_WPS).tolist(),
                    z=zFunc((i+1.0)/NUM_WPS).tolist()),
                orientation=Orientation(
                    x=0,
                    y=0.7071068,
                    z=0,
                    w=0.7071068))

            prog.context.add_waypoint(w)
            waypoint_uuids.append(w.uuid)

        trajectory.waypoint_uuids = waypoint_uuids

        # Put a trace datapoint at each location and waypoint
        trace = Trace('ee',{'ee': [
            TraceDataPoint(home_loc.position,home_loc.orientation),
            TraceDataPoint(retract_loc.position,retract_loc.orientation),
            TraceDataPoint(place_final_loc.position,place_final_loc.orientation)]})

        trajectory.trace = trace

    #print '\n\n\n\n'
    #print 'Cache Log'
    #pprint.pprint(cache.utility_cache_stats())

    #pprint.pprint(prog.context.to_dct())
    #print json.dumps(prog.context.to_dct())

    #===========================================================================
    # Define Environment
    #===========================================================================

    env = Environment(
        reach_sphere=ReachSphere(),
        pinch_points=[],
        collision_meshes=[],
        occupancy_zones=[
            OccupancyZone(OccupancyZone.HUMAN_TYPE),
            OccupancyZone(OccupancyZone.ROBOT_TYPE)],
        locations=locations,
        trajectories=cache.trajectories.values())

    return {"program": prog, "environment": env}
