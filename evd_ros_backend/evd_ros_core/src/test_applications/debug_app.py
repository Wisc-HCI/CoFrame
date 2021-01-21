import pprint

from evd_script import *


def generate():

    cache = get_evd_cache_obj()

    #===========================================================================
    # Define Program
    #===========================================================================
    prog = Program()

    # Define and add location
    home_loc = Location(Position(0,0.2,0),Orientation.Identity(),name='home')
    pick_stock_loc = Location(Position(0.1,0.3,0),Orientation.Identity(), name='pick stock')
    place_stock_loc = Location(Position(0.4,0.4,0.2),Orientation.Identity(), name='place stock')
    retract_loc = Location(Position(0.3,0.3,0.4),Orientation.Identity(), name='retract')
    pick_final_loc = Location(Position(0.4,0.4,0.2),Orientation.Identity(), name='pick_final')
    place_final_loc = Location(Position(-0.1,-0.1,-0.1),Orientation.Identity(), name='place_final')

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
        trace = Trace('ee',{'ee': [
            TraceDataPoint(home_loc.position,home_loc.orientation),
            TraceDataPoint(retract_loc.position,retract_loc.orientation),
            TraceDataPoint(place_final_loc.position,place_final_loc.orientation)]})

        waypoints = [
            Waypoint(retract_loc.position,retract_loc.orientation)
        ]

        cache.get(trajUuid,'trajectory').waypoints = waypoints
        cache.get(trajUuid,'trajectory').trace = trace

    #print '\n\n\n\n'
    #print 'Cache Log'
    #pprint.pprint(cache.utility_cache_stats())

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
