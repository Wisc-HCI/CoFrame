from cobots_model.data.trace import *
from cobots_model.program.task import *
from cobots_model.data.machine import *
from cobots_model.data.waypoint import *
from cobots_model.data.location import *
from cobots_model.data.geometry import *
from cobots_model.program.program import *
from cobots_model.program.primitive import *

def generate():
    # Define Program
    prog = Program()

    # Define and add location
    home_loc = Location(Position(0,0.2,0),Orientation.Identity())
    pick_stock_loc = Location(Position(0.1,0.3,0),Orientation.Identity())
    place_stock_loc = Location(Position(0.4,0.4,0.2),Orientation.Identity())
    retract_loc = Location(Position(0.3,0.3,0.4),Orientation.Identity())
    pick_final_loc = Location(Position(0.4,0.4,0.2),Orientation.Identity())
    place_final_loc = Location(Position(-0.1,-0.1,-0.1),Orientation.Identity())

    prog.context.add_location(home_loc)
    prog.context.add_location(pick_stock_loc)
    prog.context.add_location(place_stock_loc)
    prog.context.add_location(retract_loc)
    prog.context.add_location(pick_final_loc)
    prog.context.add_location(place_final_loc)

    # Define and add machines
    cnc = Machine()

    prog.context.add_machine(cnc)

    # Define and add primitives
    prog.add_primitive(Initialize(
        homeLocUuid=home_loc.uuid,
        machineUuid=cnc.uuid))

    prog.add_primitive(Loop(
        primitives=[
            PickAndPlace(
                startLocUuid=home_loc.uuid,
                pickLocUuid=pick_stock_loc.uuid,
                placeLocUuid=place_stock_loc.uuid),
            MoveTrajectory(
                startLocUuid=place_stock_loc.uuid,
                endLocUuid=retract_loc.uuid),
            MachineBlockingProcess(
                machineUuid=cnc.uuid),
            PickAndPlace(
                startLocUuid=retract_loc.uuid,
                pickLocUuid=pick_final_loc.uuid,
                placeLocUuid=place_final_loc.uuid)
        ]
    ))

    for trajUuid in prog.cache.trajectories.keys():
        trace = Trace('ee',{'ee': [
            TraceDataPoint(home_loc.position,home_loc.orientation),
            TraceDataPoint(retract_loc.position,retract_loc.orientation),
            TraceDataPoint(place_final_loc.position,place_final_loc.orientation)]})

        waypoints = [
            Waypoint(retract_loc.position,retract_loc.orientation)
        ]

        prog.cache.get(trajUuid,'trajectory').waypoints = waypoints
        prog.cache.get(trajUuid,'trajectory').trace = trace

    return prog
