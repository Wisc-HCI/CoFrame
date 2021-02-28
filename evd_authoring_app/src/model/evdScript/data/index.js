import { Pose, Position, Orientation } from './geometry';
import { Location } from './location';
import { Waypoint } from './waypoint';
import { Machine, MachineRecipe } from './machine';
import { Trace, TraceDataPoint } from './trace';
import { Trajectory } from './trajectory';
import { Region, CubeRegion, SphereRegion } from './region';
import { Thing } from './thing';


const DataNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'waypoint':
            node = Waypoint.fromDict(dct);
            break;
        case 'location':
            node = Location.fromDict(dct);
            break;
        case 'trajectory':
            node = Trajectory.fromDict(dct);
            break;
        case 'trace':
            node = Trace.fromDict(dct);
            break;
        case 'trace-data-point':
            node = TraceDataPoint.fromDict(dct);
            break;
        
        case 'pose':
            node = Pose.fromDict(dct);
            break;
        case 'position':
            node = Position.fromDict(dct);
            break;
        case 'orientation':
            node = Orientation.fromDict(dct);
            break;

        case 'machine':
            node = Machine.fromDict(dct);
            break;
        case 'machine-recipe':
            node = MachineRecipe.fromDict(dct);
            break;
        
        case 'thing':
            node = Thing.fromDict(dct);
            break;

        case 'region':
            node = Region.fromDict(dct);
            break;
        case 'cube-region':
            node = CubeRegion.fromDict(dct);
            break;
        case 'sphere-region':
            node = SphereRegion.fromDict(dct);
            break;

        default:
            break;
    }

    return node;
};

export {
    Pose, 
    Position, 
    Orientation,
    Location,
    Waypoint,
    Machine, 
    MachineRecipe,
    Trace, 
    TraceDataPoint,
    Trajectory,
    Region, 
    CubeRegion, 
    SphereRegion,
    Thing,
    DataNodeParser
};