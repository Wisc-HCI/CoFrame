import { Environment } from './environment';
import { CollisionMesh } from './collision_mesh';
import { OccupancyZone } from './occupancy_zone';
import { PinchPoint } from './pinch_point';
import { ReachSphere } from './reach_sphere';


const EnvironmentNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'reach-sphere':
            node = ReachSphere.fromDict(dct);
            break;
        case 'pinch-point':
            node = PinchPoint.fromDict(dct);
            break;
        case 'collision-mesh':
            node = CollisionMesh.fromDict(dct);
            break;
        case 'occupancy-zone':
            node = OccupancyZone.fromDict(dct);
            break;
        case 'environment':
            node = Environment.fromDict(dct);
            break;
    }

    return node;
};

export {
    EnvironmentNodeParser,
    Environment,
    CollisionMesh,
    OccupancyZone,
    PinchPoint,
    ReachSphere
};