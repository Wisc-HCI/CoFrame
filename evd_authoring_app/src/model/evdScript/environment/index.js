import { Environment } from './environment';
import { CollisionMesh } from './collisionMesh';
import { OccupancyZone } from './occupancyZone';
import { PinchPoint } from './pinchPoint';
import { ReachSphere } from './reachSphere';


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