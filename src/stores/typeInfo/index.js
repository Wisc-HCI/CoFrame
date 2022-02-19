import { programType } from './program';
import { machineType } from './machine';
import { locationType } from './location';
import { waypointType } from './waypoint';
import { thingType } from './thing';
import { trajectoryType } from './trajectory';
import { hierarchicalType } from './hierarchical';
import { skillType } from './skill';
import { fixtureType } from './fixture';
import { linkType } from './link';
import { meshType } from './mesh';
import { tfType } from './tf';
import { processType } from './process';
import { zoneType } from './zone';
import actionTypes from './action';
import agentTypes from './agents';

export default { 
    programType, 
    machineType, 
    locationType,
    waypointType,
    thingType,
    trajectoryType,
    hierarchicalType,
    skillType,
    fixtureType,
    linkType,
    meshType,
    tfType,
    processType,
    zoneType,
    ...actionTypes,
    ...agentTypes
}
