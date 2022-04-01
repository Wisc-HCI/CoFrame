import { programType } from './program';
import { locationType } from './location';
import { waypointType } from './waypoint';
import { thingType } from './thing';
import { trajectoryType } from './trajectory';
import { hierarchicalType } from './hierarchical';
import { skillType } from './skill';
import { meshType } from './mesh';
import { processType } from './process';
import {inputOutputType} from './inputOutput'
import actionTypes from './action';
import agentTypes from './agents';
import sceneObjects from './sceneObjects';
import collisionTypes from './collision';


export default { 
    inputOutputType,
    programType,
    locationType,
    waypointType,
    thingType,
    trajectoryType,
    hierarchicalType,
    skillType,
    meshType,
    processType,
    ...actionTypes,
    ...agentTypes,
    ...sceneObjects,
    ...collisionTypes
}
