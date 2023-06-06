import { programType } from './program';
import { locationType, waypointType } from './pose';
import { thingType } from './thing';
import { trajectoryType } from './trajectory';
import { hierarchicalType } from './hierarchical';
import { skillType } from './skill';
import { meshType } from './mesh';
import { processType } from './process';
import {inputOutputType} from './inputOutput';
import {graspPointType} from './graspPoint';
import actionTypes from './action';
import agentTypes from './agents';
import sceneObjects from './sceneObjects';
import collisionTypes from './collision';
import { goalType } from './goal';
import { goalProgramType } from './goalProgram';

const mod = { 
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
    graspPointType,
    goalType,
    ...actionTypes,
    ...agentTypes,
    ...sceneObjects,
    ...collisionTypes,
    goalProgramType
}

export default mod;
