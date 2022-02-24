import { delaySteps } from "./delay";
import { gripperMotionSteps } from "./gripperMotion";
import { machineSteps } from "./machine";
import { nullSteps } from './null';
import { poseSteps } from './pose';
import { processSteps } from './process';
import { robotMotionSteps } from './robotMotion';
import { simpleSteps } from './simple';
import { skillSteps } from './skill';

import { STEP_CALCULATOR } from "../Constants";

const KEY_MAPPING = {
    NULL: nullSteps,
    SIMPLE: simpleSteps,
    MACHINE: machineSteps,
    DELAY: delaySteps,
    PROCESS: processSteps,
    SKILL: skillSteps,
    POSE: poseSteps,
    GRIPPER: gripperMotionSteps,
    ROBOT_MOTION: robotMotionSteps
}
// Ordering corresponds to the values in the STEP_CALCULATOR constant

export const stepProcessors =  Object.keys(STEP_CALCULATOR).map(k=>KEY_MAPPING[k]);