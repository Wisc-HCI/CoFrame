import { sceneBase } from "./sceneBase";
import { SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const linkType = {
    ...sceneBase,
    name: 'Link',
    properties: {
        robot: {
            name: 'Robot Ref',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: '',
            isList: false,
            fullWidth: true
        },
        collision: {
            name: 'Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: '',
            isList: false,
            fullWidth: true
        },
        status: {
          name: 'Status',
          type: SIMPLE_PROPERTY_TYPES.IGNORED,
          default: STATUS.PENDING
        },
        computeSteps: {
          name: 'Compute Trace',
          type: SIMPLE_PROPERTY_TYPES.IGNORED,
          default: STEP_CALCULATOR.NULL
        },
        steps: {
            name: 'Steps',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: {}
          }
    }
}