import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const sceneBase = {
    name: 'base',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
        mesh: {
            name: 'Mesh',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
        },
        tf: {
            name: 'Frame',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
        },
        showCollision: {
            name: 'Show Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: false,
            isList: false,
            fullWidth: true
        },
        showName: {
            name: 'Show Name',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: false,
            isList: false,
            fullWidth: true
        },
        highlighted: {
            name: 'Highlighted',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: false,
            isList: false,
            fullWidth: true
        },
        status: {
            name: 'Status',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: STATUS.PENDING
        },
        computeSteps: {
            name: 'Compute Steps',
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