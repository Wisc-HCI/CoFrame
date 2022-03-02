import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const meshType = {
    name: 'Mesh',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
        position: {
            name: 'Local Position',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {x:0, y:0, z:0},
            isList: false,
            fullWidth: true
        },
        rotation: {
            name: 'Local Rotation',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {w:0, x:0, y:0, z:0},
            isList: false,
            fullWidth: true
        },
        scale: {
            name: 'Scale',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {x:1, y:1, z:1},
            isList: false,
            fullWidth: true
        },
        color: {
            name: 'Color',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: undefined,
            isList: false,
            fullWidth: true
        },
        keyword: {
            name: 'Robot Scene Keyword',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: '',
            isList: false,
            fullWidth: true
        },
        extraParams: {
            name: 'Simple Shape Paramaters',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: null,
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