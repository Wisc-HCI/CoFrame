import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const tfType = {
    name: 'Transform Frame',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
        position: {
            name: 'Position',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {x:0, y:0, z:0},
            isList: false,
            fullWidth: true
        },
        rotation: {
            name: 'Rotation',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {w:0, x:0, y:0, z:0},
            isList: false,
            fullWidth: true
        },
        frame: {
            name: 'Frame',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
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