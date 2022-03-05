import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";
import { ProcessIconStyled } from "./icons";

export const processType = {
    name: 'Process',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#a83832",
      icon: ProcessIconStyled,
    },
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
      },
      machine: {
        name: 'Machine',
        accepts: ['machineType'],
        default: null,
        isList: false,
        fullWidth: false,
        nullValid: true
      },
      processTime: { 
        name: 'Process Time',
        name: 'Duration',
        type: SIMPLE_PROPERTY_TYPES.NUMBER,
        default: 0,
        min: 0,
        max: null
      },
      inputs: {
        name: 'Inputs',
        default: [],
        accepts: ["inputOutputType"],
        isList: true,
        fullWidth: false
      },
      outputs: {
        name: 'Outputs',
        default: [],
        accepts: ["inputOutputType"],
        isList: true,
        fullWidth: false
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
