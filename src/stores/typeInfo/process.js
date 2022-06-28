import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import { ProcessIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";

export const processType = {
    name: 'Process',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#a83832",
      icon: ProcessIconStyled,
      extras: [
        EXTRA_TYPES.LOCKED_INDICATOR,
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.DELETE_BUTTON,
            EXTRA_TYPES.DEBUG_TOGGLE,
            EXTRA_TYPES.SELECTION_TOGGLE
          ]
        }
      ]
    },
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
      },
      gizmo: {
        name: 'Gizmo',
        accepts: ['machineType','toolType'],
        default: null,
        isList: false,
        fullWidth: false,
        nullValid: true
      },
      processTime: {
        name: 'Duration',
        type: SIMPLE_PROPERTY_TYPES.NUMBER,
        default: 0,
        min: 0,
        max: Infinity,
        visualScaling: 1/1000
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
      compileFn: {
        name: 'Compile Function',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: COMPILE_FUNCTIONS.PROPERTY
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: ['gizmo','processTime','inputs','outputs']
      },
      singleton: {
        name: 'singleton',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: true
      }
    }
  }
