import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ThingIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

export const thingType = {
    name: 'Thing',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#E08024",
      icon: ThingIconStyled,
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
      safe: {
        name: 'Safe',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: false,
        isList: false,
        fullWidth: true
      },
      weight: {
        name: 'Weight',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: 0,
        isList: false,
        fullWidth: true
      },
      mesh: {
        name: 'Mesh',
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
        default: ['safe','weight']
      }
    }
  }