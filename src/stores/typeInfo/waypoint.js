import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { WaypointIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

export const waypointType = {
    name: 'Waypoint',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#AD1FDE",
      icon: WaypointIconStyled,
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
      position: {
        name: 'Position',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: {x: null, y: null, z: null},
        isList: false,
        fullWidth: true
      },
      rotation: {
        name: 'Rotation',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {x: null, y: null, z: null, w: null},
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
        default: COMPILE_FUNCTIONS.POSE
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: ['position','rotation']
      }
    }
  }