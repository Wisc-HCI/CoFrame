import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { LocationIconStyled, statusIcon } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { COMPILE_FUNCTIONS } from "../Constants";
import './rotate.css'
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const locationDoc = "Locations are meaningful positions in the scene. For example, they can be used to define goals for placing or picking up [Things](thingType), or specifying starting or ending positions for the [Robot](robotAgentType). [Waypoints](waypointType) can be used in [Trajectories](trajectoryType) to specify intermediates between pairs of locations.";

const locationFeatures = {
  name: 'Location',
  type: TYPES.OBJECT,
  description: locationDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#8624E0",
    icon: LocationIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      { 
        type: EXTRA_TYPES.INDICATOR_ICON,
        accessor: statusIcon,
        label: 'Status'
      },
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.NAME_EDIT_TOGGLE,
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DOC_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE
        ]
      }
    ]
  },
  properties: {
    position: {
      name: 'Position',
      type: SIMPLE_PROPERTY_TYPES.IGNORED, 
      default: {x: 0, y: 0, z: 0},
      isList: false,
      fullWidth: true
    },
    rotation: {
      name: 'Rotation',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {x: 0, y: 0, z: 0, w: 1},
      isList: false,
      fullWidth: true
    },
    states: {
      name: 'States',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    },
    reachability: {
      name: 'Reachability',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.POSE
    },
    updateFields: {
      default: ['position','rotation']
    },
    singleton: {
      default: true
    }
  }
}

export const locationType = merge(locationFeatures, baseTypeData);