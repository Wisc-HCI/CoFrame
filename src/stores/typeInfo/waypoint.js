import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { WaypointIconStyled, statusIcon } from "./icons";
import { FiMoreHorizontal, FiAlertTriangle, FiAlertOctagon, FiRefreshCw, FiThumbsUp } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import './rotate.css'
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const waypointDoc = "Waypoints are positions and orientations that are used as parts of [Trajectories](trajectoryType), and unlike [Locations](locationType), do not have inherent meaning other than to allow greater specificity of the manner with which the [Robot](robotAgentType) moves between a pair of locations."

const waypointFeatures = {
    name: 'Waypoint',
    type: TYPES.OBJECT,
    description: waypointDoc,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#AD1FDE",
      icon: WaypointIconStyled,
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

export const waypointType = merge(waypointFeatures, baseTypeData);