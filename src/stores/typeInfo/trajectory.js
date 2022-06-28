import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import {
  FiMoreHorizontal,
  FiAlertTriangle,
  FiAlertOctagon,
  FiRefreshCw,
  FiThumbsUp,
} from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import "./rotate.css";
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const trajectoryFeatures = {
  name: "Trajectory",
  type: TYPES.OBJECT,
  instanceBlock: {
    hideNewPrefix: true,
    onCanvas: false,
    color: "#c5329a",
    icon: ContainerIconStyled,
    extras: [
      
      {
        type: EXTRA_TYPES.INDICATOR_ICON,
        
        accessor: (data) => {
          if (data.properties.status === STATUS.FAILED) {
            return <FiAlertOctagon color="white" fill="red" />;
          } else if (data.properties.status === STATUS.VALID) {
            return <FiThumbsUp color="white" />;
          } else if (data.properties.status === STATUS.WARN) {
            return <FiAlertTriangle color="white" fill="#ff7300" />;
          } else if (data.properties.status === STATUS.PENDING) {
            return <FiRefreshCw className="rotate" />;
          }
        },
        label: "Status",
      },
      EXTRA_TYPES.LOCKED_INDICATOR,
      EXTRA_TYPES.NAME_EDIT_TOGGLE,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
        ],
      },
    ],
  },
  referenceBlock: {
    hideNewPrefix: true,
    onCanvas: false,
    color: "#c5329a",
    icon: ContainerIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
        ],
      },
    ],
  },
  properties: {
    startLocation: {
      name: "Start Location",
      accepts: ["locationType"],
      default: null,
      isList: false,
    },
    waypoints: {
      name: "Waypoints",
      accepts: ["waypointType"],
      default: [],
      isList: true,
    },
    endLocation: {
      name: "End Location",
      accepts: ["locationType"],
      default: null,
      isList: false,
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.PROPERTY,
    },
    updateFields: {
      default: ["startLocation", "waypoints", "endLocation"],
    },
  },
};

export const trajectoryType = merge(trajectoryFeatures, baseTypeData);