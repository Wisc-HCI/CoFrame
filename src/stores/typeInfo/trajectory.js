import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled, statusIcon } from "./icons";
import {
  FiMoreHorizontal,
  FiAlertTriangle,
  FiAlertOctagon,
  FiRefreshCw,
  FiThumbsUp,
} from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import "./rotate.css";
import { baseIndicatorLabelFn, baseTypeData } from "./baseType";
import { merge } from "lodash";

const trajectoryDoc = 'Trajectories are a collection of two [Locations](locationType), and a set of optional intermediate [Waypoints](waypointType). They are used within the [Move Trajectory](moveTrajectoryType) action to move a [Robot](robotAgentType) from one location to another.';

const trajectoryFeatures = {
  name: "Trajectory",
  type: TYPES.OBJECT,
  description: trajectoryDoc,
  instanceBlock: {
    hideNewPrefix: true,
    onCanvas: false,
    color: "#c5329a",
    icon: ContainerIconStyled,
    extras: [
      
      {
        type: EXTRA_TYPES.INDICATOR_ICON,
        
        accessor: statusIcon,
        label: baseIndicatorLabelFn,
      },
      // EXTRA_TYPES.LOCKED_INDICATOR,
      EXTRA_TYPES.NAME_EDIT_TOGGLE,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DOC_TOGGLE,
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