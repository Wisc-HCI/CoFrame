import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { PrimitiveIconStyled } from "./icons";
import {
  FiMoreHorizontal,
  FiAlertTriangle,
  FiAlertOctagon,
  FiRefreshCw,
  FiThumbsUp,
} from "react-icons/fi";
import { merge } from "lodash";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import "./rotate.css";

const basicActionData = {
  type: TYPES.OBJECT,
  instanceBlock: {
    hideNewPrefix: true,
    onCanvas: false,
    color: "#629e6c",
    icon: PrimitiveIconStyled,
    extras: [
      // EXTRA_TYPES.LOCKED_INDICATOR,
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
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
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
        ],
      },
    ],
  },
  referenceBlock: null,
  properties: {
    description: {
      name: "Description",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      isList: false,
      fullWidth: true,
    },
    status: {
      name: "Status",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING,
    },
    compileFn: {
      name: "Compile Function",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
    },
    compiled: {
      name: "Compiled",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {},
    },
    updateFields: {
      name: "Update Fields",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
    },
    singleton: {
      name: "singleton",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: false,
    },
  },
};

const delayFeatures = {
  name: "Delay",
  properties: {
    description: { default: "Delay action for a specified amount of time" },
    duration: {
      name: "Duration",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 1000,
      min: 0,
      max: 3600000,
      step: 10,
      visualScaling: 1 / 1000,
      visualPrecision: 1,
      units: "sec",
    },
    compileFn: { default: COMPILE_FUNCTIONS.DELAY },
    updateFields: { default: ["duration"] },
  },
};

const breakpointFeatures = {
  name: "Breakpoint",
  properties: {
    description: { default: "Stop computation and processing here" },
    compileFn: { default: COMPILE_FUNCTIONS.BREAK },
    updateFields: { default: [] },
  },
};

const gripperFeatures = {
  name: "Move Gripper",
  properties: {
    description: { default: "Adjust the gripper position" },
    thing: {
      name: "Thing",
      accepts: ["thingType", "toolType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    positionStart: {
      name: "Start Position",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 0,
      max: 85,
    },
    positionEnd: {
      name: "End Position",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 0,
      max: 85,
    },
    speed: {
      name: "Speed",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 20,
      min: 1,
      max: 50,
      step: 1,
      visualScaling: 1,
      visualPrecision: 0,
      units: "mm/s",
    },
    compileFn: { default: COMPILE_FUNCTIONS.GRIPPER_MOTION },
    updateFields: {
      default: ["thing", "positionStart", "positionEnd", "speed"],
    },
  },
};

const machineInitFeatures = {
  name: "Machine Initialize",
  properties: {
    description: { default: "Initialize a machine for use" },
    machine: {
      name: "Machine",
      accepts: ["machineType"],
      default: null,
      isList: false,
    },
    compileFn: { default: COMPILE_FUNCTIONS.MACHINE },
    updateFields: { default: ["machine"] },
  },
};

const processStartFeatures = {
  name: "Process Start",
  properties: {
    description: { default: "Begin a machine process" },
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false,
    },
    gizmo: {
      name: "Gizmo",
      accepts: ["machineType","toolType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    compileFn: { default: COMPILE_FUNCTIONS.PROCESS },
    updateFields: { default: ["process", "gizmo"] },
  },
};

const processStopFeatures = {
  name: "Process Stop",
  properties: {
    description: {
      default: "Mark completion of a process and enable retrieval of results",
    },
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false,
    },
    gizmo: {
      name: "Gizmo",
      accepts: ["machineType","toolType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    compileFn: { default: COMPILE_FUNCTIONS.PROCESS },
    updateFields: { default: ["process", "gizmo"] },
  },
};

const processWaitFeatures = {
  name: "Process Wait",
  properties: {
    description: {
      default:
        "Fill any remaining time while process is running by making the robot wait",
    },
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false,
    },
    gizmo: {
      name: "Gizmo",
      accepts: ["machineType","toolType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    compileFn: { default: COMPILE_FUNCTIONS.PROCESS },
    updateFields: { default: ["process", "gizmo"] },
  },
};

const moveTrajectoryFeatures = {
  name: "Move Trajectory",
  properties: {
    description: {
      default: "Move Robot according to a trajectory and motion type",
    },
    trajectory: {
      name: "Trajectory",
      accepts: ["trajectoryType"],
      default: null,
      isList: false,
    },
    velocity: {
      // mm/ms or m/s
      name: "Velocity",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 0.5,
      min: 0.01,
      max: 5,
      step: 0.01,
      visualScaling: 1,
      visualPrecision: 2,
      units: "m/s",
    },
    motionType: {
      name: "Motion Type",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: ["IK", "Joint"],
      default: "IK",
    },
    compileFn: { default: COMPILE_FUNCTIONS.ROBOT_MOTION },
    updateFields: { default: ["trajectory", "velocity", "motionType"] },
  },
};

// const moveUnplannedFeatures = {
//   name: 'Move Unplanned',
//   properties: {
//     description: {default: 'Instantly Move Robot (initialization only)'},
//     location: {
//       name: "To Location",
//       accepts: ["locationType"],
//       default: null,
//       isList: false
//     },
//     compileFn: {default:COMPILE_FUNCTIONS.ROBOT_MOTION}
//   }
// }

const actionTypes = {
  delayType: merge(delayFeatures, basicActionData),
  moveGripperType: merge(gripperFeatures, basicActionData),
  machineInitType: merge(machineInitFeatures, basicActionData),
  processStartType: merge(processStartFeatures, basicActionData),
  // processStopType: merge(processStopFeatures, basicActionData),
  processWaitType: merge(processWaitFeatures, basicActionData),
  moveTrajectoryType: merge(moveTrajectoryFeatures, basicActionData),
  // moveUnplannedType: merge(moveUnplannedFeatures,basicActionData),
  breakpointType: merge(breakpointFeatures, basicActionData, {
  instanceBlock: { color: "#3a5e40" },
  }),
};

console.log(actionTypes);

export default actionTypes;
