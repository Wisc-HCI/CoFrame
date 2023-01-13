import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { PrimitiveIconStyled, statusIcon } from "./icons";
import {
  FiMoreHorizontal
} from "react-icons/fi";
import { merge } from "lodash";
import { COMPILE_FUNCTIONS } from "../Constants";
import "./rotate.css";
import { baseTypeData } from "./baseType";

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
        accessor: statusIcon,
        label: "Status",
      },
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DOC_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
          {
            type: EXTRA_TYPES.INDICATOR_ICON,
            accessor: statusIcon,
            label: "Status",
          },
        ],
      },
    ],
  },
  referenceBlock: null,
  ...baseTypeData
};

const delayFeatures = {
  name: "Delay",
  description: "A pause by the [Robot](robotAgentType) by a specified amount of time.",
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
  description: "A terminator that prematurely ends the compilation of the [Program](programType).",
  properties: {
    description: { default: "Stop computation and processing here" },
    compileFn: { default: COMPILE_FUNCTIONS.BREAK },
    updateFields: { default: [] },
  },
};

const gripperFeatures = {
  name: "Move Gripper",
  description: "An action by the [Robot](robotAgentType) that adjusts the distance between the two fingers of the gripper. If interacting with a [Thing](thingType) or [Tool](toolType), it should be specified in the action.",
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
  description: "An action that initializes the [Machine](machineType) for usage. This need only be done once per execution of the [Program](programType).",
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
  description: "An action that begins a process [Process](processType). If the process needs a [Machine](machineType) or [Tool](toolType), this must also be provided in the 'gizmo' field.",
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

const processWaitFeatures = {
  name: "Process Wait",
  description: "An action that has the [Robot](robotAgentType) wait until the completion of the process [Process](processType). If the process needs a [Machine](machineType) or [Tool](toolType), this must also be provided in the 'gizmo' field.",
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
  description: "An action by the [Robot](robotAgentType) that moves its gripper along a specified [Trajectory](trajectoryType). Motion types include 'IK', which attempts to move the gripper in a straight line from one location to another, while 'Joint' interpolates the joints. Velocity adjusts the speed of the motion.",
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
      options: [{value: "IK", label: "IK"}, {value: "Joint", label:'Joint'}],
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
