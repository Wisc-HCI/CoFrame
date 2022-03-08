import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { PrimitiveIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { merge } from 'lodash';
import { STATUS, STEP_CALCULATOR } from "../Constants";

const basicActionData = {
  type: TYPES.OBJECT,
  instanceBlock: {
    hideNewPrefix: true,
    onCanvas: false,
    color: "#629e6c",
    icon: PrimitiveIconStyled,
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
  referenceBlock: null,
  properties: {
    description: {
      name: 'Description',
      type: SIMPLE_PROPERTY_TYPES.IGNORED, 
      isList: false,
      fullWidth: true
    },
    status: {
      name: 'Status',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING
    },
    computeSteps: {
      name: 'Compute Steps',
      type: SIMPLE_PROPERTY_TYPES.IGNORED
    },
    steps: {
      name: 'Steps',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    }
  }
}

const delayFeatures = {
  name: 'Delay',
  properties: {
    description: {default: 'Delay action for a specified amount of time'},
    duration: {
      name: 'Duration (sec)',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 1000,
      min: 0,
      max: Infinity,
      visualScaling: 1/1000
    },
    computeSteps: {default:STEP_CALCULATOR.DELAY}
  }
}

const breakpointFeatures = {
  name: 'Breakpoint',
  properties: {
    description: {default: 'Stop computation and processing here'},
    computeSteps: {default:STEP_CALCULATOR.NULL}
  }
}

const gripperFeatures = {
  name: 'Gripper',
  properties: {
    description: {default: 'Adjust the gripper position'},
    thing: {
      name: "Thing",
      accepts: ["thingType"],
      default: null,
      isList: false,
      nullValid: true
    },
    position: {
      name: 'Position',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 0,
      max: 85
    },
    speed: {
      name: 'Speed',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 20,
      max: 150
    },
    computeSteps: {default:STEP_CALCULATOR.GRIPPER}
  }
}

const machineInitFeatures = {
  name: 'Machine Initialize',
  properties: {
    description: {default: 'Initialize a machine for use'},
    machine: {
      name: "Machine",
      accepts: ["machineType"],
      default: null,
      isList: false
    },
    computeSteps: {default:STEP_CALCULATOR.MACHINE}
  }
}

const processStartFeatures = {
  name: 'Process Start',
  properties: {
    description: {default: 'Begin a machine process'},
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false
    },
    machine: {
      name: "Machine",
      accepts: ["machineType"],
      default: null,
      isList: false,
      nullValid: true
    },
    computeSteps: {default:STEP_CALCULATOR.PROCESS}
  }
}

const processStopFeatures = {
  name: 'Process Stop',
  properties: {
    description: {default: 'Mark completion of a process and enable retrieval of results'},
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false
    },
    machine: {
      name: "Machine",
      accepts: ["machineType"],
      default: null,
      isList: false,
      nullValid: true
    },
    computeSteps: {default:STEP_CALCULATOR.PROCESS}
  }
}

const processWaitFeatures = {
  name: 'Process Wait',
  properties: {
    description: {default: 'Fill any remaining time while process is running by making the robot wait'},
    process: {
      name: "Process",
      accepts: ["processType"],
      default: null,
      isList: false
    },
    machine: {
      name: "Machine",
      accepts: ["machineType"],
      default: null,
      isList: false,
      nullValid: true
    },
    computeSteps: {default:STEP_CALCULATOR.PROCESS}
  }
}

const moveTrajectoryFeatures = {
  name: 'Move Trajectory',
  properties: {
    description: {default: 'Move Robot according to a trajectory and motion type'},
    trajectory: {
      name: "Trajectory",
      accepts: ["trajectoryType"],
      default: null,
      isList: false
    },
    velocity: {
      name: 'Velocity',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 1,
      min: 0.01,
      max: 5
    },
    motionType: {
      name: 'Motion Type',
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: ['IK', 'Joint'],
      default: 'IK'
    },
    computeSteps: {default:STEP_CALCULATOR.ROBOT_MOTION}
  }
}

const moveUnplannedFeatures = {
  name: 'Move Unplanned',
  properties: {
    description: {default: 'Instantly Move Robot (initialization only)'},
    location: {
      name: "To Location",
      accepts: ["locationType"],
      default: null,
      isList: false
    },
    computeSteps: {default:STEP_CALCULATOR.ROBOT_MOTION}
  }
}

const actionTypes = {
  delayType: merge(delayFeatures,basicActionData),
  breakpointType: merge(breakpointFeatures,basicActionData),
  gripperType: merge(gripperFeatures,basicActionData),
  machineInitType: merge(machineInitFeatures,basicActionData),
  processStartType: merge(processStartFeatures,basicActionData),
  processStopType: merge(processStopFeatures,basicActionData),
  processWaitType: merge(processWaitFeatures,basicActionData),
  moveTrajectoryType: merge(moveTrajectoryFeatures,basicActionData),
  moveUnplannedType: merge(moveUnplannedFeatures,basicActionData)
}

console.log(actionTypes)

export default actionTypes