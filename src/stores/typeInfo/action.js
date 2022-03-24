import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { PrimitiveIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { merge } from 'lodash';
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

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
    compileFn: {
      name: 'Compile Function',
      type: SIMPLE_PROPERTY_TYPES.IGNORED
    },
    compiled: {
      name: 'Compiled',
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
    compileFn: {default:COMPILE_FUNCTIONS.DELAY}
  }
}

const breakpointFeatures = {
  name: 'Breakpoint',
  properties: {
    description: {default: 'Stop computation and processing here'},
    compileFn: {default:COMPILE_FUNCTIONS.BREAK}
  }
}

const gripperFeatures = {
  name: 'Gripper',
  properties: {
    description: {default: 'Adjust the gripper position'},
    thing: {
      name: "Thing",
      accepts: ["thingType", "toolType"],
      default: null,
      isList: false,
      nullValid: true
    },
    positionStart: {
      name: 'Start Position',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 0,
      max: 85
    },
    positionEnd: {
      name: 'End Position',
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
    compileFn: {default:COMPILE_FUNCTIONS.GRIPPER}
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
    compileFn: {default:COMPILE_FUNCTIONS.MACHINE}
  }
}

const robotInitFeatures = {
  name: 'Robot Initialize',
  properties: {
    description: {default: 'Initialize the robot for use'},
    initialGripper: {
      name: 'Initial Grip State',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      min: 0,
      max: 85
    },
    initialPosition: {
      name: 'Initial Location',
      accepts: ['locationType'],
      default: null,
      isList: false
    },
    compileFn: {default:COMPILE_FUNCTIONS.AGENT}
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
    compileFn: {default:COMPILE_FUNCTIONS.PROCESS}
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
    compileFn: {default:COMPILE_FUNCTIONS.PROCESS}
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
    compileFn: {default:COMPILE_FUNCTIONS.PROCESS}
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
    compileFn: {default:COMPILE_FUNCTIONS.ROBOT_MOTION}
  }
}

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
  delayType: merge(delayFeatures,basicActionData),
  gripperType: merge(gripperFeatures,basicActionData),
  machineInitType: merge(machineInitFeatures,basicActionData),
  robotInitType: merge(robotInitFeatures,basicActionData),
  processStartType: merge(processStartFeatures,basicActionData),
  processStopType: merge(processStopFeatures,basicActionData),
  processWaitType: merge(processWaitFeatures,basicActionData),
  moveTrajectoryType: merge(moveTrajectoryFeatures,basicActionData),
  // moveUnplannedType: merge(moveUnplannedFeatures,basicActionData),
  breakpointType: merge(breakpointFeatures,basicActionData,{instanceBlock:{color: "#3a5e40"}})
}

console.log(actionTypes)

export default actionTypes