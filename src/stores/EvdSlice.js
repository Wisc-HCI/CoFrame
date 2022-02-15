import { arrayMove, deleteAction } from './helpers';
import lodash from 'lodash';
import {urdf} from "./robot";
import { FiClipboard, FiBriefcase, FiGrid, FiBox, FiLogOut, FiMoreHorizontal, FiLayers, FiFeather } from "react-icons/fi";
import { DATA_TYPES, TYPES, EXTRA_TYPES, SIMPLE_PROPERTY_TYPES } from 'simple-vp';

import typeInfo from './typeInfo';
import { performPoseProcess } from './planner-worker';

import * as Comlink from 'comlink';
/* eslint-disable import/no-webpack-loader-syntax */
import Worker from 'worker-loader!./planner-worker';

import {
  LocationIconStyled,
  PrimitiveIconStyled,
  MachineIconStyled,
  SkillIconStyled,
  ThingIconStyled,
  WaypointIconStyled,
  ContainerIconStyled
} from './typeInfo/icons';

export const EvdSlice = (set, get) => ({
  solver: null,
  programSpec: {
    drawers: [
      // Icon is FiGrid, otherwise no icons show in the drawer
      { title: "Machines", dataType: DATA_TYPES.REFERENCE, objectType: 'machineType', icon: MachineIconStyled },
      { title: "Locations", dataType: DATA_TYPES.REFERENCE, objectType: 'locationType', icon: LocationIconStyled },
      { title: "Waypoints", dataType: DATA_TYPES.REFERENCE, objectType: 'waypointType', icon: WaypointIconStyled },
      { title: "Things", dataType: DATA_TYPES.REFERENCE, objectType: 'thingType', icon: ThingIconStyled },
      { title: "Containers", dataType: DATA_TYPES.INSTANCE, objectTypes: ['trajectoryType', 'hierarchicalType', 'skillType'], icon: ContainerIconStyled },
      { title: "Skills", dataType: DATA_TYPES.CALL, objectType: 'skillType', icon: SkillIconStyled },
      { title: "Actions", dataType: DATA_TYPES.INSTANCE, objectTypes: ['delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'], icon: PrimitiveIconStyled }
    ],
    objectTypes: {
      ...typeInfo,
      delayType: {
        name: 'Delay',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          duration: {
            name: 'Duration',
            type: SIMPLE_PROPERTY_TYPES.NUMBER,
            default: 1,
            min: 0,
            max: 5
          }
        }
      },
      breakpointType: {
        name: 'Breakpoint',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          }
        }
      },
      gripperType: {
        name: 'Gripper',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: "Thing",
            accepts: ["thingType"],
            default: null,
            isList: false
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
          }
        }
      },
      machineInitType: {
        name: 'Machine Initialization',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
          
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: "Machine",
            accepts: ["machineType"],
            default: null,
            isList: false
          }
        }
      },
      processStartType: {
        name: 'Process Start',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: "Machine",
            accepts: ["machineType"],
            default: null,
            isList: false
          }
        }
      },
      processStopType: {
        name: 'Process Stop',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: "Machine",
            accepts: ["machineType"],
            default: null,
            isList: false
          }
        }
      },
      processWaitType: {
        name: 'Process Wait',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: "Machine",
            accepts: ["machineType"],
            default: null,
            isList: false
          }
        }
      },
      moveTrajectoryType: {
        name: 'Move Trajectory',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: false
          },
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
          }
        }
      },
      moveUnplannedType: {
        name: 'Move Unplanned',
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          location: {
            name: "To Location",
            accepts: ["locationType"],
            default: null,
            isList: false
          }
        }
      },
      robotAgentType: {
        name: 'robot-agent',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          }
        }
      },
      processType: {
        name: 'process',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          machine: {
            name: 'Machine',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: "",
            isList: false,
            fullWidth: true
          },
          processTime: {
            name: 'Process Time',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: 0,
            isList: false,
            fullWidth: true
          },
          outputs: {
            name: 'Outputs',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: [],
            isList: true,
            fullWidth: true
          },
          inputs: {
            name: 'Inputs',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      zoneType: {
        name: 'zone',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          agent: {
            name: 'Agent ID',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          scale: {
            name: 'Scale',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: {x: null, y: null, z: null},
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
          orientation: {
            name: 'Orientation',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: {x: null, y: null, z: null, w: null},
            isList: false,
            fullWidth: true
          }
        }
      },
      humanAgentType: {
        name: 'human-agent',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          }
        }
      },
    }
  },
  programData: {},
  // All the old stuff below
  data: {
    "program-484de43e-adaa-4801-a23b-bca38e211365":{
      "name": "Knife Assembly",
      "editable": true,
      "deleteable": false,
      "description": "The top-level program",
      "parameters": {},
      "children": [],
      "transform": {"x": 0, "y": 0}
    }
  },
  // A macro for updating the entire program from raw data
  setData: (data) => set((_) => ({ programData: data})),
  updatePoseJoints: (id,value,process) => set(state=>{
    state.programData[id].joints = value;
    state.processes[id] = process;
  }),
  performProcessStupid: (id) => {
    console.log('starting stupidly ',id);
    const result = performPoseProcess({urdf, pose:get().programData[id], scene:{}});
    console.log('process finished', id);
    console.log(result)
  },
  performPoseProcess: async (id) => {
    console.log('starting ',id)
    const currentProcess = get().processes[id];
    if (currentProcess) {
        console.log('terminating process for ',id)
        currentProcess.terminate();
    }
    const workerInstance = new Worker();
    get().updatePoseJoints(id,null,workerInstance);
    const workerLib = Comlink.wrap(workerInstance);
    const result = await workerLib.performPoseProcess({urdf, pose:get().programData[id], scene:{}});
    get().updatePoseJoints(id,result,null);
  },
  processes: {}
});