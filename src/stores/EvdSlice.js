import { arrayMove, deleteAction } from './helpers';
import lodash from 'lodash';
import {urdf} from "./robot";
import { FiClipboard, FiBriefcase, FiGrid, FiBox, FiLogOut, FiMoreHorizontal, FiLayers, FiFeather } from "react-icons/fi";
import { DATA_TYPES, TYPES, EXTRA_TYPES, SIMPLE_PROPERTY_TYPES } from 'simple-vp';
import { performPoseProcess } from './planner-worker';

import * as Comlink from 'comlink';
/* eslint-disable import/no-webpack-loader-syntax */
import Worker from 'worker-loader!./planner-worker';

// These icons need to be readjusted for Simple-VP
import { ReactComponent as LocationIcon } from '../components/CustomIcons/Location.svg';
import { ReactComponent as PrimitiveIcon } from '../components/CustomIcons/Primitive.svg';
import { ReactComponent as MachineIcon } from '../components/CustomIcons/Machine.svg';
import { ReactComponent as SkillIcon } from '../components/CustomIcons/Skill.svg';
import { ReactComponent as ThingIcon } from '../components/CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../components/CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../components/CustomIcons/Container.svg';

const LocationIconStyled = ()=><LocationIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const PrimitiveIconStyled = ()=><PrimitiveIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const MachineIconStyled = ()=><MachineIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const SkillIconStyled = ()=><SkillIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const ThingIconStyled = ()=><ThingIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const WaypointIconStyled = ()=><WaypointIcon style={{color:'white',width:18,height:18,fill:'white'}}/>
const ContainerIconStyled = ()=><ContainerIcon style={{color:'white',width:18,height:17,fill:'white'}}/>

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
      programType: {
        name: 'Program',
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: true,
          color: "#3f3f3f",
          icon: ContainerIconStyled,
          extras: [
            { 
              type: EXTRA_TYPES.INDICATOR,
              accessor: (data)=>data.properties.children.length,
              label: 'Size'
            },
            {
              icon: FiMoreHorizontal,
              type: EXTRA_TYPES.DROPDOWN,
              contents: [
                EXTRA_TYPES.NAME_EDIT_TOGGLE,
                EXTRA_TYPES.LOCKED_INDICATOR,
                {
                  icon: FiMoreHorizontal,
                  label: 'More Options',
                  type: EXTRA_TYPES.DROPDOWN,
                  contents: [
                    EXTRA_TYPES.NAME_EDIT_TOGGLE,
                    EXTRA_TYPES.COLLAPSE_TOGGLE,
                    EXTRA_TYPES.LOCKED_INDICATOR,
                    { 
                      type: EXTRA_TYPES.INDICATOR,
                      accessor: (data)=>data.properties.children.length,
                      label: 'Size'
                    },
                    { 
                      type: EXTRA_TYPES.FUNCTION_BUTTON,
                      onClick: 'updateItemBlockColors',
                      label: 'Cycle Color',
                      icon: FiFeather
                    }
                  ]
                }
              ]
            },
            EXTRA_TYPES.LOCKED_INDICATOR
          ]
        },
        referenceBlock: null,
        properties: {
          children: {
            name: 'Children',
            accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      machineType: {
        name: 'Machine',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: {
          onCanvas: false,
          color: "#B3A533",
          icon: MachineIconStyled,
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
        }
      },
      locationType: {
        name: 'Location',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: {
          onCanvas: false,
          color: "#8624E0",
          icon: LocationIconStyled,
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
        }
      },
      waypointType: {
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
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        }
      },
      thingType: {
        name: 'Thing',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: {
          onCanvas: false,
          color: "#E08024",
          icon: ThingIconStyled,
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
        }
      },
      trajectoryType: {
        name: 'Trajectory',
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: true,
          color: '#c5329a',
          icon: ContainerIconStyled,
          extras: [
            EXTRA_TYPES.LOCKED_INDICATOR,
            EXTRA_TYPES.DELETE_BUTTON
          ]
        },
        properties: {
          startLocation: {
            name: "Start Location",
            accepts: ["locationType"],
            default: null,
            isList: false
          },
          waypoints: {
            name: "Waypoints",
            accepts: ["waypointType"],
            default: [],
            isList: true
          },
          endLocation: {
            name: "End Location",
            accepts: ["locationType"],
            default: null,
            isList: false
          }
        }
      },
      hierarchicalType: {
        name: "Hierarchical",
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: false,
          color: '#7f7f7f',
          icon: ContainerIconStyled,
          extras: [
            EXTRA_TYPES.COLLAPSE_TOGGLE,
            { 
              type: EXTRA_TYPES.INDICATOR,
              accessor: (data)=>data.properties.children.length,
              label: 'Size'
            },
            EXTRA_TYPES.LOCKED_INDICATOR
          ]
        },
        referenceBlock: null,
        properties: {
          children: {
            name: 'Children',
            accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      skillType: {
        name: 'Skill',
        type: TYPES.FUNCTION,
        instanceBlock: {
          onCanvas: true,
          color: "#62869e",
          icon: SkillIconStyled,
          extras: [
            EXTRA_TYPES.LOCKED_INDICATOR,
            {
              icon: FiMoreHorizontal,
              type: EXTRA_TYPES.DROPDOWN,
              contents: [
                EXTRA_TYPES.SELECTION_TOGGLE,
                EXTRA_TYPES.DELETE_BUTTON,
                EXTRA_TYPES.LOCKED_INDICATOR,
                EXTRA_TYPES.DEBUG_TOGGLE,
                {
                  type: EXTRA_TYPES.ADD_ARGUMENT_GROUP,
                  allowed: ['machineType', 'locationType', 'thingType', 'trajectoryType']
                }
              ]
            },
            {
              type: EXTRA_TYPES.ADD_ARGUMENT_GROUP,
              allowed: ['machineType', 'locationType', 'thingType', 'trajectoryType']
            }
          ]
        },
        callBlock: {
          onCanvas: false,
          color: "#62869e",
          icon: SkillIconStyled
        },
        properties: {
          children: {
            name: 'Children',
            accepts: ['hierarchicalType', 'skillCallType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      delayType: {
        name: 'Delay',
        type: TYPES.OBJECT,
        instanceBlock: {
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
        referenceBlock: null
      },
      gripperType: {
        name: 'Gripper',
        type: TYPES.OBJECT,
        instanceBlock: {
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
          location: {
            name: "To Location",
            accepts: ["locationType"],
            default: null,
            isList: false
          }
        }
      }
    }
  },
  programData: {
    "45535153s": {
      id: "45535153s",
      name: 'MyProgram',
      type: "programType",
      dataType: DATA_TYPES.INSTANCE,
      properties: {
        children: []
      },
      position: { x: 0, y: 10 },
      canDelete: false,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "655sssefs": {
      id: "655sssefs",
      name: 'MyFunction',
      type: "skillType",
      dataType: DATA_TYPES.INSTANCE,
      arguments: [],
      properties: {
        children: []
      },
      position: { x: 400, y: 10 },
      canDelete: true,
      canEdit: true,
      selected: true,
      editing: false,
      
    },
    "6dewwwwww": {
      id: "6dewwwwww",
      name: 'Good Machine',
      type: "machineType",
      dataType: DATA_TYPES.INSTANCE,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "95g7dc13": {
      id: "95g7dc13",
      name: 'Not Good Machine',
      type: "machineType",
      dataType: DATA_TYPES.INSTANCE,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "83h77t67a": {
      id: "83h77t67a",
      name: 'Some Location 1',
      type: "locationType",
      dataType: DATA_TYPES.INSTANCE,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "83h77t67b": {
      id: "83h77t67b",
      name: 'Some Location 2',
      type: "locationType",
      dataType: DATA_TYPES.INSTANCE,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    }
  },
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
  setData: (data) => set((_) => ({ data: data})),
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