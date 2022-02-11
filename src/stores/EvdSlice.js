import { arrayMove, deleteAction } from './helpers';
import lodash from 'lodash';
import {urdf} from "./robot";
import { FiClipboard, FiBriefcase, FiGrid, FiBox, FiLogOut, FiMoreHorizontal, FiLayers, FiFeather } from "react-icons/fi";
import { DATA_TYPES, TYPES, EXTRA_TYPES, SIMPLE_PROPERTY_TYPES } from 'simple-vp';

export const EvdSlice = (set, get) => ({
  solver: null,
  // Ported over the example code from the sandbox
  programSpec: {
    drawers: [
      { title: "Structures", dataType: DATA_TYPES.INSTANCE, objectTypes: ["functionType", "operationType", "blockType"], icon: FiClipboard },
      { title: "Functions", dataType: DATA_TYPES.CALL, objectType: 'functionType', icon: FiLogOut },
      { title: "Hats", dataType: DATA_TYPES.REFERENCE, objectType: 'hatType', icon: FiGrid },
      { title: "Boots", dataType: DATA_TYPES.REFERENCE, objectType: 'bootType', icon: FiBox },
    ],
    objectTypes: {
      programType: {
        name: 'Program',
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: true,
          color: "#3f3f3f",
          icon: FiBriefcase,
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
            accepts: ['operationType', 'functionType', 'blockType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      blockType: {
        name: "Block",
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: false,
          color: '#7f7f7f',
          icon: FiLayers,
          extras: [
            EXTRA_TYPES.COLLAPSE_TOGGLE,
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
            },
            EXTRA_TYPES.LOCKED_INDICATOR
          ]
        },
        referenceBlock: null,
        properties: {
          children: {
            name: 'Children',
            accepts: ['operationType', 'functionType', 'blockType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      functionType: {
        name: 'Function',
        type: TYPES.FUNCTION,
        instanceBlock: {
          onCanvas: true,
          color: "#62869e",
          icon: FiLogOut,
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
                  allowed: ['hatType','bootType']
                },
                {
                  type: EXTRA_TYPES.ADD_ARGUMENT,
                  argumentType: 'hatType'
                }
              ]
            },
            {
              type: EXTRA_TYPES.ADD_ARGUMENT_GROUP,
              allowed: ['hatType','bootType']
            }
          ]
        },
        callBlock: {
          onCanvas: false,
          color: "#62869e",
          icon: FiLogOut
        },
        properties: {
          children: {
            name: 'Children',
            accepts: ['operationType'],
            default: [],
            isList: true,
            fullWidth: true
          }
        }
      },
      operationType: {
        name: 'Operation',
        type: TYPES.OBJECT,
        instanceBlock: {
          onCanvas: false,
          color: "#629e6c",
          icon: FiClipboard,
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
        properties: {
          hat: {
            name: "Hat",
            accepts: ["hatType"],
            default: null,
            isList: false
          },
          boot: {
            name: "Boot",
            accepts: ["bootType"],
            default: null,
            isList: false
          },
          speed: {
            name: "Speed",
            type: SIMPLE_PROPERTY_TYPES.NUMBER,
            default: 1,
            min: 0,
            max: 2
          },
          doFunky: {
            name: "Do Funky",
            type: SIMPLE_PROPERTY_TYPES.BOOLEAN,
            default: false
          },
          greeting: {
            name: "Greeting",
            type: SIMPLE_PROPERTY_TYPES.STRING,
            default: ''
          },
          time: {
            name: "Time",
            type: SIMPLE_PROPERTY_TYPES.OPTIONS,
            options: [{value:'am',label:'AM'},{value:'pm',label:'PM'}],
            default: 'am'
          }
        }
      },
      hatType: {
        name: 'Hat',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: {
          onCanvas: false,
          color: "#AD1FDE",
          icon: FiGrid,
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
      bootType: {
        name: 'Boot',
        type: TYPES.OBJECT,
        instanceBlock: null,
        referenceBlock: {
          onCanvas: false,
          color: "#B3A533",
          icon: FiGrid,
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
        children: ['2dfsessfs']
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
      type: "functionType",
      dataType: DATA_TYPES.INSTANCE,
      arguments: ['s3siakawme'],
      properties: {
        children: []
      },
      position: { x: 400, y: 10 },
      canDelete: true,
      canEdit: true,
      selected: true,
      editing: false,
      
    },
    "s3siakawme" : {
      id: "s3siakawme",
      name: 'Passed Hat',
      type: "hatType",
      dataType: DATA_TYPES.ARGUMENT,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "2dfsessfs": {
      id: "2dfsessfs",
      name: 'MyOperation',
      type: "operationType",
      dataType: DATA_TYPES.INSTANCE,
      properties: {
        hat: null,
        boot: null,
        speed: 1,
        doFunky: true,
        greeting: 'Hello!'
      },
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "6dewwwwww": {
      id: "6dewwwwww",
      name: 'Sombrero',
      type: "hatType",
      dataType: DATA_TYPES.INSTANCE,
      canDelete: true,
      canEdit: true,
      selected: false,
      editing: false,
    },
    "pspssse32": {
      id: "pspssse32",
      name: 'Fur Boots',
      type: "bootType",
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
  // // Program-level updates
  // addChildPrimitive: (primitive, parentId) => set((state) => {
  //   if (!state.data[primitive.uuid]) {
  //     state.data[primitive.uuid] = primitive;
  //   }
  //   state.data[parentId].children.push(primitive.uuid);
  // }),
  // deleteBlock: (parentId, childId) => set((state) => {
  //   state.data = deleteAction(state.data, childId);
  //   if (state.data[parentId] && state.data[parentId].children) {
  //     state.data[parentId].children = state.data[parentId].children.filter(id => id !== childId);
  //   }
  // }),
  // moveTrajectoryWaypoint: (waypointId, oldParentId, newParentId, oldIndex, newIndex) => set((state) => {
  //   if (oldParentId === newParentId) {
  //     arrayMove(state.data[newParentId].waypoints,oldIndex,newIndex)
  //   } else {
  //     state.data[oldParentId].waypoints.splice(oldIndex, 1);
  //     state.data[newParentId].waypoints.splice(newIndex, 0, waypointId);
  //   }
  // }),
  // insertTrajectoryWaypoint: (waypointId, newParentId, newIndex) => set((state) => {
  //   state.data[newParentId].waypoints.splice(newIndex, 0, waypointId);
  //   // state.renderData[newParentId] = state.data[newParentId];
  // }),
  // deleteTrajectoryWaypoint: (parentId, index) => set((state) => {
  //   state.data[parentId].waypoints.splice(index, 1)
  // }),
  // moveChildPrimitive: (primitiveId, oldParentId, newParentId, oldIndex, newIndex) => set((state)=>{
  //   if (oldParentId === newParentId) {
  //     // console.log('moving within program')
  //     // Moving within the program
  //     arrayMove(state.data[oldParentId].children,oldIndex,newIndex)
  //   } else {
  //     state.data[oldParentId].children.splice(oldIndex, 1);
  //     state.data[newParentId].children.splice(newIndex, 0, primitiveId)
  //   }
  // }),
  // insertChildPrimitive: (primitive, newParentId, newIndex) => set(state=>{
  //   if (!state.data[primitive.uuid]) {
  //     // Add if it isn't here yet.
  //     state.data[primitive.uuid] = lodash.omit(primitive,'idx','parentData','dragBehavior','onDelete')
  //   }
  //   state.data[newParentId].children.splice(newIndex, 0, primitive.uuid);
  // }),
  // moveTrajectoryBlock: (trajectory, newParentId, argKey) => set((state) => {
  //   if (!state.data[trajectory.uuid]) {
  //     console.log('Adding because it doesnt exist in store')
  //     state.data[trajectory.uuid] = lodash.omit(trajectory,'parentData','dragBehavior','onDelete')
  //   }
  //   const field = argKey ? argKey : 'trajectory_uuid';
  //   if (trajectory.parentData.type !== 'drawer') {
  //     console.log('from drawer, not removing')
  //     state.data[trajectory.parentData.uuid].parameters[trajectory.parentData.field] = null
  //   }
  //   console.log(`Setting field ${field} of ${newParentId} to ${trajectory.uuid}`)
  //   state.data[newParentId].parameters[field] = trajectory.uuid;
  // }),
  // deletePrimitiveTrajectory: (parentId, field, trajectoryId) => set(state=>{
  //   state.data[parentId].parameters[field] = null;
  //   delete state.data[trajectoryId];
  // }),
  // // Piecewise update functions for data
  // addItem: (item) => set((state) => {
  //   state.data[item.uuid] = item;
  // }),
  // setItemProperty: (uuid, property, value) => set((state) => {
  //   state.data[uuid][property] = value;
  // }),
  // setPoseTransform: (uuid, transform) => set(state=>{
  //   state.data[uuid].position.x = transform.local.position.x;
  //   state.data[uuid].position.y = transform.local.position.y;
  //   state.data[uuid].position.z = transform.local.position.z;
  //   state.data[uuid].orientation.x = transform.local.quaternion.x;
  //   state.data[uuid].orientation.y = transform.local.quaternion.y;
  //   state.data[uuid].orientation.z = transform.local.quaternion.z;
  //   state.data[uuid].orientation.w = transform.local.quaternion.w;
  // }),
  // setPrimitiveParameter: (uuid, property, value) => set((state) => {
  //   state.data[uuid].parameters[property] = value
  // }),
  // deleteItem: (uuid) => set((state) => {
  //   if (state.data[uuid].type === 'waypoint') {
  //     Object.values(state.data).filter(v=>v.type === 'trajectory').forEach(trajectory => {
  //       state.data[trajectory.uuid].waypoints = trajectory.waypoints.filter(otherId => otherId !== uuid)
  //     })
  //   } else if (state.data[uuid].type === 'location') {
  //     Object.values(state.data).filter(v=>v.type === 'trajectory').forEach(trajectory => {
  //       if (trajectory.startLocation === uuid) {
  //         state.data[trajectory.uuid].startLocation = null
  //       }
  //       if (trajectory.endLocation === uuid) {
  //         state.data[trajectory.uuid].endLocation = null
  //       }
  //     })
  //   }
  //   delete state.data[uuid];
  // }),
  // moveItem: (uuid, x, y) => set((state) => {
  //   state.data[uuid].transform.x = x;
  //   state.data[uuid].transform.y = y;
  // }),
  // transferBlock: (data, previous, destination) => set(state=>{
  //   console.log({data,previous,destination})
  //   if (previous.uuid === 'drawer' && destination.idx === null) {
  //     state.data[destination.uuid][destination.field] = data.uuid;
  //     state.data[data.uuid] = data;
  //     return
  //   } else if (previous.uuid === 'drawer') {
  //     state.data[destination.uuid][destination.field].splice(destination.idx, 0, data.uuid)
  //     state.data[data.uuid] = data;
  //     return
  //   }
  //   if (previous.uuid !== destination.uuid || previous.field !== destination.field) {
  //     // Just delete from the previous location.
  //     if (previous.idx === null) {
  //       state.data[previous.uuid][previous.field] = null;
  //     } else {
  //       state.data[previous.uuid][previous.field].splice(previous.idx,1);
  //     }
  //     // Recreate in the destination location
  //     if (destination.idx === null) {
  //       state.data[destination.uuid][destination.field] = data.uuid;
  //     } else {
  //       state.data[destination.uuid][destination.field].splice(destination.idx,0, data.uuid);
  //     }
  //     return
  //   } else {
  //     if (previous.idx !== null) {
  //       // Move in the array
  //       arrayMove(state.data[previous.uuid][previous.field],previous.idx,destination.idx)
  //       return
  //     }
  //   }
  // }),
  // createAndPlaceItem: (item, x, y) => set((state) => {
  //   state.data[item.uuid] = lodash.omit({ ...item, transform: { x, y } }, 'parentData');
  // }),
  // createSkillArgument: (skillId, argument) => set((state)=>{
  //   state.data[skillId].arguments.push(argument);
  // }),
  // deleteArgumentFromChild: (uuid, argument) => set((state) => {
  //   if (argument.parameterType === 'machine' && state.data[uuid].parameters.machine === argument.uuid) {
  //     state.data[uuid].parameters.machine = null;
  //   } else if (argument.parameterType === 'thing' && state.data[uuid].parameters.thing === argument.uuid) {
  //     state.data[uuid].parameters.thing = null;
  //   } else if (argument.parameterType === 'trajectory' && state.data[uuid].parameters.trajectory === argument.uuid) {
  //     state.data[uuid].parameters.trajectory = null;
  //   } else if (argument.parameterType === 'location' && state.data[uuid].parameters.location === argument.uuid) {
  //     state.data[uuid].parameters.location = null;
  //   }
  // }),
  // deleteArgumentsFromHierarchical: (hierachicalId, argument) => {
  //   get().data[hierachicalId].children.forEach(uuid => {
  //     // Clear argument from hierarchicals
  //     if (uuid.includes("hierarchical")) {
  //       get().deleteArgumentsFromHierarchical(uuid, argument);
  //     }

  //     // Clear argument from children
  //     get().deleteArgumentFromChild(uuid, argument);
  //   });
  // },
  // deleteArgumentFromSkill: (skillId, argument) => set((state) => {
  //   state.data[skillId].arguments = state.data[skillId].arguments.filter(arg => arg.uuid !== argument.uuid)
  // }),
  // deleteSkillArgument: (skill, argument) => {
  //   skill.children.forEach(uuid => {
  //     // Clear argument from hierarchicals
  //     if (uuid.includes("hierarchical")) {
  //       get().deleteArgumentsFromHierarchical(uuid, argument);
  //     }

  //     // Clear argument from children
  //     get().deleteArgumentFromChild(uuid, argument);
  //   });

  //   // Delete Argument
  //   get().deleteArgumentFromSkill(skill.uuid, argument);
  // },
  // getSkillArugment: (skillId, argumentId) => get((state)=>{
  //   for (let i = 0; i < state.data[skillId].arguments.length; i++) {
  //     if (state.data[skillId].arguments[i].uuid === argumentId) {
  //       return state.data[skillId].arguments[i];
  //     }
  //   }
  //   return undefined
  // }),
  // setArgumentProperty: (skillId, argumentId, property, value) => set((state)=>{
  //   for (let i = 0; i < state.data[skillId].arguments.length; i++) {
  //     if (state.data[skillId].arguments[i].uuid === argumentId) {
  //       state.data[skillId].arguments[i][property] = value;
  //     }
  //   }
    
  // }),
  loadSolver: async () => {
    const module = await import('@people_and_robots/lively_tk');
    const solver = new module.Solver(urdf,[
      {type:'PositionMatch',name:"EE Position",link:"panda_hand",weight:50},
      {type:'OrientationMatch',name:"EE Rotation",link:"panda_hand",weight:25},
      {type:'SmoothnessMacro',name:"General Smoothness",weight:10},
      {type:'CollisionAvoidance',name:"Collision Avoidance",weight:10}
    ],[]);
    console.log(solver)
    set({solver})
  }
});