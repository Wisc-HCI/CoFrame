import { arrayMove, deleteAction } from './helpers';
import lodash from 'lodash';

export const EvdSlice = (set, get) => ({
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
  // Program-level updates
  addChildPrimitive: (primitive, parentId) => set((state) => {
    if (!state.data[primitive.uuid]) {
      state.data[primitive.uuid] = primitive;
    }
    state.data[parentId].children.push(primitive.uuid);
  }),
  deleteBlock: (parentId, childId) => set((state) => {
    state.data = deleteAction(state.data, childId);
    if (state.data[parentId] && state.data[parentId].children) {
      state.data[parentId].children = state.data[parentId].children.filter(id => id !== childId);
    }
  }),
  moveTrajectoryWaypoint: (waypointId, oldParentId, newParentId, oldIndex, newIndex) => set((state) => {
    if (oldParentId === newParentId) {
      arrayMove(state.data[newParentId].waypoints,oldIndex,newIndex)
    } else {
      state.data[oldParentId].waypoints.splice(oldIndex, 1);
      state.data[newParentId].waypoints.splice(newIndex, 0, waypointId);
    }
  }),
  insertTrajectoryWaypoint: (waypointId, newParentId, newIndex) => set((state) => {
    state.data[newParentId].waypoints.splice(newIndex, 0, waypointId);
    // state.renderData[newParentId] = state.data[newParentId];
  }),
  deleteTrajectoryWaypoint: (parentId, index) => set((state) => {
    state.data[parentId].waypoints.splice(index, 1)
  }),
  moveChildPrimitive: (primitiveId, oldParentId, newParentId, oldIndex, newIndex) => set((state)=>{
    if (oldParentId === newParentId) {
      // console.log('moving within program')
      // Moving within the program
      arrayMove(state.data[oldParentId].children,oldIndex,newIndex)
    } else {
      state.data[oldParentId].children.splice(oldIndex, 1);
      state.data[newParentId].children.splice(newIndex, 0, primitiveId)
    }
  }),
  insertChildPrimitive: (primitive, newParentId, newIndex) => set(state=>{
    if (!state.data[primitive.uuid]) {
      // Add if it isn't here yet.
      state.data[primitive.uuid] = lodash.omit(primitive,'idx','parentData','dragBehavior','onDelete')
    }
    state.data[newParentId].children.splice(newIndex, 0, primitive.uuid);
  }),
  moveTrajectoryBlock: (trajectory, newParentId, argKey) => set((state) => {
    if (!state.data[trajectory.uuid]) {
      console.log('Adding because it doesnt exist in store')
      state.data[trajectory.uuid] = lodash.omit(trajectory,'parentData','dragBehavior','onDelete')
    }
    const field = argKey ? argKey : 'trajectory_uuid';
    if (trajectory.parentData.type !== 'drawer') {
      console.log('from drawer, not removing')
      state.data[trajectory.parentData.uuid].parameters[trajectory.parentData.field] = null
    }
    console.log(`Setting field ${field} of ${newParentId} to ${trajectory.uuid}`)
    state.data[newParentId].parameters[field] = trajectory.uuid;
  }),
  deletePrimitiveTrajectory: (parentId, field, trajectoryId) => set(state=>{
    state.data[parentId].parameters[field] = null;
    delete state.data[trajectoryId];
  }),
  // Piecewise update functions for data
  addItem: (item) => set((state) => {
    state.data[item.uuid] = item;
  }),
  setItemProperty: (uuid, property, value) => set((state) => {
    state.data[uuid][property] = value;
  }),
  setPoseTransform: (uuid, transform) => set(state=>{
    state.data[uuid].position.x = transform.local.position.x;
    state.data[uuid].position.y = transform.local.position.y;
    state.data[uuid].position.z = transform.local.position.z;
    state.data[uuid].orientation.x = transform.local.quaternion.x;
    state.data[uuid].orientation.y = transform.local.quaternion.y;
    state.data[uuid].orientation.z = transform.local.quaternion.z;
    state.data[uuid].orientation.w = transform.local.quaternion.w;
  }),
  setPrimitiveParameter: (uuid, property, value) => set((state) => {
    state.data[uuid].parameters[property] = value
  }),
  deleteItem: (uuid) => set((state) => {
    if (state.data[uuid].type === 'waypoint') {
      Object.values(state.data).filter(v=>v.type === 'trajectory').forEach(trajectory => {
        state.data[trajectory.uuid].waypoints = trajectory.waypoints.filter(otherId => otherId !== uuid)
      })
    } else if (state.data[uuid].type === 'location') {
      Object.values(state.data).filter(v=>v.type === 'trajectory').forEach(trajectory => {
        if (trajectory.startLocation === uuid) {
          state.data[trajectory.uuid].startLocation = null
        }
        if (trajectory.endLocation === uuid) {
          state.data[trajectory.uuid].endLocation = null
        }
      })
    }
    delete state.data[uuid];
  }),
  moveItem: (uuid, x, y) => set((state) => {
    state.data[uuid].transform.x = x;
    state.data[uuid].transform.y = y;
  }),
  createAndPlaceItem: (item, x, y) => set((state) => {
    state.data[item.uuid] = lodash.omit({ ...item, transform: { x, y } }, 'parentData');
  }),
  createSkillArgument: (skillId, argument) => set((state)=>{
    state.data[skillId].arguments.push(argument);
  }),
  deleteArgumentFromChild: (uuid, argument) => set((state) => {
    if (argument.parameterType === 'machine' && state.data[uuid].parameters.machine === argument.uuid) {
      state.data[uuid].parameters.machine = null;
    } else if (argument.parameterType === 'thing' && state.data[uuid].parameters.thing === argument.uuid) {
      state.data[uuid].parameters.thing = null;
    } else if (argument.parameterType === 'trajectory' && state.data[uuid].parameters.trajectory === argument.uuid) {
      state.data[uuid].parameters.trajectory = null;
    } else if (argument.parameterType === 'location' && state.data[uuid].parameters.location === argument.uuid) {
      state.data[uuid].parameters.location = null;
    }
  }),
  deleteArgumentsFromHierarchical: (hierachicalId, argument) => {
    get().data[hierachicalId].children.forEach(uuid => {
      // Clear argument from hierarchicals
      if (uuid.includes("hierarchical")) {
        get().deleteArgumentsFromHierarchical(uuid, argument);
      }

      // Clear argument from children
      get().deleteArgumentFromChild(uuid, argument);
    });
  },
  deleteArgumentFromSkill: (skillId, argument) => set((state) => {
    state.data[skillId].arguments = state.data[skillId].arguments.filter(arg => arg.uuid !== argument.uuid)
  }),
  deleteSkillArgument: (skill, argument) => {
    skill.children.forEach(uuid => {
      // Clear argument from hierarchicals
      if (uuid.includes("hierarchical")) {
        get().deleteArgumentsFromHierarchical(uuid, argument);
      }

      // Clear argument from children
      get().deleteArgumentFromChild(uuid, argument);
    });

    // Delete Argument
    get().deleteArgumentFromSkill(skill.uuid, argument);
  },
  getSkillArugment: (skillId, argumentId) => get((state)=>{
    for (let i = 0; i < state.data[skillId].arguments.length; i++) {
      if (state.data[skillId].arguments[i].uuid === argumentId) {
        return state.data[skillId].arguments[i];
      }
    }
    return undefined
  }),
  setArgumentProperty: (skillId, argumentId, property, value) => set((state)=>{
    for (let i = 0; i < state.data[skillId].arguments.length; i++) {
      if (state.data[skillId].arguments[i].uuid === argumentId) {
        state.data[skillId].arguments[i][property] = value;
      }
    }
    
  })
});