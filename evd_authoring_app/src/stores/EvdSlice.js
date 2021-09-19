import { arrayMove, flattenProgram, unFlattenProgramPrimitives, unFlattenProgramSkills } from './helpers';
import lodash from 'lodash';
import { typeToKey } from './helpers';
// import { useSceneStore } from 'robot-scene';

export const EvdSlice = (set, get) => ({
  name: 'Default Program Name',
  uuid: 'program',
  type: 'node.primitive.hierarchical.program.',
  description: 'Default Program Description',
  transform: { x: 30, y: 10 },
  data: {
    //TODO store other data
    gradeTypes: {},
    occupancyZones: {},
    collisionMeshes: {},
    pinchPoints: {},
    reachSpheres: {},
    locations: {},
    trajectories: {},
    waypoints: {},
    machines: {},
    thingTypes: {},
    placeholders: {},      // Only shown through thingTypes
    regions: {},      // Only shown through machines (I added regions back in after reworking machines)
    primitives: {}, //lookup table (of flattened hierarchicals)
    // for hierarchical primitives, extract children from primtives list, 
    // replace with uuid (or make a primtitiveUuids list and delete primitives list)
    skills: {}, // skills are not blockly! (Designers can add new skills)
  },
  primitiveIds: [], // ordered list of uuids
  // A macro for updating the entire program from raw data
  setProgram: (program) => {
    get().setName(program.name);
    get().setUuid(program.uuid);
    get().setType(program.type);
    get().setDescription(program.description);

    program.environment.grade_types.forEach((gradeType) => {
      get().addItem('gradeType', gradeType)
    });
    program.environment.occupancy_zones.forEach((occupancyZone) => {
      get().addItem('occupancyZone', occupancyZone)
    });
    program.environment.collision_meshes.forEach((collisionMesh) => {
      get().addItem('collisionMesh', collisionMesh)
    });
    program.environment.pinch_points.forEach((pinchPoint) => {
      get().addItem('pinchPoint', pinchPoint)
    });
    get().addItem('reachSphere', program.environment.reach_sphere);
    program.environment.locations.forEach((location) => {
      get().addItem('location', location)
    });
    program.environment.waypoints.forEach((waypoint) => {
      get().addItem('waypoint', waypoint)
    });
    program.environment.trajectories.forEach((trajectory) => {
      get().addItem('trajectory', trajectory)
    });
    program.environment.machines.forEach((machine) => {
      get().addItem('machine', machine)
    });
    program.environment.thing_types.forEach((thingType) => {
      get().addItem('thingType', thingType)
    });
    program.environment.placeholders.forEach((placeholder) => {
      get().addItem('placeholder', placeholder)
    });
    program.environment.regions.forEach((region) => {
      get().addItem('region', region)
    });
    const [flattenedPrimitives, flattenedSkills] = flattenProgram(program.primitives, program.skills, { type: 'program', uuid: program.uuid });
    flattenedPrimitives.forEach((primitive) => {
      get().addItem('primitive', primitive)
    });
    flattenedSkills.forEach((skill, idx) => {
      get().addItem('skill', { ...skill, transform: { x: 520 + idx * 400, y: 10 } })
    });
    program.primitives.forEach((primitive) => {
      get().addChildPrimitive(primitive, program.uuid)
    });
  },
  getProgram: () => ({
    name: get().name,
    uuid: get().uuid,
    type: get().type,
    description: get().description,
    transform: get().transform,
    environment: {
      grade_types: Object.values(get().data.gradeTypes),
      occupancy_zones: Object.values(get().data.occupancyZones),
      collision_meshes: Object.values(get().data.collisionMeshes),
      pinch_points: Object.values(get().data.pinchPoints),
      reach_sphere: Object.values(get().data.reachSpheres)[0],
      locations: Object.values(get().data.locations),
      waypoints: Object.values(get().data.waypoints),
      machines: Object.values(get().data.machines),
      trajectories: Object.values(get().data.trajectories),
      thing_types: Object.values(get().data.thingTypes),
      placeholders: Object.values(get().data.placeholders),
      regions: Object.values(get().data.regions),
    },
    primitives: unFlattenProgramPrimitives(get().data.primitives, get().primitiveIds),
    skills: unFlattenProgramSkills(get().data.skills, get().data.primitives)
  }),
  // Program-level updates
  setName: (text) => set((_) => ({ name: text })),
  setUuid: (text) => set((_) => ({ uuid: text })),
  setType: (text) => set((_) => ({ type: text })),
  setDescription: (text) => set((_) => ({ description: text })),
  addChildPrimitive: (primitive, parentId) => set((state) => {
    if (!state.data.primitives[primitive.uuid]) {
      state.data.primitives[primitive.uuid] = primitive;
    }
    if (parentId === state.uuid) {
      // This is the top-level program, so add to top level list of uuids
      state.data.primitives[primitive.uuid].parentData = { type: 'program', uuid: state.uuid }
      state.primitiveIds.push(primitive.uuid)
    } else if (state.data.primitives[parentId]) {
      // This is a child of another primitive
      state.data.primitives[primitive.uuid].parentData = { type: 'primitive', uuid: parentId }
      state.data.primitives[parentId].primitiveIds.push(primitive.uuid)
    } else if (state.data.skills[parentId]) {
      // This is a child of a skill
      state.data.primitives[primitive.uuid].parentData = { type: 'skill', uuid: parentId }
      state.data.skills[parentId].primitiveIds.push(primitive.uuid)
    }
  }),
  deleteChildPrimitive: (primitiveId) => set((state) => {
    const { type, uuid } = state.data.primitives[primitiveId].parentData;
    if (type === 'program') {
      // console.log('removing primitive from program')
      state.primitiveIds = state.primitiveIds.filter(id => id !== primitiveId)
    } else {
      state.data[typeToKey(type)][uuid].primitiveIds = state.data[typeToKey(type)][uuid].primitiveIds.filter(id => id !== primitiveId)
    }
    if (state.data.primitives[primitiveId].type === 'node.primitive.move-trajectory.' && state.data.primitives[primitiveId].parameters.trajectory_uuid) {
      delete state.data.trajectories[state.data.primitives[primitiveId].parameters.trajectory_uuid]
    }
    delete state.data.primitives[primitiveId];
  }),
  deleteSkill: (skillId) => {
    const skill = get().data.skills[skillId];
    get().deleteHierarchical(skill)
  },
  deleteHierarchical: (hierarchical) => {
    // First, clean out all the contents recursively
    hierarchical.primitiveIds.forEach(id => {
      if (get().data.primitives[id].type.includes('hierarchical')) {
        get().deleteHierarchical(get().data.primitives[id])
      } else {
        get().deleteChildPrimitive(id)
      }
    })
    // If a skill, go through and delete all calls using this skill.
    if (hierarchical.type === 'node.primitive.hierarchical.skill.') {
      Object.values(get().data.primitives).forEach(primitive => {
        if (primitive.type === 'node.primitive.skill-call' && primitive.parameters.skill_uuid) {
          get().deleteChildPrimitive(primitive.uuid)
        }
      })
      get().deleteItem('skill', hierarchical.uuid)
    } else {
      get().deleteItem('primitive', hierarchical.uuid)
    }

  },
  moveTrajectoryWaypoint: (waypointId, oldParentId, newParentId, oldIndex, newIndex) => set((state) => {
    if (oldParentId === newParentId) {
      arrayMove(state.data.trajectories[newParentId].waypoint_uuids,oldIndex,newIndex)
    } else {
      state.data.trajectories[oldParentId].waypoint_uuids.splice(oldIndex, 1);
      state.data.trajectories[newParentId].waypoint_uuids.splice(newIndex, 0, waypointId)
    }
  }),
  insertTrajectoryWaypoint: (waypointId, newParentId, newIndex) => set((state) => {
    state.data.trajectories[newParentId].waypoint_uuids.splice(newIndex, 0, waypointId)
  }),
  deleteTrajectoryWaypoint: (parentId, index) => set((state) => {
    state.data.trajectories[parentId].waypoint_uuids.splice(index, 1)
  }),
  moveChildPrimitive: (primitiveId, oldParentId, newParentId, oldIndex, newIndex) => set((state)=>{
    if (oldParentId === newParentId && state.uuid === oldParentId) {
      // Moving within the program
      arrayMove(state.primitiveIds,oldIndex,newIndex)
    } else if (oldParentId === newParentId) {
      // Moving within a single skill or primitive
      const parentType = state.data.skills[newParentId] ? 'skills' : 'primitives';
      arrayMove(state.data[parentType][newParentId].primitiveIds,oldIndex,newIndex)
    } else if (state.uuid === oldParentId) {
      // Move from 
      const newParentType = state.data.skills[newParentId] ? 'skills' : 'primitives';
      state.primitiveIds.splice(oldIndex, 1);
      state.data[newParentType][newParentId].primitiveIds.splice(newIndex, 0, primitiveId)
    } else if (state.uuid === newParentId) {
      const oldParentType = state.data.skills[oldParentId] ? 'skills' : 'primitives';
      state.data[oldParentType][oldParentId].primitiveIds.splice(oldIndex, 1);
      state.primitiveIds.splice(newIndex, 0, primitiveId)
    } else {
      const newParentType = state.data.skills[newParentId] ? 'skills' : 'primitives';
      const oldParentType = state.data.skills[oldParentId] ? 'skills' : 'primitives';
      state.data[oldParentType][oldParentId].primitiveIds.splice(oldIndex, 1);
      state.data[newParentType][newParentId].primitiveIds.splice(newIndex, 0, primitiveId)
    }
  }),
  insertChildPrimitive: (primitive, newParentId, newIndex) => set(state=>{
    if (!state.data.primitives[primitive.uuid]) {
      // Add if it isn't here yet.
      state.data.primitives[primitive.uuid] = lodash.omit(primitive,'idx','parentData','dragBehavior','onDelete')
    }
    if (newParentId === state.uuid) {
      state.primitiveIds.splice(newIndex, 0, primitive.uuid);
    } else if (state.data.skills[newParentId]) {
      state.data.skills[newParentId].primitiveIds.splice(newIndex, 0, primitive.uuid);
    } else {
      state.data.primitives[newParentId].primitiveIds.splice(newIndex, 0, primitive.uuid);
    }
  }),
  moveTrajectoryBlock: (trajectory, newParentId, argKey) => set((state) => {
    const onFile = state.data.trajectories[trajectory.uuid];
    if (onFile) {
      // It is either a parameter for a skill-call or move-trajectory primitive
      const oldParentNode = state.data.primitives[onFile.parentData.uuid];
      if (oldParentNode.type === 'node.primitive.skill-call.') {
        Object.entries(oldParentNode.parameters).some((keyValuePair) => {
          if (keyValuePair[1] === trajectory.uuid) {
            ///state.data.primitives[onFile.parentData.uuid].parameters[keyValuePair[0]] = null
            // Short-circuits the iteration
            return true
          } else { return false }
        })
      } else if (oldParentNode.type === 'node.primitive.move-trajectory.') {
        state.data.primitives[onFile.parentData.uuid].parameters.trajectory_uuid = null
      }
    } else {
      state.data.trajectories[trajectory.uuid] = lodash.omit(trajectory,'parentData','dragBehavior','onDelete')
    }
    const newParentNode = state.data.primitives[newParentId];
    if (newParentNode.type === 'node.primitive.skill-call') {
      state.data.primitives[newParentId].parameters[argKey] = trajectory.uuid
    } else if (newParentNode.type === 'node.primitive.move-trajectory.') {
      state.data.primitives[newParentId].parameters.trajectory_uuid = trajectory.uuid
    }
  }),
  deletePrimitiveTrajectory: (parentId, field, trajectoryId) => set(state=>{
    state.data.primitives[parentId].parameters[field] = null;
    delete state.data.trajectories[trajectoryId];
  }),
  // Piecewise update functions for data
  addItem: (type, item) => set((state) => {
    state.data[typeToKey(type)][item.uuid] = item;
  }),
  setItemProperty: (type, uuid, property, value) => set((state) => {
    state.data[typeToKey(type)][uuid][property] = value;
  }),
  setPrimitiveParameter: (type, uuid, property, value) => set((state) => {
    state.data[typeToKey(type)][uuid].parameters[property] = value
  }),
  deleteItem: (type, uuid) => set((state) => {
    delete state.data[typeToKey(type)][uuid]
    if (type === "waypoint") {
      Object.keys(state.data.trajectories).forEach(trajectory_uuid => {
        state.data.trajectories[trajectory_uuid].waypoint_uuids = state.data.trajectories[trajectory_uuid].waypoint_uuids.filter(other_uuid => other_uuid !== uuid)
      })
      // TODO: Remove from other params.
    } else if (type === 'location') {
      Object.keys(state.data.trajectories).forEach(trajectory_uuid => {
        if (state.data.trajectories[trajectory_uuid].start_location_uuid === uuid) {
          state.data.trajectories[trajectory_uuid].start_location_uuid = null
        }
        if (state.data.trajectories[trajectory_uuid].end_location_uuid === uuid) {
          state.data.trajectories[trajectory_uuid].end_location_uuid = null
        }
      })
      // TODO: Remove from other params.
    }
  }),
  moveItem: (type, uuid, x, y) => set((state) => {
    if (type === 'program') {
      state.transform.x += x;
      state.transform.y += y;
    } else if (type === 'skill') {
      state.data[typeToKey(type)][uuid].transform.x += x;
      state.data[typeToKey(type)][uuid].transform.y += y;
    }
  }),
  createAndPlaceItem: (type, item, x, y) => set((state) => {
    if (type === 'skill') {
      state.data[typeToKey(type)][item.uuid] = lodash.omit({ ...item, transform: { x, y } }, 'parentData');
    }
  }),
  createSkillArgument: (skill_uuid, argument) => set((state)=>{
    state.data[typeToKey('skill')][skill_uuid].arguments.push(argument);
  }),
  deleteSkillArgument: (skill_uuid, argument) => set((state)=>{
    // Clear the children's parameters of deleted argument
    state.data[typeToKey('skill')][skill_uuid].primitiveIds.forEach(uuid => {
      if (argument.parameter_type === 'node.machine.' && state.data.primitives[uuid].parameters.machine_uuid === argument.uuid) {
        state.data.primitives[uuid].parameters.machine_uuid = null;
      } else if (argument.parameter_type === 'node.pose.thing.' && state.data.primitives[uuid].parameters.thing_uuid === argument.uuid) {
        state.data.primitives[uuid].parameters.thing_uuid = null;
      } else if (argument.parameter_type === 'node.trajectory.' && state.data.primitives[uuid].parameters.trajectory_uuid === argument.uuid) {
        state.data.primitives[uuid].parameters.trajectory_uuid = null;
      } else if (argument.parameter_type === 'node.pose.waypoint.location.' && state.data.primitives[uuid].parameters.location_uuid === argument.uuid) {
        state.data.primitives[uuid].parameters.location_uuid = null;
      }
    });
    // Delete Argument
    state.data[typeToKey('skill')][skill_uuid].arguments = state.data[typeToKey('skill')][skill_uuid].arguments.filter(arg => arg.uuid !== argument.uuid)

  }),
  getSkillArugment: (skill_uuid, argument_uuid) => get((state)=>{
    for (let i = 0; i < state.data[typeToKey('skill')][skill_uuid].arguments.length; i++) {
      if (state.data[typeToKey('skill')][skill_uuid].arguments[i].uuid === argument_uuid) {
        return state.data[typeToKey('skill')][skill_uuid].arguments[i];
      }
    }
    return undefined
  }),
  setArgumentProperty: (skill_uuid, argument_uuid, property, value) => set((state)=>{
    for (let i = 0; i < state.data[typeToKey('skill')][skill_uuid].arguments.length; i++) {
      if (state.data[typeToKey('skill')][skill_uuid].arguments[i].uuid === argument_uuid) {
        state.data[typeToKey('skill')][skill_uuid].arguments[i][property] = value;
      }
    }
    
  }),
  toggleSkillEditable: (skill_uuid) => set((state) => {
    state.data[typeToKey('skill')][skill_uuid].editable = !state.data[typeToKey('skill')][skill_uuid].editable
  })
});