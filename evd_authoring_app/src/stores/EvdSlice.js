import { flattenProgram, unFlattenProgramPrimitives, unFlattenProgramSkills } from './helpers';
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
  moveTrajectoryWaypoint: (waypoint, newParentId, index) => set((state) => {
    if (waypoint.parentData.uuid !== newParentId) {
      state.data.trajectories[newParentId].waypoint_uuids.splice(index,0,waypoint.uuid)
    } else {
      console.log(waypoint)
    }
    
  }),
  deleteTrajectoryWaypoint: (parentId, index) => set((state)=> {
    state.data.trajectories[parentId].waypoint_uuids.splice(index,1)
  }),
  moveTrajectoryBlock: (trajectory, newParentId, argKey) => set((state) => {
    const onFile = state.data.trajectories[trajectory.uuid];
    if (onFile) {
      // It is either a parameter for a skill-call or move-trajectory primitive
      const oldParentNode = state.data.primitives[onFile.parentData.uuid];
      if (oldParentNode.type === 'node.primitive.skill-call.') {
        Object.entries(oldParentNode.parameters).some((keyValuePair) => {
          if (keyValuePair[1] === trajectory.uuid) {
            state.data.primitives[onFile.parentData.uuid].parameters[keyValuePair[0]] = null
            // Short-circuits the iteration
            return true
          } else { return false }
        })
      } else if (oldParentNode.type === 'node.primitive.move-trajectory.') {
        state.data.primitives[onFile.parentData.uuid].parameters.trajectory_uuid = null
      }
    }
    const newParentNode = state.data.primitives[newParentId];
    if (newParentNode.type === 'node.primitive.skill-call') {
      state.data.primitives[newParentId].parameters[argKey] = trajectory.uuid
    } else if (newParentNode.type === 'node.primitive.move-trajectory.') {
      state.data.primitives[newParentId].parameters.trajectory_uuid = trajectory.uuid
    }
    // Update the new parent data for this trajectory
    state.data.trajectories[trajectory.uuid].parentData = { type: 'primitive', uuid: newParentId };
  }),
  moveChildPrimitive: (primitive, newParentId, newParentType, index) => set((state) => {
    const onFile = state.data.primitives[primitive.uuid];
    if (onFile) {
      const oldParentData = state.data.primitives[primitive.uuid].parentData;
      // Short-circuit if no move needs to be done.
      if (oldParentData.type === 'program' && oldParentData.uuid === newParentId && state.primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('program/index match')
        return;
      } else if (oldParentData.type === 'drawer') {
        // Don't bother moving
      } else if (oldParentData.type === 'primitive' && oldParentData.uuid === newParentId && state.data.primitives[oldParentData.uuid].primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('primitive/index match')
        return;
      } else if (oldParentData.type === 'skill' && oldParentData.uuid === newParentId && state.data.skills[oldParentData.uuid].primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('skill/index match')
        return;
      }
      // Remove from previous location
      if (oldParentData.type === 'program') {
        state.primitiveIds = state.primitiveIds.filter(id => id !== primitive.uuid)
      } else if (oldParentData.type === 'drawer') { } else {
        state.data[typeToKey(oldParentData.type)][oldParentData.uuid].primitiveIds = state.data[typeToKey(oldParentData.type)][oldParentData.uuid].primitiveIds.filter(id => id !== primitive.uuid)
      }
    }
    state.data.primitives[primitive.uuid] = { ...primitive, parentData: { type: newParentType, uuid: newParentId } };
    if (newParentType === 'program') {
      state.primitiveIds.splice(index, 0, primitive.uuid)
    } else if (newParentType === 'skill') {
      state.data.skills[newParentId].primitiveIds.splice(index, 0, primitive.uuid)
    } else if (newParentType === 'primitive') {
      state.data.primitives[newParentId].primitiveIds.splice(index, 0, primitive.uuid)
    }
  }),
  // Piecewise update functions for data
  addItem: (type, item) => set((state) => {
    state.data[typeToKey(type)][item.uuid] = item;
  }),
  setItemProperty: (type, uuid, property, value) => set((state) => {
    state.data[typeToKey(type)][uuid][property] = value;
    // let item = state.data[typeToKey(type)][uuid];
    // let frame = state.frame;
    // if (['waypoint','location'].indexOf(type)>=0) {
      // poseDataToShapes(item,frame).forEach(shape=>{
      //   state.item[shape.uuid].position = shape.position;
      //   state.item[shape.uuid].rotation = shape.rotation;
      // });
      // // Enumerate the trajectories and update their visuals if they use this location or waypoint
      // Object.keys(state.data.trajectories).forEach(trajectory_uuid=>{
      //   let trajectory = state.data.trajectories[trajectory_uuid];
      //   if (trajectory.start_location_uuid === uuid || trajectory.end_location_uuid === uuid || trajectory.waypoint_uuids.indexOf(uuid) >= 0) {
      //     let locations = state.data.locations;
      //     let waypoints = state.data.waypoints;
      //     let frame = state.frame;
      //     state.lines[trajectory_uuid] = trajectoryDataToLine(trajectory,locations,waypoints,frame);
      //   }
      // })
    // }
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
      console.log(state.transform)
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
  createSkillParameter: (skill_uuid, parameter) => set((state)=>{
    state.data[typeToKey('skill')][skill_uuid].parameters[parameter.uuid] = parameter
  }),
  deleteSkillParameter: (skill_uuid, parameter_uuid) => set((state)=>{
    delete state.data[typeToKey('skill')][skill_uuid].parameters[parameter_uuid]
  }),
  setParameterProperty: (skill_uuid, parameter_uuid, property, value) => set((state)=>{
    state.data[typeToKey('skill')][skill_uuid].parameters[parameter_uuid][property] = value
  }),
  toggleSkillEditable: (skill_uuid) => set((state) => {
    state.data[typeToKey('skill')][skill_uuid].editable = !state.data[typeToKey('skill')][skill_uuid].editable
  })
});

// const useEvdStore = create(immer(store));

// console.log(fakeEvdData.arbitrary.program)
// // Remove for ROS-based updates later (useful for frontend dev)
// useEvdStore.getState().setProgram(fakeEvdData.arbitrary.program);

// console.log('Primitives');
// console.log(useEvdStore.getState().data.primitives);
// console.log('Skills');
// console.log(useEvdStore.getState().data.skills);
// console.log('Top-level primitives');
// console.log(useEvdStore.getState().primitiveIds);

// export default useEvdStore;