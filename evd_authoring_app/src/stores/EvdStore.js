import create from "zustand";
import produce from "immer";
import fakeEvdData from './fakeEvdData';
import { flattenProgram, unFlattenProgramPrimitives, unFlattenProgramSkills } from './helpers';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

export const typeToKey = (type) => {
  let key;
  switch(type) {
    case 'trajectory':
      key = 'trajectories';
      break;
    case 'collisionMesh':
      key = 'collisionMeshes';
      break;
    default:
      key = type + 's'
  }
  return key;
}

const store = (set,get) => ({
    name: 'Default Program Name',
    uuid: 'program',
    type: 'node.primitive.hierarchical.program.',
    description: 'Default Program Description',
    transform: {x:100,y:100},
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

      program.environment.grade_types.forEach((gradeType)=>{
        get().addItem('gradeType',gradeType)
      });
      program.environment.occupancy_zones.forEach((occupancyZone)=>{
        get().addItem('occupancyZone',occupancyZone)
      });
      program.environment.collision_meshes.forEach((collisionMesh)=>{
        get().addItem('collisionMesh',collisionMesh)
      });
      program.environment.pinch_points.forEach((pinchPoint)=>{
        get().addItem('pinchPoint',pinchPoint)
      });
      program.environment.trajectories.forEach((trajectory)=>{
        get().addItem('trajectory',trajectory)
      });
      get().addItem('reachSphere',program.environment.reach_sphere);
      program.environment.locations.forEach((location)=>{
        get().addItem('location',location)
      });
      program.environment.waypoints.forEach((waypoint)=>{
        get().addItem('waypoint',waypoint)
      });
      program.environment.machines.forEach((machine)=>{
        get().addItem('machine',machine)
      });
      program.environment.thing_types.forEach((thingType)=>{
        get().addItem('thingType',thingType)
      });
      program.environment.placeholders.forEach((placeholder)=>{
        get().addItem('placeholder',placeholder)
      });
      program.environment.regions.forEach((region)=>{
        get().addItem('region',region)
      });
      const [flattenedPrimitives,flattenedSkills] = flattenProgram(program.primitives,program.skills,{type:'program',uuid:program.uuid});
      flattenedPrimitives.forEach((primitive)=>{
        get().addItem('primitive',primitive)
      });
      flattenedSkills.forEach((skill,idx)=>{
        get().addItem('skill',{...skill,transform:{x:100,y:100+idx*20}})
      });
      program.primitives.forEach((primitive)=>{
        get().addChildPrimitive(primitive, program.uuid)
      });
    },
    getProgram: () => ({
      name:get().name,
      uuid:get().uuid,
      type:get().type,
      description:get().description,
      transform:get().transform,
      environment:{
        grade_types:Object.values(get().data.gradeTypes),
        occupancy_zones:Object.values(get().data.occupancyZones),
        collision_meshes:Object.values(get().data.collisionMeshes),
        pinch_points:Object.values(get().data.pinchPoints),
        reach_sphere:Object.values(get().data.reachSpheres)[0],
        locations:Object.values(get().data.locations),
        waypoints:Object.values(get().data.waypoints),
        machines:Object.values(get().data.machines),
        trajectories:Object.values(get().data.trajectories),
        thing_types:Object.values(get().data.thingTypes),
        placeholders:Object.values(get().data.placeholders),
        regions:Object.values(get().data.regions),
      },
      primitives:unFlattenProgramPrimitives(get().data.primitives, get().primitiveIds),
      skills:unFlattenProgramSkills(get().data.skills, get().data.primitives)
    }),
    // Program-level updates
    setName: (text) => set((_)=>({name:text})),
    setUuid: (text) => set((_)=>({uuid:text})),
    setType: (text) => set((_)=>({type:text})),
    setDescription: (text) => set((_)=>({description:text})),
    addChildPrimitive: (primitive, parentId) => set((state)=>{
      if (!state.data.primitives[primitive.uuid]) {
        state.data.primitives[primitive.uuid] = primitive;
      }
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.data.primitives[primitive.uuid].parentData = {type:'program',uuid:state.uuid}
        state.primitiveIds.push(primitive.uuid)
      } else if (state.data.primitives[parentId]) {
        // This is a child of another primitive
        state.data.primitives[primitive.uuid].parentData = {type:'primitive',uuid:parentId}
        state.data.primitives[parentId].primitiveIds.push(primitive.uuid)
      } else if (state.data.skills[parentId]) {
        // This is a child of a skill
        state.data.primitives[primitive.uuid].parentData = {type:'skill',uuid:parentId}
        state.data.skills[parentId].primitiveIds.push(primitive.uuid)
      }
    }),
    deleteChildPrimitive: (primitiveId) => set((state)=>{
      const {type,uuid} = state.data.primitives[primitiveId].parentData;
      if (type === 'program') {
        // console.log('removing primitive from program')
        state.primitiveIds = state.primitiveIds.filter(id=>id!==primitiveId)
      } else {
        state.data[typeToKey(type)][uuid].primitiveIds = state.data[typeToKey(type)][uuid].primitiveIds.filter(id=>id!==primitiveId)
      }
      delete state.data.primitives[primitiveId];
    }),
    moveChildPrimitive: (primitive, parentId, index) => set((state)=>{
      if (!state.data.primitives[primitive.uuid]) {
        state.data.primitives[primitive.uuid] = primitive;
      }
      const {type,uuid} = state.data.primitives[primitive.uuid].parentData;
      
      // Short-circuit if no move needs to be done.
      if (type === 'program' && uuid === parentId && state.primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('program/index match')
        return;
      } else if (type === 'drawer') {
        
      } else if (type === 'primitive' && uuid === parentId && state.data.primitives[uuid].primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('primitive/index match')
        return;
      } else if (type === 'skill' && uuid === parentId && state.data.skills[uuid].primitiveIds.indexOf(primitive.uuid) === index) {
        // console.log('skill/index match')
        return;
      }
      // remove from previous location
      if (type === 'program') {
        state.primitiveIds = state.primitiveIds.filter(id=>id!==primitive.uuid)
      } else if (type === 'drawer') {} else {
        state.data[typeToKey(type)][uuid].primitiveIds = state.data[typeToKey(type)][uuid].primitiveIds.filter(id=>id!==primitive.uuid)
      }
      // add into the correct location
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.primitiveIds.splice(index,0,primitive.uuid)
        state.data.primitives[primitive.uuid].parentData.type = 'program';
      } else if (state.data.primitives[parentId]) {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds.splice(index,0,primitive.uuid)
        state.data.primitives[primitive.uuid].parentData.type = 'primitive';
      } else if (state.data.skills[parentId]) {
        state.data.skills[parentId].primitiveIds.splice(index,0,primitive.uuid)
        state.data.primitives[primitive.uuid].parentData.type = 'skill';
      }
      state.data.primitives[primitive.uuid].parentData.uuid = parentId;
    }),
    // Piecewise update functions for data
    addItem: (type, item) => set((state)=>{
      state.data[typeToKey(type)][item.uuid] = item
    }),
    setItemProperty: (type, uuid, property, value) => set((state)=>{
      state.data[typeToKey(type)][uuid][property] = value
    }),
    setPrimitiveParameter: (type, uuid, property, value) => set((state)=>{
      state.data[typeToKey(type)][uuid].parameters[property] = value
    }),
    deleteItem: (type, uuid) => set((state)=>{
      delete state.data[typeToKey(type)][uuid]
    }),
    moveItem: (type,uuid,x,y) => set((state)=>{
      if (type==='program') {
        state.transform.x += x;
        state.transform.y += y;
      } else if (type==='skill') {
        state.data[typeToKey(type)][uuid].transform.x += x;
        state.data[typeToKey(type)][uuid].transform.y += y;
      }
    })
});

const useEvdStore = create(immer(store));

console.log(fakeEvdData.arbitrary.program)
// Remove for ROS-based updates later (useful for frontend dev)
useEvdStore.getState().setProgram(fakeEvdData.arbitrary.program);

console.log('Primitives');
console.log(useEvdStore.getState().data.primitives);
console.log('Skills');
console.log(useEvdStore.getState().data.skills);
console.log('Top-level primitives');
console.log(useEvdStore.getState().primitiveIds);

export default useEvdStore;