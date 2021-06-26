import create from "zustand";
import produce from "immer";
import fakeEvdData from './fakeEvdData';
import { flattenProgram } from './helpers';

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
      waypoints: {},
      machines: {},
      thingTypes: {},
      things: {},      // Only shown through thingTypes
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
      program.environment.things.forEach((thing)=>{
        get().addItem('thing',thing)
      });
      program.environment.regions.forEach((region)=>{
        get().addItem('region',region)
      });

      const [flattenedPrimitives,flattenedSkills] = flattenProgram(program.primitives,program.skills);
      flattenedPrimitives.forEach((primitive)=>{
        get().addItem('primitive',primitive)
      });
      flattenedSkills.forEach((skill)=>{
        get().addItem('skill',skill)
      });
      program.primitives.forEach((primitive)=>{
        get().addPrimitiveId(primitive.uuid, program.uuid)
      })

    },
    // Program-level updates
    setName: (text) => set((_)=>({name:text})),
    setUuid: (text) => set((_)=>({uuid:text})),
    setType: (text) => set((_)=>({type:text})),
    setDescription: (text) => set((_)=>({description:text})),
    setTransform: (x,y) => set((_)=>({transform:{x,y}})),
    insertPrimitiveId: (primitiveId, parentId, index) => set((state)=>{
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.primitiveIds.splice(index,0,primitiveId)
      } else {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds.splice(index,0,primitiveId)
      }
    }),
    addPrimitiveId: (primitiveId, parentId) => set((state)=>{
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.primitiveIds.push(primitiveId)
      } else {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds.push(primitiveId)
      }
    }),
    deletePrimitiveId: (primitiveId, parentId) => set((state)=>{
      if (parentId === state.uuid) {
        // This is the top-level program, so remove from top-level set of primitiveIds
        state.primitiveIds = state.primitiveIds.filter(id=>id!==primitiveId)
      } else {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds = state.data.primitives[parentId].primitiveIds.filter(id=>id!==primitiveId)
      }
    }),
    movePrimitiveId: (primitiveId, parentId, index) => set((state)=>{
      // remove from all locations
      state.primitiveIds = state.primitiveIds.filter(id=>id!==primitiveId)
      Object.keys(state.data.primitives).forEach(parentId=>{
        if (state.data.primitives[parentId].type.includes('hierarchical')) {
          state.data.primitives[parentId].primitiveIds = state.data.primitives[parentId].primitiveIds.filter(id=>id!==primitiveId)
        }
      })
      Object.keys(state.data.skills).forEach(parentId=>{
        state.data.skills[parentId].primitiveIds = state.data.skills[parentId].primitiveIds.filter(id=>id!==primitiveId)
      })
      // add into the correct location
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.primitiveIds.splice(index,0,primitiveId)
      } else {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds.splice(index,0,primitiveId)
      }
    }),
    setPrimitiveIds: (primitiveIds, parentId) => set((state)=>{
      if (parentId === state.uuid) {
        // This is the top-level program, so add to top level list of uuids
        state.primitiveIds = primitiveIds
      } else {
        // This is some other child of the program, so look it up and edit in data
        state.data.primitives[parentId].primitiveIds = primitiveIds
      }
    }),
    // Piecewise update functions for data
    addItem: (type, item) => set((state)=>{
      state.data[typeToKey(type)][item.uuid] = item
    }),
    setItemProperty: (type, uuid, property, value) => set((state)=>{
      state.data[typeToKey(type)][uuid][property] = value
    }),
    deleteItem: (type, uuid) => set((state)=>{
      delete state.data[typeToKey(type)][uuid]
    }),
    getBlocklySkills: () => ({}),
    getBlocklyPrimitives: () =>({})
});

const useEvdStore = create(immer(store));

console.log(fakeEvdData.arbitrary.program)
// Remove for ROS-based updates later (useful for frontend dev)
useEvdStore.getState().setProgram(fakeEvdData.arbitrary.program)

console.log('Primitives');
console.log(useEvdStore.getState().data.primitives);
console.log('Skills');
console.log(useEvdStore.getState().data.skills);
console.log('Top-level primitives');
console.log(useEvdStore.getState().primitiveIds);

export default useEvdStore;