import create from "zustand";
import produce from "immer";
import fakeEvdData from './fakeEvdData';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const typeToKey = (type) => {
  let key;
  switch(type) {
    case 'trajectory':
      key = 'trajectories';
      break;
    default:
      key = type + 's'
  }
  return key;
}

const store = (set) => ({
    updateProgram: ({data,action,changes,currentTag,previousTag})=> set((state)=>{
      const dataBlob = JSON.parse(data);
      const changesBlob = JSON.parse(changes);
      console.log(dataBlob);
      console.log(changesBlob);

      // {
      //   data:str (json blob ~ probably your program), 
      //   action:str, 
      //   changes:str (json blob), 
      //   currentTag:Version, 
      //   previousTag:Version
      // }
      // Version: {
      //   timestamp: time (ros time: sec, nsec),
      //   uuid: str,
      //   source: str
      // }
      state.setProgram(dataBlob);
    }),
    setProgram: (program) => set((state)=>{      
      program.locations.forEach((location)=>{
        state.addItem('location',location)
      })


      program.waypoints.forEach((waypoint)=>{
        useEvdStore.getState().addItem('waypoint',waypoint)
      })
      fakeEvdData.arbitrary.machines.forEach((machine)=>{
        useEvdStore.getState().addItem('machine',machine)
      })
      fakeEvdData.arbitrary.thingTypes.forEach((thingType)=>{
        useEvdStore.getState().addItem('thingType',thingType)
      })
      fakeEvdData.arbitrary.things.forEach((thing)=>{
        useEvdStore.getState().addItem('thing',thing)
      })
      fakeEvdData.arbitrary.regions.forEach((region)=>{
        useEvdStore.getState().addItem('region',region)
      })
    }),
    data: {
      //TODO store other data

      locations: {},
      waypoints: {},
      machines: {},
      thingTypes: {},
      things: {},      // Only shown through thingTypes
      regions: {},      // Only shown through machines (I added regions back in after reworking machines)
      primitives: {}, //lookup table (of flattened hierarchicals)
      // for hierarchical primitives, extract children from primtives list, replace with uuid
      skills: {}, // skills are not blockly! (Designers can add new skills)
    },
    top_level_ordered: [], // ordered list of uuids, with hierarchcials being an inner list
    /*
    [
      'uuid1',
      {
        'h-uuid'
      }
    ]
    */
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

/*
fakeEvdData.arbitrary.locations.forEach((location)=>{
  useEvdStore.getState().addItem('location',location)
})
fakeEvdData.arbitrary.waypoints.forEach((waypoint)=>{
  useEvdStore.getState().addItem('waypoint',waypoint)
})
fakeEvdData.arbitrary.machines.forEach((machine)=>{
  useEvdStore.getState().addItem('machine',machine)
})
fakeEvdData.arbitrary.thingTypes.forEach((thingType)=>{
  useEvdStore.getState().addItem('thingType',thingType)
})
fakeEvdData.arbitrary.things.forEach((thing)=>{
  useEvdStore.getState().addItem('thing',thing)
})
fakeEvdData.arbitrary.regions.forEach((region)=>{
  useEvdStore.getState().addItem('region',region)
})
*/

useEvdStore.getState().setProgram(fakeEvdData.arbitrary.program)

console.log(useEvdStore.getState().environment);

export default useEvdStore;