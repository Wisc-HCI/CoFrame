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
    program: null,
    updateProgram: ({data,action,changes,currentTag,previousTag})=>{
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
    },
    setProgram: (program) => set((state)=>{
      state.program = program;
      program.locations.forEach((l) => {
        //TODO
      });
      program.waypoints.forEach((w) => {
        //TODO
      });
    }),
    data: {
      locations: {},
      waypoints: {},
      machines: {},
      thingTypes: {},
      things: {},      // Only shown through thingTypes
      regions: {},      // Only shown through machines (I added regions back in after reworking machines)
      primitives: [],
      skills: {},
    },
    addItem: (type, item) => set((state)=>{
        state.data[typeToKey(type)][item.uuid] = item
    }),
    setItemProperty: (type, uuid, property, value) => set((state)=>{
      state.data[typeToKey(type)][uuid][property] = value
    }),
    deleteItem: (type, uuid) => set((state)=>{
      delete state.data[typeToKey(type)][uuid]
    })
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