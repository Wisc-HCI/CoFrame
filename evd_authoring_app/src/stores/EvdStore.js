import create from "zustand";
import produce from "immer";
import fakeEvdData from './fakeEvdData';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

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
    environment: {
      locations: {},
      waypoints: {},
      machines: {},
      thingTypes: {},
      things: {}      // Only shown through thingTypes
    },
    addItem: (type, item) => set((state)=>{
      state.environment[type+'s'][item.uuid] = item
    }),
    setItemProperty: (type, uuid, property, value) => set((state)=>{
      state.environment[type+'s'][uuid][property] = value
    }),
    deleteItem: (type, uuid) => set((state)=>{
      delete state.environment[type+'s'][uuid]
    }),
    primitives: {},
    setProgram: (program) => set((_)=>({program:program}))
});

const useEvdStore = create(immer(store));

fakeEvdData.locations.forEach((location)=>{
  useEvdStore.getState().addItem('location',location)
})
fakeEvdData.waypoints.forEach((waypoint)=>{
  useEvdStore.getState().addItem('waypoint',waypoint)
})
fakeEvdData.machines.forEach((machine)=>{
  useEvdStore.getState().addItem('machine',machine)
})
fakeEvdData.thingTypes.forEach((thingType)=>{
  useEvdStore.getState().addItem('thingType',thingType)
})
fakeEvdData.things.forEach((thing)=>{
  useEvdStore.getState().addItem('thing',thing)
})

console.log(useEvdStore.getState().environment);

export default useEvdStore;