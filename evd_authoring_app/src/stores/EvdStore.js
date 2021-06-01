import create from "zustand";
import produce from "immer";
import useRosStore from './RosStore';
import fakeEvdData from './fakeEvdData';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    setup: ()=>set((state)=>{
      useRosStore.getState().subscribeToProgramTopic(get().updateProgram);
    }),
    program: null,
    updateProgram: ({data,action,changes,currentTag,previousTag})=>{
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
      locations: [],
      waypoints: [],
      regions: [],
      machines: [],
      thingTypes: [],
      things: []
    },
    addLocation: (location) => set((state)=>{
      state.environment.locations.push(location)
    }),
    deleteLocation: (uuid) => set((state)=>{
      state.environment.locations = state.environment.locations.filter(item=>item.uuid !== uuid)
    }),
    addWaypoint: (waypoint) => set((state)=>{
      state.environment.waypoints.push(waypoint)
    }),
    deleteWaypoint: (uuid) => set((state)=>{
      state.environment.waypoints = state.environment.waypoints.filter(item=>item.uuid !== uuid)
    }),
    addRegion: (region) => set((state)=>{
      state.environment.regions.push(region)
    }),
    deleteRegion: (uuid) => set((state)=>{
      state.environment.regions = state.environment.regions.filter(item=>item.uuid !== uuid)
    }),
    addMachine: (machine) => set((state)=>{
      state.environment.machines.push(machine)
    }),
    deleteMachine: (uuid) => set((state)=>{
      state.environment.machines = state.environment.machines.filter(item=>item.uuid !== uuid)
    }),
    addThingType: (thingType) => set((state)=>{
      state.environment.thingTypes.push(thingType)
    }),
    deleteThingType: (uuid) => set((state)=>{
      state.environment.thingTypes = state.environment.thingTypes.filter(item=>item.uuid !== uuid)
    }),
    addThing: (thing) => set((state)=>{
      state.environment.things.push(thing)
    }),
    deleteThing: (uuid) => set((state)=>{
      state.environment.things = state.environment.things.filter(item=>item.uuid !== uuid)
    }),

    primitives: {},
    setProgram: (program) => set((_)=>({program:program}))
});

const useEvdStore = create(immer(store));

fakeEvdData.locations.forEach((location)=>{
  useEvdStore.getState().addLocation(location)
})
fakeEvdData.waypoints.forEach((waypoint)=>{
  useEvdStore.getState().addWaypoint(waypoint)
})
fakeEvdData.regions.forEach((region)=>{
  useEvdStore.getState().addRegion(region)
})
fakeEvdData.machines.forEach((machine)=>{
  useEvdStore.getState().addMachine(machine)
})
fakeEvdData.thingTypes.forEach((thingType)=>{
  useEvdStore.getState().addThingType(thingType)
})
fakeEvdData.things.forEach((thing)=>{
  useEvdStore.getState().addThing(thing)
})

console.log(useEvdStore.getState().environment);

export default useEvdStore;