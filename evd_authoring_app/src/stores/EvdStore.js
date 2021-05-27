import create from "zustand";
import produce from "immer";
import useRosStore from './RosStore';

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
    environment: {locations: {},waypoints: {}},
    primitives: {},
    setProgram: (program) => set((_)=>({program:program}))
});



const useEvdStore = create(immer(store));

export default useEvdStore;