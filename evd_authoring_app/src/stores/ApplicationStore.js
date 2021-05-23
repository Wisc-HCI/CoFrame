import create from "zustand";
import produce from "immer";
import useRosStore from './RosStore';
import useEvdStore from './EvdStore';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
    filename: 'Untitled',
    filenameChanged: false,
    setFilename: (name) => set((_)=>({filename:name,filenameChanged:true})),
    load: () => {
      const loadAppSrv = useRosStore.getState().loadAppSrv;
      if (loadAppSrv !== null) {
        loadAppSrv(null,(result)=>{
          const setProgram = useEvdStore.getState().setProgram;
          setProgram(result)
        })
      }
    },
    save: () => {
      const saveAppSrv = useRosStore.getState().saveAppSrv;
      if (saveAppSrv !== null) {
        saveAppSrv({use_current_info:true},(_)=>{})
      }
    }
});

const useApplicationStore = create(immer(store));

export default useApplicationStore;