import create from "zustand";
import produce from "immer";
import useRosStore from './RosStore';
// import useEvdStore from './EvdStore';
import ROSLIB from '@robostack/roslib';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    filename: 'Untitled',
    filenameChanged: false,
    options: [],
    setFilename: (name) => set((_)=>({filename:name,filenameChanged:true})),
    load: (filename) => {
      const loadAppSrv = useRosStore.getState().loadAppSrv;
      if (loadAppSrv !== null) {
        const request = ROSLIB.ServiceRequest({filename:filename});
        loadAppSrv.callService(request,(response)=>{
          // Could make a toast or message here
          // {status:bool,message:str}
        })
      }
    },
    save: () => {
      const saveAppSrv = useRosStore.getState().saveAppSrv;
      if (saveAppSrv !== null) {
        const request = ROSLIB.ServiceRequest({use_current_info:false,filename:get().filename,name:get().filename,level:0,description:''});
        saveAppSrv.callService(request,(response)=>{
          // Could make a toast or message here
          // {status:bool,message:str}
        })
      }
    },
    getAppOptions: () => {
      const getAppOptionsSrv = useRosStore.getState().getAppOptionsSrv;
      if (getAppOptionsSrv !== null) {
        const request = ROSLIB.ServiceRequest({});
        getAppOptionsSrv.callService(request,(response)=>{
          // Could make a toast or message here
          // {title:str,
          //  description:str,
          //  currently_loaded_filename:str,
          //  status:bool,
          //  message:str,
          //  options: [     // ApplicationOption[]
          //    {
          //      filename:str,
          //      name:str,
          //      description:str,
          //      level:int,
          //      custom:bool // tracks whether there were changes by operator
          //    },
          //    ...
          //  ]
        })
      }
    }
});

const useApplicationStore = create(immer(store));

export default useApplicationStore;