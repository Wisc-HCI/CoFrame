import create from "zustand";
import ROSLIB from '@robostack/roslib';

const store = (set) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set((state)=>{
        const ros = new ROSLIB.Ros({url:url});
        ros.on('connection', state.onConnection);
        ros.on('error', state.onError);
        ros.on('close', state.onClose);

        const loadAppSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/load_application_data',
            serviceType: 'evd_ros_core/LoadData'
        });
    
        const saveAppSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/save_application_data',
            serviceType: 'evd_ros_core/SaveData'
        });
    
        const getAppOptionsSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/get_application_options',
            serviceType: 'evd_ros_core/GetOptions'
        });

        const getProgramSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/get_data',
            serviceType: 'evd_ros_core/GetData'
        });
    
        const setProgramSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/set_data',
            serviceType: 'evd_ros_core/SetData'
        });

        const updateProgramTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'data_server/update',
            messageType: 'evd_ros_core/UpdateData'
        });
        console.log(url);
        console.log(ros);
        return {
            url:url,
            connection:'disconnected',
            ros:ros,
            loadAppSrv:loadAppSrv, 
            saveAppSrv:saveAppSrv, 
            getAppOptionsSrv:getAppOptionsSrv,
            getProgramSrv:getProgramSrv,
            setProgramSrv:setProgramSrv,
            updateProgramTopic:updateProgramTopic
        };
    }),
    ros: null,
    connection: 'disconnected',
    loadAppSrv: null,
    saveAppSrv: null,
    getApOptionsSrv: null,
    getProgramSrv: null,
    setProgramSrv: null,
    updateProgramTopic: null,
    subscribeToProgramTopic: (fn) => set((state)=>{
        state.updateProgramTopic.subscribe(fn)
    }),
    onConnection: () => set({connection:'connected'}),
    onError: () => set({connection:'disconnected'}),
    onClose: () => set({connection:'disconnected'}),
    connect: () => set((state)=>{
        console.log(state.ros);
        state.ros.connect();
        return {connection:'connecting'}
    })
});

const useRosStore = create(store);

useRosStore.getState().setUrl('ws://localhost:9090');


export default useRosStore;