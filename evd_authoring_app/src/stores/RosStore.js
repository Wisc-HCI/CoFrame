import create from "zustand";
import ROSLIB from '@robostack/roslib';

import useEvdStore from './EvdStore';
import useSimStore from './SimStore';

const store = (set) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set((_)=>({url:url,connection:'disconnected'})),
    ros: null,
    connection: 'disconnected',
    loadAppSrv: null,
    saveAppSrv: null,
    getApOptionsSrv: null,
    getProgramSrv: null,
    setProgramSrv: null,
    updateProgramTopic: null,
    updateTfsTopic: null,
    onConnection: () => set({connection:'connected'}),
    onError: () => set({connection:'disconnected'}),
    onClose: () => set({connection:'disconnected'}),
    connect: () => set((state)=>{
        const ros = new ROSLIB.Ros({url:state.url});
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

        const updateTfsTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'tf',
            messageType: 'tf2_msgs/TFMessage'
        });

        const updateFn = useSimStore.getState().updateFromTfs;
        updateProgramTopic.subscribe(useEvdStore.getState().updateProgram);
        // updateTfsTopic.subscribe(useSimStore.getState().updateFromTfs)
        updateTfsTopic.subscribe(updateFn);

        ros.connect();
        return {
            url:state.url,
            connection:'connecting',
            ros:ros,
            loadAppSrv:loadAppSrv, 
            saveAppSrv:saveAppSrv, 
            getAppOptionsSrv:getAppOptionsSrv,
            getProgramSrv:getProgramSrv,
            setProgramSrv:setProgramSrv,
            updateProgramTopic:updateProgramTopic,
            updateTfsTopic:updateTfsTopic
        };
    })
});

const useRosStore = create(store);

useRosStore.getState().setUrl('ws://localhost:9090');
useSimStore.getState().setup();


export default useRosStore;