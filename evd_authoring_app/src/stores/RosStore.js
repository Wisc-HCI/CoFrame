import create from "zustand";
import produce from "immer";
import ROSLIB from '@robostack/roslib';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);



const store = (set, get) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set({url:url,connected:false,ros:null,loadAppSrv:null, saveAppSrv:null, getAppOptionsSrv:null}),
    ros: null,
    connected: false,
    loadAppSrv: null,
    saveAppSrv: null,
    getApOptionsSrv: null,
    onConnection: ()=>{},
    onError: ()=>{},
    onClose: ()=>{},
    connect: async () => {
        const ros = new ROSLIB.Ros();
        ros.on('connection', get().onConnection);
        ros.on('error', get().onError);
        ros.on('close',get().onClose);

        await ros.connect({url: get().url})
        set({ros: ros})
    }
});

const useRosStore = create(immer(store));

const onConnection = () => {
    const ros = useRosStore.getState().ros;
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
    useRosStore.setState({connected:true,loadAppSrv:loadAppSrv, saveAppSrv:saveAppSrv, getAppOptionsSrv:getAppOptionsSrv});
    window.alert('ROS is now connected');
}

const onError = (error) => {
    useRosStore.setState({connected:false});
    window.alert('ROS encountered an error!');
    console.log('ros connection encountered an error', error);
}

const onClose = () => {
    useRosStore.setState({connected:false});
    window.alert('ROS connection is closed');
}

useRosStore.setState({onConnection:onConnection,onError:onError,onClose:onClose})

export default useRosStore;