import ROSLIB from '@robostack/roslib';

export const RosSlice = (set,get) => ({
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
    onConnection: () => set((_)=>({connection:'connected'})),
    onError: () => set((_)=>({connection:'disconnected'})),
    onClose: () => set((_)=>({connection:'disconnected'})),
    connect: () => {
        const ros = new ROSLIB.Ros({url:get().url});
        ros.on('connection', get().onConnection);
        ros.on('error', get().onError);
        ros.on('close', get().onClose);

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

        const updateTfsFn = get().updateFromTfs;
        updateTfsTopic.subscribe(updateTfsFn);

        ros.connect();
        set(()=>({
            connection:'connecting',
            ros:ros,
            loadAppSrv:loadAppSrv, 
            saveAppSrv:saveAppSrv, 
            getAppOptionsSrv:getAppOptionsSrv,
            getProgramSrv:getProgramSrv,
            setProgramSrv:setProgramSrv,
            updateProgramTopic:updateProgramTopic,
            updateTfsTopic:updateTfsTopic
        }))
    }
});