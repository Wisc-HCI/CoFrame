import ROSLIB from '@robostack/roslib';

export const RosSlice = (set,get) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set((_)=>({url:url,connection:'disconnected'})),
    ros: null,
    connection: 'disconnected',
    updateProgramTopic: null,
    updateTfsTopic: null,
    jointProcessorRequestTopic: null,
    jointProcessorResponseTopic: null,
    traceProcessorRequestTopic: null,
    traceProcessorResponseTopic: null,
    onConnection: () => set((_)=>({connection:'connected'})),
    onError: () => set((_)=>({connection:'disconnected'})),
    onClose: () => set((_)=>({connection:'disconnected'})),
    connect: () => {
        const ros = new ROSLIB.Ros({url:get().url});
        ros.on('connection', get().onConnection);
        ros.on('error', get().onError);
        ros.on('close', get().onClose);

        const updateProgramTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/update',
            messageType: 'std_msgs/String'
        });

        const updateTfsTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'tf',
            messageType: 'tf2_msgs/TFMessage'
        });

        const jointProcessorRequestTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/request/joints',
            messageType: 'evd_ros_core/Job'
        });

        const jointProcessorResponseTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/submit/joints',
            messageType: 'evd_ros_core/Job'
        })

        const traceProcessorRequestTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/request/trace',
            messageType: 'evd_ros_core/Job'
        });

        const traceProcessorResponseTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/submit/trace',
            messageType: 'evd_ros_core/Job'
        })

        const updateTfsFn = get().updateFromTfs;
        const handleJointProcessorResponseFn = get().updateJointFromProcessor;
        const handleTraceProcessorResponseFn = get().updateTraceFromProcessor;
        updateTfsTopic.subscribe(updateTfsFn);
        jointProcessorResponseTopic.subscribe(handleJointProcessorResponseFn)
        traceProcessorResponseTopic.subscribe(handleTraceProcessorResponseFn)

        ros.connect();
        set(()=>({
            connection:'connecting',
            ros,
            jointProcessorRequestTopic,
            jointProcessorResponseTopic,
            traceProcessorRequestTopic,
            traceProcessorResponseTopic,
            updateProgramTopic,
            updateTfsTopic
        }))
    },
    updateJointFromProcessor: (msg) => set((state) => {
        
    }),
    updateTraceFromProcessor: (msg) => set((state) => {
  
    })
});