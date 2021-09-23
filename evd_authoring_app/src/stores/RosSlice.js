import ROSLIB from '@robostack/roslib';
// import { generateUuid } from './generateUuid';
import { typeToKey } from './helpers';

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
        const {joint, trace} = JSON.parse(msg.data);
        const {pybullet_frame_data} = trace;
        if (state.data.locations[msg.id]) {
            state.data.locations[msg.id].frames = pybullet_frame_data;
            state.data.locations[msg.id].joints = joint;
        } else if (state.data.waypoints[msg.id]) {
            state.data.waypoints[msg.id].frames = pybullet_frame_data;
            state.data.waypoints[msg.id].joints = joint;
        }
    }),
    updateTraceFromProcessor: (msg) => set((state) => {
        const {trace} = JSON.parse(msg.data);
        const {trajectory,duration,pybullet_joint_velocities,pybullet_collisions,pybullet_pinchpoints,pybullet_frame_data} = trace;
        if (state.data.trajectories[trajectory.uuid]) {
            state.data.trajectories[trajectory.uuid].trace = {
                duration,
                joint_velocities:pybullet_joint_velocities,
                collisions:pybullet_collisions,
                pinchpoints:pybullet_pinchpoints,
                frames:pybullet_frame_data
            }
        }
    }),
    requestJointProcessorUpdate: (type, uuid) => {
        const poseInfo = get().data[typeToKey(type)][uuid];
        const msg = {id:uuid,data:JSON.stringify({point:poseInfo})};
        if (get().connection === 'connected') {
            get().jointProcessorRequestTopic.publish(msg);
        } else {
            console.log('Disregarding processor request due to null ROS connection')
        }
        
    },
    requestJTraceProcessorUpdate: (uuid) => {
        const trajectoryInfo = get().data.trajectories[uuid];
        let points = [];
        if (trajectoryInfo.start_location_uuid) {
            points.push(get().data.locations[trajectoryInfo.start_location_uuid])
        }
        trajectoryInfo.waypoint_uuids.forEach(waypoint_uuid=>{
            points.push(get().data.waypoints[waypoint_uuid])
        })
        if (trajectoryInfo.end_location_uuid) {
            points.push(get().data.locations[trajectoryInfo.end_location_uuid])
        }
        const msg = {id:uuid,data:JSON.stringify({
            trajectory:trajectoryInfo,
            points
        })};
        if (get().connection === 'connected') {
            get().traceProcessorRequestTopic.publish(msg);
        } else {
            console.log('Disregarding processor request due to null ROS connection')
        }
    }
});