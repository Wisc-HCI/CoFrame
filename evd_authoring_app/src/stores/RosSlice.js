import ROSLIB from '@robostack/roslib';
// import { generateUuid } from './generateUuid';
import { typeToKey } from './helpers';

export const RosSlice = (set, get) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set((_) => ({ url: url, connection: 'disconnected' })),
    ros: null,
    connection: 'disconnected',
    updateProgramTopic: null,
    // updateTfsTopic: null,
    configureProgramProcessorsTopic: null,
    jointProcessorRequestTopic: null,
    jointProcessorResponseTopic: null,
    traceProcessorRequestTopic: null,
    traceProcessorResponseTopic: null,
    onConnection: () => {
        set((_) => ({ connection: 'connected' }))
        const configuration = {
            occupancy_zones:Object.values(get().data.occupancyZones).filter(zone => zone.occupancy_type === "human"),
            collision_meshes:Object.values(get().data.collisionMeshes),
            pinch_points:[]
        }
        console.log(configuration)
        get().configureProgramProcessorsTopic.publish({data:JSON.stringify(configuration)});
        const bulkRequest = get().doBulkJointRequest
        setTimeout(bulkRequest(),5000);
    },
    onError: () => set((_) => ({ connection: 'disconnected' })),
    onClose: () => set((_) => ({ connection: 'disconnected' })),
    connect: () => {
        const ros = new ROSLIB.Ros({ url: get().url });
        ros.on('connection', get().onConnection);
        ros.on('error', get().onError);
        ros.on('close', get().onClose);

        const updateProgramTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/update',
            messageType: 'std_msgs/String'
        });

        // const updateTfsTopic = new ROSLIB.Topic({
        //     ros: ros,
        //     name: 'tf',
        //     messageType: 'tf2_msgs/TFMessage'
        // });

        const configureProgramProcessorsTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'program/configure/processors',
            messageType: 'std_msgs/String'
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

        // const updateTfsFn = get().updateFromTfs;
        const handleJointProcessorResponseFn = get().updateJointFromProcessor;
        const handleTraceProcessorResponseFn = get().updateTraceFromProcessor;
        // updateTfsTopic.subscribe(updateTfsFn);
        jointProcessorResponseTopic.subscribe(handleJointProcessorResponseFn)
        traceProcessorResponseTopic.subscribe(handleTraceProcessorResponseFn)

        ros.connect();
        set(() => ({
            connection: 'connecting',
            ros,
            configureProgramProcessorsTopic,
            jointProcessorRequestTopic,
            jointProcessorResponseTopic,
            traceProcessorRequestTopic,
            traceProcessorResponseTopic,
            updateProgramTopic,
            // updateTfsTopic
        }))
    },
    updateJointAndTraceData: (uuid, joint, trace) => set(state => {
        const { pybullet_frame_data } = trace;
        if (state.data.locations[uuid]) {
            state.data.locations[uuid].frames = pybullet_frame_data;
            state.data.locations[uuid].joints = joint;
        } else if (state.data.waypoints[uuid]) {
            state.data.waypoints[uuid].frames = pybullet_frame_data;
            state.data.waypoints[uuid].joints = joint;
        }
    }),
    updateJointFromProcessor: (msg) => {
        const { joint, trace, status } = JSON.parse(msg.data);
        console.log('receiving joint data from processor')
        console.log({ joint, trace })
        if (status) {
            get().updateJointAndTraceData(msg.id, joint, trace);
            Object.values(get().data.trajectories).forEach(trajectory => {
                if (trajectory.start_location_uuid === msg.id ||
                    trajectory.end_location_uuid === msg.id ||
                    trajectory.waypoint_uuids.includes(msg.id)) {
                    get().requestTraceProcessorUpdate(trajectory.uuid)
                }
            })
        }

    },
    updateTraceFromProcessor: (msg) => set((state) => {
        const { input, trace, status } = JSON.parse(msg.data);
        console.log('receiving trace data from processor')
        console.log({input,trace,status})
        if (status) {
            const { 
                duration, time_data, in_timeout, pybullet_joint_data, 
                postprocess_collisions, postprocess_self_collisions, 
                pybullet_frame_data, postprocess_occupancy} = trace;
            if (state.data.trajectories[msg.id]) {
                state.data.trajectories[msg.id].trace = {
                    duration,
                    time_data,
                    in_timeout,
                    joint_data:pybullet_joint_data,
                    frames:pybullet_frame_data,
                    env_collisions:postprocess_collisions,
                    self_collisions:postprocess_self_collisions,
                    occupancy:postprocess_occupancy,
                    pinch_points:null
                }
            }
        }
    }),
    requestJointProcessorUpdate: (type, uuid, bypass_connection_check=false) => {
        const poseInfo = get().data[typeToKey(type)][uuid];
        const msg = { id: uuid, data: JSON.stringify({ point: poseInfo }) };
        if (get().connection === 'connected' || bypass_connection_check) {
            console.log('Requesting processing for joint ' + uuid)
            get().jointProcessorRequestTopic.publish(msg);
        } else {
            console.log('Disregarding processor request due to null ROS connection')
        }
    },
    doBulkJointRequest: () => {
        Object.keys(get().data.locations).forEach(location => get().requestJointProcessorUpdate('location', location, true));
        Object.keys(get().data.waypoints).forEach(waypoint => get().requestJointProcessorUpdate('waypoint', waypoint, true))
    },
    requestTraceProcessorUpdate: (uuid) => {
        let trajectoryInfo = { ...get().data.trajectories[uuid] };
        trajectoryInfo.trace = null;
        let points = [];
        if (trajectoryInfo.start_location_uuid) {
            const location = get().data.locations[trajectoryInfo.start_location_uuid]
            if (!location.joints?.reachable) {
                return
            }
            points.push(location)
        } else {
            return
        }
        trajectoryInfo.waypoint_uuids.forEach(waypoint_uuid => {
            const waypoint = get().data.waypoints[waypoint_uuid];
            if (!waypoint.joints?.reachable) { return };
            points.push(waypoint)
        })
        if (trajectoryInfo.end_location_uuid) {
            const location = get().data.locations[trajectoryInfo.end_location_uuid]
            if (!location.joints?.reachable) { return }
            points.push(location)
        } else {
            return
        }
        const msg = {
            id: uuid, data: JSON.stringify({
                trajectory: trajectoryInfo,
                points
            })
        };
        if (get().connection === 'connected') {
            console.log('Requesting processing for trajectory ' + uuid)
            get().traceProcessorRequestTopic.publish(msg);
        } else {
            console.log('Disregarding processor request due to null ROS connection')
        }
    }
});