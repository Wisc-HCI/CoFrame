import ROSLIB from 'roslib';


class UnityService {

    constructor() {

        // Unity Frame Linking
        this.rosFramePub = null;
        this.cameraPoseFramePub = null;
        this.controlTargetPosePub = null;

        // Robot Control Feedback
        this.atStartSub = null;
        this.atEndSub = null;
        this.lockoutSub = null;

        // Unity Robot Controls
        this.useSimulatedRobotPub = null;
        this.usePhysicalRobotPub = null;
        this.freedrivePub = null;
        this.playPub = null;
        this.stopPub = null;
        this.pausePub = null;
        this.resetPub = null;
        this.stepForwardPub = null;
        this.stepBackwardPub = null;

    }

    setup(ros, atStartCB=null, atEndCB=null, lockoutCB=null) {
        
        this.rosFramePub = new ROSLIB.Topic({
            ros: ros,
            name: 'application/ros_frame',
            messageType: 'geometry_msgs/PoseStamped'
        });

        this.cameraPoseFramePub = new ROSLIB.Topic({
            ros: ros,
            name: 'application/camera_pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        this.controlTargetPosePub = new ROSLIB.Topic({
            ros: ros,
            name: 'application/control_target_pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        this.atStartSub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/at_start',
            messageType: 'std_msgs/Bool'
        });
        this.atStartSub.subscribe((atStartCB !== null) ? atStartCB : (msg) => {console.log(msg)});

        this.atEndSub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/at_end',
            messageType: 'std_msgs/Bool'
        });
        this.atEndSub.subscribe((atEndCB !== null) ? atEndCB : (msg) => {console.log(msg)});

        this.lockoutSub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/lockout',
            messageType: 'std_msgs/Bool'
        });
        this.lockoutSub.subscribe((lockoutCB !== null) ? lockoutCB : (msg) => {console.log(msg)});

        this.useSimulatedRobotPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/use_simulated_robot',
            messageType: 'std_msgs/Bool'
        });

        this.usePhysicalRobotPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/use_physical_robot',
            messageType: 'std_msgs/Bool'
        });

        this.freedrivePub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/freedrive',
            messageType: 'std_msgs/Bool'
        });

        this.playPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/play',
            messageType: 'std_msgs/Empty'
        });

        this.stopPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/stop',
            messageType: 'std_msgs/Empty'
        });

        this.pausePub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/pause',
            messageType: 'std_msgs/Empty'
        });;

        this.resetPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/reset',
            messageType: 'std_msgs/Empty'
        });

        this.stepForwardPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/step_forward',
            messageType: 'std_msgs/Empty'
        });

        this.stepBackwardPub = new ROSLIB.Topic({
            ros: ros,
            name: 'robot_control_server/step_backward',
            messageType: 'std_msgs/Empty'
        });
    }

    teardown() {
        this.rosFramePub = null;
        this.cameraPoseFramePub = null;
        this.controlTargetPosePub = null;

        this.atStartSub = null;
        this.atEndSub = null;
        this.lockoutSub = null;

        this.useSimulatedRobotPub = null;
        this.usePhysicalRobotPub = null;
        this.freedrivePub = null;
        this.playPub = null;
        this.stopPub = null;
        this.pausePub = null;
        this.resetPub = null;
        this.stepForwardPub = null;
        this.stepBackwardPub = null;
    }

    publishRosFrame(poseMsg) {
        this.rosFramePub.publish(poseMsg);
    }

    publishCameraFrame(poseMsg) {
        this.cameraPoseFramePub.publish(poseMsg);
    }

    publishControlTargetPose(poseMsg) {
        this.controlTargetPosePub.publish(poseMsg);
    }

    publishUseSimulatedRobot(value=true) {
        this.useSimulatedRobotPub.publish(new ROSLIB.Message({data: value}));
    }

    publishUsePhysicalRobot(value=false) {
        this.usePhysicalRobotPub.publish(new ROSLIB.Message({data: value}));
    }

    publishFreedrive(value=false) {
        this.freedrivePub.publish(new ROSLIB.Message({data: value}));
    }

    publishPlay() {
        this.playPub.publish(new ROSLIB.Message({}));
    }

    publishStop() {
        this.stopPub.publish(new ROSLIB.Message({}));
    }

    publishPause() {
        this.pausePub.publish(new ROSLIB.Message({}));
    }

    publishReset() {
        this.resetPub.publish(new ROSLIB.Message({}));
    }

    publishSteFordward() {
        this.stepForwardPub.publish(new ROSLIB.Message({}));
    }

    publishStepBackward() {
        this.stepBackwardPub.publish(new ROSLIB.Message({}));
    }

}

let singleton = null;
export const GetUnityServiceSingleton = () => {
    if (singleton === null) {
        singleton = new UnityService();
    }

    return singleton;
};