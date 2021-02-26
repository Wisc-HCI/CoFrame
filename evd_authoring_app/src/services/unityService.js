
class UnityService {

    constructor() {

        // Unity Frame Linking
        this.rosFramePub = null;
        this.cameraPoseFramePub = null;
        this.controlTargetPosePub = null;

        // Unity Robot Controls
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

    setup(ros) {
        
    }

}

let singleton = null;
export const GetUnityServiceSingleton = () => {
    if (singleton === null) {
        singleton = new UnityService();
    }

    return singleton;
};