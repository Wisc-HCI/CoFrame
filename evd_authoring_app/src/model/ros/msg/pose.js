import ROSLIB from 'roslib';
import { HeaderUnwrapped } from './header';

export const PositionUnwrapped = (x=0, y=0, z=0) => {
    return {
        x,
        y,
        z
    };
};

export const QuaternionUnwrapped = (x=0, y=0, z=0, w=0) => {
    return {
        x,
        y,
        z,
        w
    };
};

export const PoseUnwrapped = (positionUnwrapped=null, orientationUnwrapped=null) => {
    if (positionUnwrapped === null) {
        positionUnwrapped = PositionUnwrapped();
    }

    if (orientationUnwrapped === null) {
        orientationUnwrapped = QuaternionUnwrapped();
    }

    return {
        position: positionUnwrapped,
        orientation: orientationUnwrapped
    };
};

export const Pose = (positionUnwrapped=null, orientationUnwrapped=null) => {
    return new ROSLIB.Message(PoseUnwrapped(positionUnwrapped, orientationUnwrapped));
};

export const PoseStampedUnwrapped = (poseUnwrapped=null, headerUnwrapped=null) => {
    if (poseUnwrapped === null) {
        poseUnwrapped = PoseUnwrapped();
    }

    if (headerUnwrapped === null) {
        headerUnwrapped = HeaderUnwrapped();
    }

    return {
        header: headerUnwrapped,
        pose: poseUnwrapped
    };
};

export const PoseStamped = (poseUnwrapped=null, headerUnwrapped=null) => {
    return new ROSLIB.Message(PoseStampedUnwrapped(poseUnwrapped, headerUnwrapped));
}