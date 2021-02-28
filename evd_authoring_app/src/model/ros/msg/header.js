import { TimeNowUnwrapped } from './time';

export const HeaderUnwrapped = (seq=0, stamp=null, frameId='') => {
    if (stamp === null) {
        stamp = new TimeNowUnwrapped();
    }

    return {
        seq: seq,
        stamp: stamp,
        frame_id: frameId
    };
};