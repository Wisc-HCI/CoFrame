import ROSLIB from '@robostack/roslib';

export const Version = (timestamp, uuid, source) => {
    return new ROSLIB.Message(VersionUnwrapped(timestamp, uuid, source));
}

export const VersionUnwrapped = (timestamp, uuid, source) => {
    return {
        timestamp,
        uuid,
        source,
    };
}