import ROSLIB from '@robostack/roslib';

export const SetDataRequestUnwrapped = (data, tagUnwrapped) => {
    return {
        data,
        tag: tagUnwrapped
    };
};

export const SetDataRequest = (data, tagUnwrapped) => {
    return new ROSLIB.ServiceRequest(SetDataRequestUnwrapped(data, tagUnwrapped));
};