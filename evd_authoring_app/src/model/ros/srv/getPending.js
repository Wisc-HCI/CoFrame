import ROSLIB from '@robostack/roslib';

export const GetPendingRequestUnwrapped = () => {
    return {};
};

export const GetPendingRequest = () => {
    return new ROSLIB.ServiceRequest(GetPendingRequestUnwrapped());
};