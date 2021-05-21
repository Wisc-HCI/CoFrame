import ROSLIB from '@robostack/roslib';

export const GetOptionsRequestUnwrapped = () => {
    return {};
};

export const GetOptionsRequest = () => {
    return new ROSLIB.ServiceRequest(GetOptionsRequestUnwrapped());
};