import ROSLIB from 'roslib';

export const GetOptionsRequestUnwrapped = () => {
    return {};
};

export const GetOptionsRequest = () => {
    return new ROSLIB.ServiceRequest(GetOptionsRequestUnwrapped());
};