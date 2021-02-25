import ROSLIB from 'roslib';

export const GetDataRequestUnwrapped = (all=true, data='') => {
    return {
        all,
        data
    };
};

export const GetDataRequest = (all=true, data='') => {
    return new ROSLIB.ServiceRequest(GetDataRequestUnwrapped(all, data));
};