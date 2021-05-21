import ROSLIB from '@robostack/roslib';

export const LoadDataRequestUnwrapped = (filename='', name='', description='', level=0, custom=true) => {
    return {
        filename,
        name,
        description,
        level,
        custom
    };
};

export const LoadDataRequest = (useCurrentInfo, filename='', name='', description='', level=0, custom=true) => {
    return new ROSLIB.ServiceRequest(LoadDataRequestUnwrapped(useCurrentInfo, filename, name, description, level, custom=true));
};