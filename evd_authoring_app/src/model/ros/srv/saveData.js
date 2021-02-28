import ROSLIB from 'roslib';

export const SaveDataRequestUnwrapped = (useCurrentInfo, filename='', name='', description='', level=0) => {
    return {
        use_current_info: useCurrentInfo,
        filename,
        name,
        description,
        level
    };
};

export const SaveDataRequest = (useCurrentInfo, filename='', name='', description='', level=0) => {
    return new ROSLIB.ServiceRequest(SaveDataRequestUnwrapped(useCurrentInfo, filename, name, description, level));
};