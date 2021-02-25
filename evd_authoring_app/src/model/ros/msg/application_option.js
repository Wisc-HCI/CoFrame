import ROSLIB from 'roslib';

export const ApplicationOption = (filename, name, description, level, custom) => {
    return new ROSLIB.Message(ApplicationOptionUnwrapped(filename,name,description,level,custom));
}

export const ApplicationOptionUnwrapped = (filename, name, description, level, custom) => {
    return {
        filename,
        name,
        description,
        level,
        custom
    };
}