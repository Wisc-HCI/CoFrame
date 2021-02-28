import ROSLIB from 'roslib';

export const UpdateData = (data, action, changes, currentTagUnwrapped, previousTagUnwrapped) => {
    return new ROSLIB.Message(UpdateDataUnwrapped(data,action,changes,currentTagUnwrapped,previousTagUnwrapped));
}

export const UpdateDataUnwrapped = (data, action, changes, currentTagUnwrapped, previousTagUnwrapped) => {
    return {
        data,
        action,
        changes,
        currentTag: currentTagUnwrapped,
        previousTag: previousTagUnwrapped
    };
}