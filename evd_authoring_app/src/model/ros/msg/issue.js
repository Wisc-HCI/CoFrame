import ROSLIB from '@robostack/roslib';


export const Issue = (source='', id='', level='', data='', humanMsg='') => {
    return new ROSLIB.Message(IssueUnwrapped(source, id, level, data, humanMsg));
};

export const IssueUnwrapped = (source='', id='', level='', data='', humanMsg='') => {
    return {
        source,
        id,
        level,
        data,
        human_msg: humanMsg
    }
};