import ROSLIB from '@robostack/roslib';

export const GetIssuesRequestUnwrapped = (filterBySource=false, source='', ids=[], filterByLevel=false, level=0) => {
    return {
        filter_by_source: filterBySource,
        source,
        ids,
        filter_by_level: filterByLevel,
        level
    };
};

export const GetIssuesRequest = (filterBySource=false, source='', ids=[], filterByLevel=false, level=0) => {
    return new ROSLIB.ServiceRequest(GetIssuesRequestUnwrapped(filterBySource, source, ids, filterByLevel, level));
};