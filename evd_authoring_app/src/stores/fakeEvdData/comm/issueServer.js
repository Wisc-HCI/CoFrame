/***************************************************************** 
* Issue Server
* - Issues
* - Pending Jobs
*****************************************************************/

const GetIssuesRequest = {
    filter_by_source: false, //tells server to look at source and id fields, just set to false for frontend
    source: '', //some string like "data_server", probably not useful for frontend display
    ids: [], //of strings, again not useful for frontend

    filter_by_level: true, //tells filter to get only a degree of error (though error registration is up to the job register)
    level: 'error' //arbitrary level scheme (dependent on what the registered issues are)
};

const GetIssuesResponse = {
    issues: [
        { // Issues.msg
            source: 'string',
            id: 'string',
            level: 'error', // can be LEVEL_NOTE='note', LEVEL_WARN='warn', LEVEL_ERROR='error'
            data: 'string -> JSON blob', // the json blob is up to the issuer so not sure what this will look like yet
            human_message: 'string' // some human description of the issue at hand
        }
    ]
};

const GetPendingJobsRequest = {}; // purposefully empty

const GetPendingJobrResponse = {
    sources: [], //of strings
    ids: [
        [] // of strings where outer list is sources and inner list is ids
    ],
    human_messages: [
        [] // of strings where outer list is sources and inner list is human readable messages of jobs
    ],
    data: [
        [] // of strings where outer list is sources and inner list is a JSON blob that the job issuer defines
    ],
    status: true, // issue server response status probably will never be false
    message: '' // or error message if status is false
};


//=================================================================

/// Export
const fields = {
    GetIssuesRequest,
    GetIssuesResponse,
    GetPendingJobsRequest,
    GetPendingJobrResponse
};

export default fields;