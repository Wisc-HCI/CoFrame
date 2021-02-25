import * as msg from './msg';
import * as srv from './srv';

const {
    ApplicationOption,
    ApplicationOptionUnwrapped,
    UpdateData,
    UpdateDataUnwrapped,
    Version,
    VersionUnwrapped
} = msg;

const {
    GetDataRequest, 
    GetDataRequestUnwrapped,
    GetIssuesRequest, 
    GetIssuesRequestUnwrapped,
    GetOptionsRequest, 
    GetOptionsRequestUnwrapped,
    GetPendingRequest, 
    GetPendingRequestUnwrapped,
    LoadDataRequest, 
    LoadDataRequestUnwrapped,
    SaveDataRequest, 
    SaveDataRequestUnwrapped,
    SetDataRequest, 
    SetDataRequestUnwrapped
} = srv;

export {
    ApplicationOption,
    ApplicationOptionUnwrapped,
    UpdateData,
    UpdateDataUnwrapped,
    Version,
    VersionUnwrapped,
    
    GetDataRequest, 
    GetDataRequestUnwrapped,
    GetIssuesRequest, 
    GetIssuesRequestUnwrapped,
    GetOptionsRequest, 
    GetOptionsRequestUnwrapped,
    GetPendingRequest, 
    GetPendingRequestUnwrapped,
    LoadDataRequest, 
    LoadDataRequestUnwrapped,
    SaveDataRequest, 
    SaveDataRequestUnwrapped,
    SetDataRequest, 
    SetDataRequestUnwrapped
};