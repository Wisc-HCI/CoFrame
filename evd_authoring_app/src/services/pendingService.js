import ROSLIB from 'roslib';
import {
    GetIssuesRequest,
    GetPendingRequest
} from '../model/ros';


/**
 * Not wed to this structure of Context, Service, and local State
 * It might be better to convert all these services into a singular large store
 */

class PendingService {

    constructor() {

        this.getIssuesSrv = null;
        this.getPendingJobsSrv = null;

        this._pendingJobs = null;
        this._issues = null;
    }

    setup(ros) {

        this.getIssuesSrv = new ROSLIB.Service({
            ros: ros,
            name: 'issue_server/get_issues',
            serviceType: 'evd_ros_core/GetIssues'
        });
        
        this.getPendingJobsSrv = new ROSLIB.Service({
            ros: ros,
            name: 'issue_server/get_pending_jobs',
            serviceType: 'evd_ros_core/GetPendingJobs'
        });
    }

    teardown() {
        this.getIssuesSrv = null;
        this.getPendingJobsSrv = null;
    }

    get pendingJobs() {
        return this._pendingJobs;
    }

    get issues() {
        return this._issues;
    }

    getIssues() { // Trigger an update on issues in state
        const request = GetIssuesRequest();
        
        this.getIssuesSrv.callService(request, (result) => {
            this._issues = result.issues;
            if (this._stateSetCallback !== null) {
                this._stateSetCallback(this.state);
            }
        });
    }

    getPendingJobs() { // Trigger an update on pending jobs in state
        const request = GetPendingRequest();

        this.getPendingJobs(request, (result) => {

            let jobs = {};
            for (let i=0; i<result.sources.length; i++) {
                const src = result.sources[i];
                jobs[src] = [];
                
                for (let j=0; j<result.ids[i].data.length; j++) {
                    jobs[src].push({
                        source: src,
                        id: result.id[i].data[j],
                        humanMsg: result.human_message[i].data[j],
                        data: JSON.parse(result.data[i].data[j]),
                    });
                }
            }

            this._pendingJobs = jobs;
            if (this._stateSetCallback !== null) {
                this._stateSetCallback(this.state);
            }
        });
    }

    get state() {
        return {
            pendingJobs: this._pendingJobs,
            issues: this._issues
        }
    }
}

let singleton = null;
export const GetPendingServiceSingleton = () => {
    if (singleton === null) {
        singleton = new PendingService();
    }

    return singleton;
};