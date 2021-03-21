import ROSLIB from 'roslib';
import {
    GetIssuesRequest,
    GetPendingRequest
} from '../model/ros';


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

    get cachedPendingJobs() {
        return this._pendingJobs;
    }

    get cachedIssues() {
        return this._issues;
    }

    async getIssues(request = null) {
        if (request === null) {
            request = GetIssuesRequest();
        }
        
        return await new Promise((resolve, reject) => {
            this.getIssuesSrv.callService(request, (result) => {
                this._issues = result.issues;
                resolve(this._issues);
            });
        }).catch((err) => {throw err});
    }

    async getPendingJobs() {
        const request = GetPendingRequest();
        
        return await new Promise((resolve, reject) => {
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

                resolve({
                    status: result.status,
                    message: result.message,
                    jobs: this._pendingJobs 
                });
            });
        }).catch((err) => {throw err});
    }

    get state() {
        return null;
    }

}

let singleton = null;
export const GetPendingServiceSingleton = () => {
    if (singleton === null) {
        singleton = new PendingService();
    }

    return singleton;
};