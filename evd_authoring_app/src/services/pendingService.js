
export class PendingService {

    constructor() {

        this.getIssuesSrv = null;
        this.clearIssueSrv = null;
        this.getPendingJobsSrv = null;
        this.clearPendingJobSrv = null;

    }

    setup(ros) {
        
    }

}

let singleton = null;
export const GetPendingServiceSingleton = () => {
    if (singleton === null) {
        singleton = new PendingService();
    }

    return singleton;
};