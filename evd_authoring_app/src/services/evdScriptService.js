import ROSLIB from 'roslib';
import {
    
} from '../model/ros';

class EvDScriptService {

    constructor() {

        this.updateProgramSub = null;
        this.getProgramSrv = null;
        this.setProgramSrv = null;
        this.getDefaultObjectsSrv = null;
    }

    setup(ros) {

        this.updateProgramSub = null;

        this.getProgramSrv = null;

        this.setProgramSrv = null;

        this.getDefaultObjectsSrv = null;

    }

    teardown() {
        this.updateProgramSub = null;
        this.getProgramSrv = null;
        this.setProgramSrv = null;
        this.getDefaultObjectsSrv = null;
    }

}

let singleton = null;
export const GetEvDScriptServiceSingleton = () => {
    if (singleton === null) {
        singleton = new EvDScriptService();
    }

    return singleton;
};

