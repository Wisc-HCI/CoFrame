import ROSLIB from 'roslib';
import {
    
} from '../model/ros';


export class EvDScriptService {

    constructor() {

        // General file settings
        this.updateProgramSub = null;
        this.loadAppSrv = null;
        this.saveAppSrv = null;
        this.getAppOptions = null;

        // EvD Script
        this.getProgramSrv = null;
        this.setProgramSrv = null;
        this.getDefaultObjectsSrv = null;

    }

    setup(ros) {

        this.updateProgramSub = null;

        this.loadAppSrv = null;

        this.saveAppSrv = null;

        this.getAppOptions = null;

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