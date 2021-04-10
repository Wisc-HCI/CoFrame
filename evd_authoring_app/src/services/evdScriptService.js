import ROSLIB from 'roslib';
import {
    
} from '../model/ros';

class EvDScriptService {

    constructor() {

        this._stateSetCallback = null;

        this.updateProgramSub = null;
        this.getProgramSrv = null;
        this.setProgramSrv = null;
        this.getDefaultObjectsSrv = null;

        this._program = null;
    }

    setup(ros) {

        this.updateProgramSub = new ROSLIB.Topic({
            ros: ros,
            name: 'data_server/update',
            messageType: 'evd_ros_core/UpdateData'
        });
        this.updateProgramSub.subscribe((msg) => {console.log(msg)});

        this.getProgramSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/get_data',
            serviceType: 'evd_ros_core/GetData'
        });

        this.setProgramSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/set_data',
            serviceType: 'evd_ros_core/SetData'
        });

        this.getDefaultObjectsSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/get_default_objects',
            serviceType: 'evd_ros_core/GetData'
        });
    }

    teardown() {
        this.updateProgramSub = null;
        this.getProgramSrv = null;
        this.setProgramSrv = null;
        this.getDefaultObjectsSrv = null;
    }

    get stateSetCallback() {
        return this._stateSetCallback;
    }

    set stateSetCallback(value) {
        this._stateSetCallback = value;
    }

    get program() {
        return this._program;
    }

    get state() {
        return {
            program: this._program,
        };
    }

}

let singleton = null;
export const GetEvDScriptServiceSingleton = () => {
    if (singleton === null) {
        singleton = new EvDScriptService();
    }

    return singleton;
};

