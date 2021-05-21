import ROSLIB from '@robostack/roslib';

import { GetApplicationServiceSingleton } from './applicationService';
import { GetEvDScriptServiceSingleton } from './evdScriptService';
import { GetPendingServiceSingleton } from './pendingService';
import { GetUnityServiceSingleton } from './unityService';

class RosService {

    constructor() {

        this._url = 'ws://localhost:9090';
        this._ros = null;
        this._connected = false;
        this._loadCb = null;
        this._stateSetCallback = null;

        this.onLoad = this.onLoad.bind(this);
        this.onRosConnection = this.onRosConnection.bind(this);
        this.onRosClose = this.onRosClose.bind(this);
        this.onRosError = this.onRosError.bind(this);
    }

    onLoad(url='ws://localhost:9090', cb=null) {
        this._loadCb = cb;
        this._url = url;
        this._ros = new ROSLIB.Ros({ url });
        this._connected = false;

        this._ros.on('connection', this.onRosConnection);
        this._ros.on('error', this.onRosError);
        this._ros.on('close',this.onRosClose);

        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    onRosConnection() {
        window.alert('ROS is now connected');
        this._connected = true;

        GetApplicationServiceSingleton().setup(this._ros);
        GetEvDScriptServiceSingleton().setup(this._ros);
        GetPendingServiceSingleton().setup(this._ros);
        GetUnityServiceSingleton().setup(this._ros);

        if (this._loadCb !== null) {
            this._loadCb(true);
        }

        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    onRosError(error) {
        window.alert('ROS Connection encountered an error');
        console.log('ros connection encountered an error', error);
        this._connected = false;

        if (this._loadCb !== null) {
            this._loadCb(false);
        }

        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    onRosClose() {
        window.alert('ROS connection is closed');
        this._connected = false;

        GetApplicationServiceSingleton().teardown();
        GetEvDScriptServiceSingleton().teardown();
        GetPendingServiceSingleton().teardown();
        GetUnityServiceSingleton().teardown();

        if (this._loadCb !== null) {
            this._loadCb(false);
        }

        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    get url() {
        return this._url;
    }

    get connected() {
        return this._connected;
    }

    get ros() {
        return this._ros;
    }

    get stateSetCallback() {
        return this._stateSetCallback;
    }

    set stateSetCallback(value) {
        this._stateSetCallback = value;
    }

    get state() {
        return {
            url: this._url,
            connected: this._connected,
            ros: this._ros
        };
    }
}


let singleton = null;
export const GetRosServiceSingleton = () => {
    if (singleton === null) {
        singleton = new RosService();
    }

    return singleton;
};