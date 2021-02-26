import ROSLIB from 'roslib';
import {
    
} from '../model/ros';


class RosService {

    constructor() {

        this._url = null;
        this.ros = null;
        this._connected = false;

        this.onLoad = this.onLoad.bind(this);
        this.onRosConnection = this.onRosConnection.bind(this);
        this.onRosClose = this.onRosClose.bind(this);
        this.onRosError = this.onRosError.bind(this);
    }

    onLoad(url='ws://localhost:9090') {
        
        this.url = url;
        this.ros = new ROSLIB.Ros({ url });
        this.connected = false;

        this.ros.on('connection', this.onRosConnection);
        this.ros.on('error', this.onRosError);
        this.ros.on('close',this.onRosClose);
    }

    onRosConnection() {
        window.alert('ROS is now connected');
        this.connected = true;
    }

    onRosError(error) {
        window.alert('ROS Connection encountered an error');
        console.log('ros connection encountered an error', error);
        this.connected = false;
    }

    onRosClose() {
        window.alert('ROS connection is closed');
        this.connected = false;
    }

    get url() {
        return this._url;
    }

    set url(value) {
        this._url = value;
    }

    get connected() {
        return this._connected;
    }

    set connected(value) {
        this._connected = value;
    }
}

let singleton = null;
export const GetRosServiceSingleton = () => {
    if (singleton === null) {
        singleton = new RosService();
    }

    return singleton;
};