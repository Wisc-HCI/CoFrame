import ROSLIB from 'roslib';
import {
    
} from '../model/ros';

class ApplicationService {

    constructor() {

        this._filename = "Untitled";
        this._filenameChanged = false;
        this._stateSetCallback = null;

        this.loadAppSrv = null;
        this.saveAppSrv = null;
        this.getAppOptionsSrv = null;
    }

    setup(ros) {

        this.loadAppSrv = null;

        this.saveAppSrv = null;

        this.getAppOptionsSrv = null;

    }

    loadFromFile(data, filename) { //data is a JSON object
        // Set data in EvD script service and commit to server

        // Update filename
        this._filename = filename;
        this._filenameChanged = true;

        // Save application to file on server

        // Propogate app changes through frontend
        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    saveToFile() {
        // Get data from EvD script service

    }

    teardown() {
        this.loadAppSrv = null;
        this.saveAppSrv = null;
        this.getAppOptionsSrv = null;
    }

    get filename() {
        return this._filename;
    }

    set filename(value) {
        this._filename = value;
        this._filenameChanged = true;
        
        if (this._stateSetCallback !== null) {
            this._stateSetCallback(this.state);
        }
    }

    get stateSetCallback() {
        return this._stateSetCallback;
    }

    set stateSetCallback(value) {
        this._stateSetCallback = value;
    }

    get state() {
        return {
            filename: this._filename,
            filenameHasChanged: this._filenameChanged
        };
    }
}

let singleton = null;
export const GetApplicationServiceSingleton = () => {
    if (singleton === null) {
        singleton = new ApplicationService();
    }

    return singleton;
};