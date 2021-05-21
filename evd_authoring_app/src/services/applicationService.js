import ROSLIB from '@robostack/roslib';
import {
    
} from '../model/ros';

class ApplicationService {

    constructor() {

        this._filename = "Untitled";
        this._filenameChanged = false;
        this._stateSetCallback = null;
        this._fileOptions = [];

        this.loadAppSrv = null;
        this.saveAppSrv = null;
        this.getAppOptionsSrv = null;
    }

    setup(ros) {

        this.loadAppSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/load_application_data',
            serviceType: 'evd_ros_core/LoadData'
        });

        this.saveAppSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/save_application_data',
            serviceType: 'evd_ros_core/SaveData'
        });

        this.getAppOptionsSrv = new ROSLIB.Service({
            ros: ros,
            name: 'data_server/get_application_options',
            serviceType: 'evd_ros_core/GetOptions'
        });
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

    loadFromOption(optionName) {
        // Command server to load backend option
    }

    saveToFile() {
        // Get data from EvD script service

        // Save application to file on server
    }

    getOptions() {
        const request = null;
        // Triggers an update on the options available on server
        this.getAppOptionsSrv(request, (result) => {
            
        });
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

    get fileOptions() {
        return this._fileOptions;
    }

    get state() {
        return {
            filename: this._filename,
            filenameHasChanged: this._filenameChanged,
            fileOptions: this._fileOptions
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