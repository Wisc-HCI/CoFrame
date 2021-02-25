import ROSLIB from 'roslib';
import {
    
} from '../model/ros';

import { GetEvDScriptServiceSingleton } from './evd_script_service';
import { GetUnityServiceSingleton } from './unity_service';


class RosService {

    constructor() {



    }

}

let singleton = null;
export const GetRosServiceSingleton = () => {
    if (singleton === null) {
        singleton = new RosService();
    }

    return singleton;
};