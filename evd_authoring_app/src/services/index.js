import { GetRosServiceSingleton } from './rosService';
import { GetEvDScriptServiceSingleton } from './evdScriptService';
import { GetPendingServiceSingleton } from './pendingService';
import { GetUnityServiceSingleton } from './unityService';
import { GetApplicationServiceSingleton } from './applicationService';

const EvDService = GetEvDScriptServiceSingleton();
const RosService = GetRosServiceSingleton();
const PendingService = GetPendingServiceSingleton();
const UnityService = GetUnityServiceSingleton();
const ApplicationService = GetApplicationServiceSingleton();

export {
    EvDService,
    RosService,
    PendingService,
    UnityService,
    ApplicationService,
    GetEvDScriptServiceSingleton,
    GetRosServiceSingleton,
    GetPendingServiceSingleton,
    GetUnityServiceSingleton,
    GetApplicationServiceSingleton
};