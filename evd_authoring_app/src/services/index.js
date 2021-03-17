import { GetRosServiceSingleton } from './rosService';
import { GetEvDScriptServiceSingleton } from './evdScriptService';
import { GetPendingServiceSingleton } from './pendingService';
import { GetUnityServiceSingleton } from './unityService';

const EvDService = GetEvDScriptServiceSingleton();
const RosService = GetRosServiceSingleton();
const PendingService = GetPendingServiceSingleton();
const UnityService = GetUnityServiceSingleton();

export {
    EvDService,
    RosService,
    PendingService,
    UnityService,
    GetEvDScriptServiceSingleton,
    GetRosServiceSingleton,
    GetPendingServiceSingleton,
    GetUnityServiceSingleton
};