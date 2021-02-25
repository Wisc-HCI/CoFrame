
class UnityService {

    constructor() {

    }

}

let singleton = null;
export const GetUnityServiceSingleton = () => {
    if (singleton === null) {
        singleton = new UnityService();
    }

    return singleton;
};