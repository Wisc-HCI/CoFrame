
export class EvDScriptService {

    constructor() {

    }

}

let singleton = null;
export const GetEvDScriptServiceSingleton = () => {
    if (singleton === null) {
        singleton = new EvDScriptService();
    }

    return singleton;
};