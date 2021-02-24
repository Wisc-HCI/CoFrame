import { Primitive } from '../primitive';


export class Branch extends Primitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'branch.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + Branch.typeString();
    }

    constructor(type='', name='', uuid=null, parent=null, appendType=true) {
        // TODO
    }

    toDict() {
        return {}; //TODO
    }

    static fromDict(dct) {
        return Branch(); // TODO
    }

}