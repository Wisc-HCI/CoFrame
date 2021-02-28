import { Node } from '../node';


export class Primitive extends Node {

    /*
    * data structure methods
    */

    static typeString() {
        return 'primitive.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Primitive.typeString();
    }

    constructor(type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            (appendType) ? 'primitive.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new Primitive(
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

}