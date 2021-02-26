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
            type= (appendType) ? 'primitive.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return new Primitive(
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

}