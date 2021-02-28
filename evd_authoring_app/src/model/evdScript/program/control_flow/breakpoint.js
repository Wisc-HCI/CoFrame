import { Primitive } from '../primitive';

export class Breakpoint extends Primitive {

    /*
     * Data structure methods
     */

     static typeString() {
         return 'breakpoint.';
     }

     static fullTypeString() {
         return Primitive.fullTypeString() + Breakpoint.typeString();
     }

     constructor(type='', name='', uuid=null, parent=null, appendType=true) {
         super(
            (appendType) ? 'breakpoint.'+type : type,
            name,
            uuid,
            parent,
            appendType
         );
     }

    static fromDict(dct) {
        return new Breakpoint(
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }
}