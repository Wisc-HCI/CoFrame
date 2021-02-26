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
            type= (appendType) ? 'breakpoint.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
         );
     }

    static fromDict(dct) {
        return new Breakpoint(
            type= dct.type,
            appendType=false,
            name= dct.name,
            uuid= dct.uuid
        );
    }
}