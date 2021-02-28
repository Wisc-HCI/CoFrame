import { Primitive } from '../primitive';

export class Delay extends Primitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'delay.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + Delay.typeString();
    }

    constructor(duration=0, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            (appendType) ? 'delay.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._duration = null;

        this.duration = duration;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            duration: this.duration
        };
        return msg;
    }

    static fromDict(dct) {
        return new Delay(
            dct.duration,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    /*
     * Data accessor/modifer methods 
     */

     get duration() {
         return this._duration;
     }

     set duration(value) {
         if (this._duration !== value) {
             this._duration = value;
         }

         super.updatedAttribute('duration','set');
     }

     set(dct) {
         if ('duration' in dct) {
             this.duration = dct.duration;
         }

         super.set(dct);
     }

     /*
     * Update methods
     */

     deepUpdate() {
         super.deepUpdate();

         this.updatedAttribute('duration','update');
     }

     shallowUpdate() {
         super.shallowUpdate();

         this.updatedAttribute('duartion','update');
     }
}