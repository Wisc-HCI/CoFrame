import { Task } from '../task';
import { NodeParser } from '../../utilityFunctions';

export class Loop extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'loop.';
    }

    static fullTypeString() {
        return Task.fullTypeString() + Loop.typeString();
    }

    constructor(primitives=[], condition=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._condition = null;

        super(
            type= (appendType) ? 'loop.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType,
            primitives= primitives
        );

        this.condition = condition;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            condition: (this.condition !== null) ? this.condition.toDict() : null
        };
    }

    static fromDict(dct) {
        return new Loop(
            primitives= dct.primitives.map(p => NodeParser(p)),
            condition= (dct.condition !== null) ? NodeParser(dct.condition) : null,
            name= dct.name,
            uuid= dct.uuid,
            type= dct.type,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get condition() {
        return this._condition;
    }

    set condition(value) {
        if (this._condition !== value) {
            if (this._condition !== null) {
                this._condition.removeFromCache();
            }
            this._condition = value;
            if (this._condition !== null) {
                this._condition.parent = this;
            }
            this.updatedAttribute('condition','set');
        }
    }

    set(dct) {

        if ('condition' in dct) {
            this.condition = (dct.condition !== null) ? NodeParser(dct.condition) : null;
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {
        if (this.condition !== null) {
            this.condition.removeFromCache();
        }

        super.removeFromCache();
    }

    addToCache() {
        if (this.condition !== null) {
            this.condition.addToCache();
        }

        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        if (this.condition !== null) {
            this.condition.lateConstructUpdate();
        }
        super.lateConstructUpdate();
    }

    deepUpdate() {
        if (this.condition !== null) {
            this.condition.deepUpdate();
        }
        super.deepUpdate();

        this.updatedAttribute('condition','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('condition','update');
    }
}