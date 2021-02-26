import { Primitive } from './primitive';
import { NodeParser } from '../utilityFunctions';


export class Task extends Primitive {

    /*
    * data structure methods
    */

    static typeString() {
        return 'task.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + Task.typeString();
    }

    constructor(primitives=[], type='', name='', uuid=null, parent=null, appendType=true) {
        this._primitives = [];

        super(
            type= (appendType) ? 'task.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.primitives = primitives;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            primitives: this.primitives.map(p => p.toDict())
        };
        return msg;
    }

    static fromDict(dct) {
        return new Task(
            primitives= dct.primitives.map(p => NodeParser(p)),
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    /*
    * data accessor/modifier methods
    */

    get parent() {
        return this._parent;
    }

    set parent(value) {
        if (this._parent !== value) {
            this.removeFromCache();
            this._parent = value;
            this.addToCache();

            this.updatedAttribute('parent','set');
        }
    }

    get primitives() {
        return this._primitives;
    }

    set primitives(value) {
        if (this._primitives !== value) {
            for (const p in this._primitives) {
                p.removeFromCache();
            }

            this._primitives = value;
            for (const p in this._primitives) {
                p.parent = this;
            }
        }
    }

    addPrimitive(primitive) {
        primitive.parent = this;
        this._primitives.push(primitive);
        this.updatedAttribute('primitives','add')
    }

    insertPrimitive(primitive, idx) {
        primitive.parent = this;
        this._primitives.splice(idx,0,primitive);
        this.updatedAttribute('primitives','add')
    }

    reorderPrimitives(uuid, shift) {
        const idx = this.findPrimitiveIndex(uuid);

        if (idx !== null) {
            let shiftedIdx = idx + shift;
            if (shiftedIdx < 0 || shiftedIdx >= this._primitives.length) {
                throw new Error('Index out of bounds');
            }

            const copy = this._primitives.splice(idx, 1);
            this._primitives.splice(shiftedIdx, 0, copy);
            this.updatedAttribute('primitives','reorder');
        }
    }

    deletePrimitive(uuid) {
        const idx = this.findPrimitiveIndex(uuid);

        if (idx !== null) {
            this._primitives.splice(idx, 1);
            this.updatedAttribute('primitives','delete',uuid);
        }
    }

    getPrimitive(uuid) {
        const idx = this.findPrimitiveIndex(uuid);
        if (idx !== null) {
            return this._primitives[idx];
        } else {
            return null;
        }
    }

    findPrimitiveIndex(uuid) {
        let idx = null;
        for (let i=0; i<this._primitives.length; i++) {
            if (this._primitives[i].uuid === uuid) {
                idx = i;
                break;
            }
        }
        return idx;
    }

    set(dct) {

        if ('primitives' in dct) {
            this.primitives = dct.primitives.map(p => NodeParser(p));
        }

        super.set(dct);
    }

    /*
    * cache methods
    */

    removeFromCache() {
        this.primitives.forEach(p => p.removeFromCache());
        super.removeFromCache();
    }

    addToCache() {
        this.primitives.forEach(p => p.addToCache());
        super.addToCache();
    }

    /*
    * children methods
    */

    deleteChild(uuid) {
        let success = true;

        if (this.findPrimitiveIndex(uuid) !== null) {
            this.deletePrimitive(uuid);
        } else {
            success = false;
        }

        return success;
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.primitives.forEach(p => p.lateConstructUpdate());
        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.primitives.forEach(p => p.deepUpdate());
        super.deepUpdate();
        
        this.updatedAttribute('primitives','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('primitives','update');
    }
}