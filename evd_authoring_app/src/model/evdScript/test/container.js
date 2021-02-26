import { Node } from '../node';
import { NodeParser } from '../utilityFunctions';

export class Container extends Node {

    /*
    * Data structure methods
    */

    static typeString(itemType='node.') {
        return `container<${itemType}>`;
    }

    static fullTypeString(itemType='node.') {
        return Node.fullTypeString() + Container.typeString();
    }

    constructor(itemType, values=[], type='', name='', uuid=null, parent=null, appendType=true) {
        this._values = null;
        this._itemType = null;

        super(
            type= (appendType) ? `container<${itemType}>`+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.values = values;
        this.itemType = itemType;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            values: this.values.map(v => v.toDict()),
            item_type: this.itemType
        };
        return msg;
    }

    static fromDict(dct) {
        return new Container(
            itemType= dct.item_type,
            values= dct.values.map(v => NodeParser(v)),
            name= dct.name,
            uuid= dct.uuid,
            type= dct.type,
            appendType= false
        );
    }

    /*
    * Data accessor/modiifer methods
    */

    get values() {
        return this._values;
    }

    set values(value) {
        if (this._values != value) {

            this._values.forEach(element => {
                element.removeFromCache();
            });

            this._values = value;
            this._values.forEach(element => {
                element.parent = this;
            });

            this.updatedAttribute('values','set');
        }
    }

    getValue(uuid) {
        return this.values.filter(v => v.uuid === uuid)[0];
    }

    addValue(value) {
        value.parent = this;
        this._values.push(value);
        this.updatedAttribute('values','add',value.uuid);
    }

    deleteValue(uuid) {
        const idx = searchForValueIndex(uuid);
        if (idx == -1) {
            throw new Error('Value not in container');
        }

        this._values[idx].removeFromCache();
        this._values.splice(idx,1);
        this.updatedAttribute('values','delete',uuid);
    }

    searchForValueIndex(uuid) {
        let idx = -1;
        this._values.forEach((v,i) => {
            if (v.uuid == uuid) {
                idx = i;
            }
        });
        return idx;
    }

    get itemType() {
        return this._itemType;
    }

    set itemType(value) {
        if (this._itemType != value) {
            this._itemType = value;
            this.updatedAttribute('item_type','set');
        }
    }

    set(dct) {

        if ('item_type' in dct) {
            this.itemType = dct.item_type;
        }

        if ('values' in dct) {
            this.values = dct.values.map(v => NodeParser(v));
        }
    }

    /*
    * Cache Methods
    */

    removeFromCache() {
        this.values.forEach(v => v.removeFromCache());

        super.removeFromCache();
    }

    addToCache() {
        this.values.forEach(v => v.addToCache());

        super.addToCache();
    }

    /*
    * Update Methods
    */

    lateConstructUpdate() {
        this.values.forEach(v => v.lateConstructUpdate());

        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.values.forEach(v => v.deepUpdate());

        super.deepUpdate();

        this.updatedAttribute('values','update');
        this.updatedAttribute('item_type','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('values','update');
        this.updatedAttribute('item_type','update');
    }
}