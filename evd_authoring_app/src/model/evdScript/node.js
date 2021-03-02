
import { v4 as uuidv4 } from 'uuid';

export class Node {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'node.';
    }

    static fullTypeString() {
        return Node.typeString();
    }

    constructor(type='', name='', uuid=null, parent=null, appendType=true) {
        this._cacheFunction = null;
        import('./cache').then((module) => {
            this._cacheFunction = module.getEvdCacheObject;
        });
        
        this._parent = null;
        this._type = null;
        this._name = null;

        if (uuid === null) {
            this._uuid = Node._generateUuid(type);
        } else {
            this._uuid = uuid;
        }

        this.parent = parent;
        this.type = (appendType) ? 'node.' + type : type;
        this.name = name;
    }

    static fromDict(dct) {
        return new Node(
            ('type' in dct) ? dct.type : '',
            ('name' in dct) ? dct.name : '',
            ('uuid' in dct) ? dct.uuid : null,
            null,
            !('type' in dct)
        );
    }

    toDict() {
        return {
            type: this.type,
            name: this.name,
            uuid: this.uuid
        }
    }

    toBlockly() {
        // Implement this for nodes that can (and should) be Blockly blocks
        return null;
    }

    static BlocklyBlock() {
        return null;
    }

    static BlocklyToolbox() {
        return null;
    }

    /*
    * Data accessors/modifer methods
    */

    get context() {
        if (this._parent !== null) {
            return this._parent.context;
        } else {
            return null;
        }
    }

    get uuid() {
        return this._uuid;
    }

    get type() {
        return this._type;
    }

    set type(value) {
        if (this._type !== value) {
            this._type = value;
            this.updatedAttribute('type','set');
        }
    }

    get name() {
        return this._name;
    }

    set name(value) {
        if (this._name !== value) {
            this._name = value;
            this.updatedAttribute('name','set');
        }
    }

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

    set(dct) {
        // Note: cannot set uuid

        if ('name' in dct) {
            this.name = dct.name;
        }

        if ('type' in dct) {
            this.type = dct.type;
        }
    }

    /*
    * Cache Methods
    */

    removeFromCache() {
        if (this._cacheFunction !== null) {
            this._cacheFunction().remove(this.uuid);
        }
    }

    addToCache() {
        if (this._cacheFunction !== null) {
            this._cacheFunction().add(this.uuid);
        }
    }

    refreshCacheEntry() {
        this.removeFromCache();
        this.addToCache();
    }

    /*
    * Children methods (optional)
    */

    deleteChild(uuid) {
        // Write this for each node that has a set of children
        return true;
    }

    childChangedEvent(attributeTrace) {
        if (this.parent !== null) {
            attributeTrace.push(this._childChangedEventMsg(null,'callback'));
            this._parent.childChangedEvent(attributeTrace);
        }
    }

    /*
    * Utility methods
    */

    static _generate_uuid(type) {
        return `${type}-js-${uuidv4()}`;
    }

    _childChangedEventMsg(attribute, verb, childUuid=null) {
        return {
            type: this.type,
            uuid: this.uuid,
            attribute,
            verb,
            childUuid
        }
    }

    /*
    * Update Methods
    */

    lateConstructUpdate() {
        // Implement if your class needs to update something after entire program is constructed
    }

    updatedAttribute(attribute, verb, childUuid=null) {
        if (this.parent != null) {
            this.parent.childChangedEvent([
                this._childChangedEventMsg(attribute,verb,childUuid)
            ]);
        }
    }

    deepUpdate() {
        this.updatedAttribute('name','update');
        this.updatedAttribute('type','update');
        this.updatedAttribute('uuid','update');
    }

    shallowUpdate() {
        this.updatedAttribute('name','update');
        this.updatedAttribute('type','update');
        this.updatedAttribute('uuid','update');
    }
}