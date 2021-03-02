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
        super(
            primitives,
            (appendType) ? 'loop.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._condition = null;

        this.condition = condition;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            condition: (this.condition !== null) ? this.condition.toDict() : null
        };
        return msg;
    }

    static fromDict(dct) {
        return new Loop(
            dct.primitives.map(p => NodeParser(p)),
            (dct.condition !== null) ? NodeParser(dct.condition) : null,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'loop' };
    }

    static BlocklyBlock() {
        return { key: 'loop', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Loop")
                this.appendStatementInput("children")
                    .setCheck(null);
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(160);
                this.setTooltip("loop");
                this.setHelpUrl("loop");
            }
        }};
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