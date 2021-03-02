import { Pose, Position, Orientation } from './geometry';
import Blockly from 'blockly';

export class Thing extends Pose {

    /*
    * Class Constants
    */

    static DANGEROUS = 0;
    static SAFE = 1;

    /*
    * Data structure methods
    */

    static typeString() {
        return 'thing.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + Thing.typeString();
    }

    constructor(thingType, safetyLevel, meshId, position=null, orientation=null, weight=0, 
                type='', name='', uuid=null, parent=null, appendType=true) 
    {
        super(
            position,
            orientation,
            (appendType) ? 'thing.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._thingType = null;
        this._safetyLevel = null;
        this._meshId = null;
        this._weight = null;

        this.thingType = thingType;
        this.safetyLevel = safetyLevel;
        this.meshId = meshId;
        this.weigth = weight;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            thing_type: this.thingType,
            safety_level: this.safetyLevel,
            mesh_id: this.meshId,
            weight: this.weight
        };
        return msg;
    }

    static fromDict(dct) {
        return new Thing(
            dct.thing_type,
            dct.safety_level,
            dct.mesh_id,
            Position.fromDict(dct.position),
            Orientation.fromDict(dct.orientation),
            dct.weight,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'thing' };
    }

    static BlocklyBlock() {
        return { key: 'thing', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Thing");
                this.appendDummyInput()
                    .appendField(new Blockly.FieldDropdown([["unnamed","default"]]), "name");
                this.setInputsInline(true);
                this.setOutput(true, null);
                this.setColour(20);
                this.setTooltip("thing");
                this.setHelpUrl("thing");
            }
        }};
    }

    /*
    * Data accessor/modifier methods
    */

    get thingType() {
        return this._thingType;
    }

    set thingType(value) {
        if (this._thingType !== value) {
            if (value === null) {
                throw new Error('Thing type must exist');
            }

            this._thingType = value;
            this.updatedAttribute('thing_type','set');
        }
    }

    get safetyLevel() {
        return this._safetyLevel;
    }

    set safetyLevel(value) {
        if (this._safetyLevel !== value) {
            if (value > Thing.SAFE || value < Thing.DANGEROUS) {
                throw new Error(`Safety level must be within range (${Thing.DANGEROUS},${Thing.SAFE})`)
            }

            this._safetyLevel = value;
            this.updatedAttribute('safety_level','set');
        }
    }

    get meshId() {
        return this._meshId;
    }

    set meshId(value) {
        if (this._meshId !== value) {
            this._meshId = value;
            this.updatedAttribute('mesh_id','set');
        }
    }

    get weight() {
        return this._weight;
    }

    set weight(value) {
        if (this._weight !== value) {
            this._weight = value;
            this.updatedAttribute('weight','set');
        }
    }

    set(dct) {

        if ('thing_type' in dct) {
            this.thingType = dct.thing_type;
        }

        if ('safety_level' in dct) {
            this.safetyLevel = dct.safety_level;
        }

        if ('mesh_id' in dct) {
            this.meshId = dct.mesh_id;
        }

        if ('weight' in dct) {
            this.weight = dct.weight;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('thing_type','update');
        this.updatedAttribute('safety_level','set');
        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('weight','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('thing_type','update');
        this.updatedAttribute('safety_level','set');
        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('weight','update');
    }
}