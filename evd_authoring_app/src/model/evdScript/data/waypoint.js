import { Pose, Position, Orientation } from './geometry';
import Blockly from 'blockly';

export class Waypoint extends Pose {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'waypoint.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + Waypoint.typeString();
    }

    constructor(position=null, orientation=null, joints=null, 
                type='', name='', uuid=null, parent=null, appendType=true) 
    {
        super(
            position,
            orientation,
            (appendType) ? 'waypoint.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._joints = null;

        this.joints = joints;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            joints: this.joints
        };
        return msg;
    }

    static fromDict(dct) {
        return new Waypoint(
            Position.fromDict(dct.position),
            Orientation.fromDict(dct.orientation),
            dct.joints,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'waypoint' };
    }

    static BlocklyBlock() {
        return { key: 'waypoint', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Waypoint");
                this.appendDummyInput()
                    .appendField(new Blockly.FieldDropdown([["unnamed","default"]]), "name");
                this.setInputsInline(true);
                this.setPreviousStatement(true, ["trajectory", "waypoint"]);
                this.setNextStatement(true, ["trajectory", "waypoint"]);
                this.setColour(290);
                this.setTooltip("waypoint");
                this.setHelpUrl("waypoint");
            }
        }};
    }

    /*
    * Data accessor/modifier methods
    */

    get joints() {
        return this._joints;
    }

    set joints(value) {
        if (this._joints !== value) {
            this._joints = value;
            this.updatedAttribute('joints','set');
        }
    }

    set(dct) {

        if ('joints' in dct) {
            this.joints = dct.joints;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('joints','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('joints','update');
    }

}