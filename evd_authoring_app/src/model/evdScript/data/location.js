import { Waypoint } from './waypoint';
import { Position, Orientation } from './geometry';
import Blockly from 'blockly';

export class Location extends Waypoint {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'location.';
    }

    static fullTypeString() {
        return Waypoint.fullTypeString() + Location.typeString();
    }

    constructor(position=null, orientation=null, joints=null, type='', name='', 
                uuid=null, parent=null, appendType=true) 
    {
        super(
            position,
            orientation,
            joints,
            (appendType) ? 'location.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new Location(
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
        return { type: 'location' };
    }

    static BlocklyBlock(locations) {
        // TODO need to support type checking here
        // iterate through locations creating new fielddropdown entries
        // remove the unnamed option
        // look up syntax for blockly adding field options
        return { key: 'location', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Location");
                this.appendDummyInput()
                    .appendField(new Blockly.FieldDropdown([["unnamed","default"]]), "name");
                this.setInputsInline(true);
                this.setOutput(true, null);
                this.setColour(260);
                this.setTooltip("location");
                this.setHelpUrl("location");
            }
        }};
    }
}