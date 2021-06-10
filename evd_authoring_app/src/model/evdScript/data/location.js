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
        return { 
            name: 'Location',
            category: 'Location',
            block: {
                init: function () {
                    this.jsonInit({
                        type: "example_dropdown",
                        message0: "Location %1",
                        args0: [
                            {
                            type: "field_dropdown",
                            name: "FIELDNAME",
                            options: locations
                            }
                        ],
                        inputsInline: true,
                        colour: 260,
                        output: 'null',
                        tooltip: "location"
                    });
                },
            },
            generator: (block) => {
                var value_length = block.getFieldValue('name');
                var code = `console.log('Length is: ${value_length}')`;
                return [code, Blockly.JavaScript.ORDER_ATOMIC];
            }
        };
    }
}