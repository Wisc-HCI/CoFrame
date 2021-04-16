import { Skill } from '../skill';
import { Gripper } from '../primitives';
import { NodeParser } from '../../utilityFunctions';
import Blockly from 'blockly';


export class CloseGripper extends Skill {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'close-gripper';
    }

    static fullTypeString() {
        return Skill.fullTypeString() + CloseGripper.typeString();
    }

    constructor(thingUuid=null, position=100, effort=100, speed=100, 
                type='', name='', uuid=null, parent=null, appendType=true, 
                primitives=null) 
    {

        if (primitives !== null) {
            primitives = [
                Gripper(
                    thingUuid,
                    position,
                    effort,
                    speed
                )
            ];
        }

        super(
            primitives,
            (appendType) ? 'close-gripper.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new CloseGripper(
            null,
            100,
            100,
            100,
            dct.type,
            dct.name,
            dct.uuid,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }

    static BlocklyToolbox() {
        return { type: 'close_gripper' };
    }

    static BlocklyBlock() {
        return { key: 'close_gripper', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Close Gripper");
                this.appendValueInput("thing")
                    .setCheck("thing")
                    .appendField("Thing");
                this.appendDummyInput()
                    .appendField("Position")
                    .appendField(new Blockly.FieldNumber(100, 100, 100), "position")
                    .appendField("%");
                this.appendDummyInput()
                    .appendField("Effort")
                    .appendField(new Blockly.FieldNumber(100, 0, 100), "effort")
                    .appendField("%");
                this.appendDummyInput()
                    .appendField("Speed")
                    .appendField(new Blockly.FieldNumber(100, 0, 100), "speed")
                    .appendField("%");
                this.setInputsInline(false);
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(210);
                this.setTooltip("close-gripper");
                this.setHelpUrl("close-gripper");
            }
        }};
    }
}