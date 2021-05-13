import { Skill } from '../skill';
import { Gripper } from '../primitives';
import { NodeParser } from '../../utilityFunctions';
import Blockly from 'blockly';


export class OpenGripper extends Skill {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'open-gripper';
    }

    static fullTypeString() {
        return Skill.fullTypeString() + OpenGripper.typeString();
    }

    constructor(thingUuid=null, position=0, effort=100, speed=100, type='', name='', 
                uuid=null, parent=null, appendType=true, primitives=null) 
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
            (appendType) ? 'open-gripper.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new OpenGripper(
            null,
            0,
            100,
            100,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }

    static BlocklyToolbox() {
        return { type: 'open_gripper' };
    }

    static BlocklyBlock() {
        return { key: 'open_gripper', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Open Gripper");
                this.appendValueInput("thing")
                    .setCheck("thing")
                    .appendField("Thing");
                this.appendDummyInput()
                    .appendField("Position")
                    .appendField(new Blockly.FieldNumber(0, 0, 0), "position")
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
                this.setTooltip("open-gripper");
                this.setHelpUrl("open-gripper");
            }
        }};
    }
}