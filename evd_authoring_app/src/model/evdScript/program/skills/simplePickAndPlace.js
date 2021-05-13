import { MoveTrajectory } from '../primitives';
import { CloseGripper } from './closeGripper';
import { OpenGripper } from './openGripper';
import { Skill } from '../skill';
import { NodeParser } from '../../utilityFunctions';


export class SimplePickAndPlace extends Skill {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'simple-pick-and-place';
    }

    static fullTypeString() {
        return Skill.fullTypeString() + SimplePickAndPlace.typeString();
    }

    constructor(startLocUuid=null, pickLocUuid=null, placeLocUuid=null, thingUuid=null, 
                type='', name='', uuid=null, parent=null, appendType=true, primitives=null) 
    {

        if (primitives === null) {
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid),
                CloseGripper(thingUuid),
                MoveTrajectory(pickLocUuid,placeLocUuid),
                OpenGripper(thingUuid)
            ];
        }

        super(
            primitives,
            (appendType) ? 'simple-pick-and-place.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new SimplePickAndPlace(
            null,
            null,
            null,
            null,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }

    static BlocklyToolbox() {
        return { type: 'simple_pick_and_place' };
    }

    static BlocklyBlock() {
        return { key: 'simple_pick_and_place', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Simple Pick and Place");
                this.appendValueInput("thing")
                    .setCheck("thing")
                    .appendField("Thing");
                this.appendValueInput("start-location")
                    .setCheck("location")
                    .appendField("Start Location");
                this.appendValueInput("pick-location")
                    .setCheck("location")
                    .appendField("Pick Location");
                this.appendValueInput("place-location")
                    .setCheck("location")
                    .appendField("Place Location");
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(210);
                this.setTooltip("simple-pick-and-place");
                this.setHelpUrl("simple-pick-and-place");
            }
        }};
    }
}