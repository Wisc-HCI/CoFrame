import { OpenGripper } from '.';
import { NodeParser } from '../../utility_functions';
import { MachineInitialize } from '../machine_operations';
import { MoveUnplanned } from '../primitives';
import { Task } from '../task';


export class Initialize extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'initialize.';
    }

    static fullTypeString() {
        return Task.fullTypeString() + Initialize.typeString();
    }

    constructor(homeLocUuid=null, machineUuids=[], type='', name='', uuid=null, parent=null, appendType=true, primitives=null) {

        if (primitives === null) {
            primitives = [];

            primitives = primitives + machineUuids.map(id => MachineInitialize(id));

            primitives = primitives + [
                MoveUnplanned(homeLocUuid,true),
                OpenGripper()
            ];
        }
    }

    static fromDict(dct) {
        return Initialize(
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid,
            primitives= dct.primitives.map(p => NodeParser(p))
        );
    }
}