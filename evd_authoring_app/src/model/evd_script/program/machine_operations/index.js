import { MachineBlockingProcess } from './machine_blocking_process';
import { MachineInitialize } from './machine_initialize';
import { MachinePrimitive } from './machine_primitive';
import { MachineStart } from './machine_start';
import { MachineStop } from './machine_stop';
import { MachineWait } from './machine_wait';


const MachineOperationsNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'machine-primitive':
            node = MachinePrimitive.fromDict(dct);
            break;
        case 'machine-start':
            node = MachineStart.fromDict(dct);
            break;
        case 'machine-wait':
            node = MachineWait.fromDict(dct);
            break;
        case 'machine-stop':
            node = MachineStop.fromDict(dct);
            break;
        case 'machine-initialize':
            node = MachineInitialize.fromDict(dct);
            break;
        case 'machine-blocking-process':
            node = MachineBlockingProcess.fromDict(dct);
            break;
    }

    return node;
};

export {
    MachineBlockingProcess,
    MachineInitialize,
    MachinePrimitive,
    MachineStart,
    MachineStart,
    MachineWait,
    MachineOperationsNodeParser
};