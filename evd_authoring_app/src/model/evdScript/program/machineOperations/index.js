import { MachineBlockingProcess } from './machineBlockingProcess';
import { MachineInitialize } from './machineInitialize';
import { MachinePrimitive } from './machinePrimitive';
import { MachineStart } from './machineStart';
import { MachineStop } from './machineStop';
import { MachineWait } from './machineWait';


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
    MachineStop,
    MachineWait,
    MachineOperationsNodeParser
};