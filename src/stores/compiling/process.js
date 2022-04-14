import { STATUS, STEP_TYPE } from "../Constants";

export const processCompiler = ({ data, properties, path, context, memo }) => {
    const process = properties.process;
    const machine = properties.machine;
    const processMachine = process.id ? properties.process?.properties?.compiled?.[path]?.machine : {};
    console.log({process, machine, processMachine});

    let newCompiled = {
        shouldBreak: false,
        // Process always required, and if there is a machine, needs to match the machine corresponding to the process
        status: machine.id === processMachine.id && process.id ? STATUS.VALID : STATUS.FAILED,
        otherPropertyUpdates: {},
        steps: []
    };

    // Define some statuses that are relevant
    // const machineStartStatus = status === STATUS.VALID && machine ? {[machine.id]:{running:true}} : {};
    // const processStartStatus = status === STATUS.VALID && process ? {[process.id]:{running:true},...machineStartStatus} : {};

    const machineStartStatus = newCompiled.status === STATUS.VALID && machine ? { [machine.id]: 'started' } : {};
    const processStartStatus = newCompiled.status === STATUS.VALID && process ? { [process.id]: 'started', ...machineStartStatus } : {};
    const machineFinishStatus = newCompiled.status === STATUS.VALID && machine ? { [machine.id]: 'finished' } : {};
    const processFinishStatus = newCompiled.status === STATUS.VALID && process ? { [process.id]: 'finished', ...machineFinishStatus } : {};
    const machineStopStatus = newCompiled.status === STATUS.VALID && machine ? { [machine.id]: 'stopped' } : {};
    const processStopStatus = newCompiled.status === STATUS.VALID && process ? { [process.id]: 'stopped', ...machineStopStatus } : {};

    const startStepData = {
        machine: machine ? machine.id : null,
        process: process ? process.id : null,
        statuses: processStartStatus,
        id: data.id
    }

    const finishStepData = {
        machine: machine ? machine.id : null,
        process: process ? process.id : null,
        statuses: processFinishStatus,
        id: data.id
    }

    const stopStepData = {
        machine: machine ? machine.id : null,
        process: process ? process.id : null,
        statuses: processStopStatus,
        id: data.id
    }

    const processSpawnables = process.properties ? process.properties.compiled[path].outputs : [];
    const processDestroyables = process.properties ? process.properties.compiled[path].inputs : [];
    console.log({processSpawnables,processDestroyables})

    if (data.type === 'processStartType') {
        newCompiled.steps = [
            {
                stepType: STEP_TYPE.PROCESS_START,
                data: startStepData,
                source: data.id,
                time: 0
            },
            ...processDestroyables.map(destroyable=>{

                return ({
                    stepType: STEP_TYPE.DESTROY_ITEM,
                    data: {
                        thing: destroyable.properties.compiled[path].thing.id,
                        inputOutput: destroyable.id,
                        position: destroyable.properties.compiled[path].position,
                        rotation: destroyable.properties.compiled[path].rotation,
                        relativeTo: destroyable.properties.compiled[path].relativeObject.id ? destroyable.properties.compiled[path].relativeObject : null
                    },
                    source: data.id,
                    time: 0
                })
            }),
            {
                stepType: STEP_TYPE.LANDMARK,
                data: processFinishStatus,
                source: data.id,
                time: process.id ? process.properties.processTime : 0
            }
        ]
    } else if (data.type === 'processWaitType') {
        newCompiled.steps = [
            {
                stepType: STEP_TYPE.ACTION_START,
                data: { agent: 'robot', id: data.id },
                source: data.id,
                time: 0
            },
            {
                stepType: STEP_TYPE.ACTION_END,
                data: { agent: 'robot', id: data.id },
                source: data.id,
                time: processFinishStatus
            },
        ];
    } else if (data.type === 'processStopType') {
        newCompiled.steps = [
            {
                stepType: STEP_TYPE.PROCESS_END,
                data: stopStepData,
                source: data.id,
                time: 0
            },
            ...processSpawnables.map(spawnable=>{

                return ({
                    stepType: STEP_TYPE.SPAWN_ITEM,
                    data: {
                        thing: spawnable.properties.compiled[path].thing.id,
                        inputOutput: spawnable.id,
                        position: spawnable.properties.compiled[path].position,
                        rotation: spawnable.properties.compiled[path].rotation,
                        relativeTo: spawnable.properties.compiled[path].relativeObject.id ? spawnable.properties.compiled[path].relativeObject : null
                    },
                    source: data.id,
                    time: 0
                })
            }),
        ]
    }

    return newCompiled
}