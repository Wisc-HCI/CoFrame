import { eventsToStates, statesToSteps } from ".";
import { STATUS, STEP_TYPE, TRIGGER_TYPE } from "../Constants";

export const processCompiler = ({ data, properties, path, context, memo }) => {
    const process = properties.process;
    const machine = properties.machine;
    const processMachine = process.id ? properties.process?.properties?.compiled?.[path]?.machine : {};
    // console.log({process, machine, processMachine});

    // Retrieve agent. For now, assume that this is always the first robotAgentType;
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];

    let newCompiled = {
        shouldBreak: false,
        // Process always required, and if there is a machine, needs to match the machine corresponding to the process
        status: machine.id === processMachine.id && process.id ? STATUS.VALID : STATUS.FAILED,
        otherPropertyUpdates: {},
        steps: [],
        events: []
    };

    // Define some statuses that are relevant
    // const machineStartStatus = status === STATUS.VALID && machine ? {[machine.id]:{running:true}} : {};
    // const processStartStatus = status === STATUS.VALID && process ? {[process.id]:{running:true},...machineStartStatus} : {};

    
    const stepData = {
        machine: machine.id ? machine.id : null,
        process: process.id ? process.id : null
    }

    const machineIdleState = newCompiled.status === STATUS.VALID && machine.id ? { [machine.id]: {busy: false} } : {};
    const processIdleState = newCompiled.status === STATUS.VALID && process.id ? { [process.id]: {busy: false}, ...machineIdleState } : {};
    const machineBusyState = newCompiled.status === STATUS.VALID && machine.id ? { [machine.id]: {busy: true} } : {};
    const processBusyState = newCompiled.status === STATUS.VALID && process.id ? { [process.id]: {busy: true}, ...machineBusyState } : {};
    const robotIdleState = {[robot.id]: {busy: false}};
    const robotBusyState = {[robot.id]: {busy: true}}

    const processSpawnables = process.properties ? process.properties.compiled[path].outputs : [];
    const processDestroyables = process.properties ? process.properties.compiled[path].inputs : [];
    // console.log({process})
    const requiredTime = process.id ? process.properties.compiled[path].processTime : 0;
    // console.log('required time:',requiredTime)

    if (data.type === 'processStartType' && newCompiled.status === STATUS.VALID) {
        newCompiled.events = [
            {
                condition: {...processIdleState,...robotIdleState},
                onTrigger: [
                    {
                        stepType: STEP_TYPE.PROCESS_START,
                        data: {...stepData,effects:processBusyState},
                        effect: processBusyState,
                        source: data.id,
                        delay: 0,
                    },
                    ...processDestroyables.map(destroyable => ({
                        stepType: STEP_TYPE.DESTROY_ITEM,
                        data: {
                            ...stepData,
                            thing: destroyable.properties.compiled[path].thing.id,
                            inputOutput: destroyable.id,
                            position: destroyable.properties.compiled[path].position,
                            rotation: destroyable.properties.compiled[path].rotation,
                            relativeTo: destroyable.properties.compiled[path].relativeObject.id ? destroyable.properties.compiled[path].relativeObject : null
                        },
                        source: data.id,
                        effect: { [destroyable.id]: 'destroyed' },
                        delay: 0
                    })),
                    {
                        stepType: STEP_TYPE.PROCESS_END,
                        data: {...stepData,effects:processIdleState},
                        effect: processIdleState,
                        source: data.id,
                        delay: requiredTime
                    },
                    ...processSpawnables.map(spawnable => ({
                        stepType: STEP_TYPE.SPAWN_ITEM,
                        data: {
                            ...stepData,
                            thing: spawnable.properties.compiled[path].thing.id,
                            inputOutput: spawnable.id,
                            position: spawnable.properties.compiled[path].position,
                            rotation: spawnable.properties.compiled[path].rotation,
                            relativeTo: spawnable.properties.compiled[path].relativeObject.id ? spawnable.properties.compiled[path].relativeObject : null
                        },
                        source: data.id,
                        effect: { [spawnable.id]: 'spawned' },
                        delay: requiredTime
                    })),
                ],
                source: data.id
            }
        ]
    } else if (data.type === 'processWaitType' && newCompiled.status === STATUS.VALID) {
        newCompiled.events = [
            {
                condition: robotIdleState,
                onTrigger: [
                    {
                        stepType: STEP_TYPE.ACTION_START,
                        data: {...stepData,effects:robotBusyState},
                        effect: robotBusyState,
                        source: data.id,
                        delay: 0,
                    }
                ],
                source: data.id
            },
            {
                condition: {...processIdleState,...robotBusyState},
                onTrigger: [
                    {
                        stepType: STEP_TYPE.ACTION_END,
                        data: {...stepData,effects:robotIdleState},
                        effect: robotIdleState,
                        source: data.id,
                        delay: 0,
                    }
                ],
                source: data.id
            }
        ]
    }

    newCompiled.steps = statesToSteps(eventsToStates(newCompiled.events));

    return newCompiled
}