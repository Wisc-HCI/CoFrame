import { eventsToStates, statesToSteps } from ".";
import { ERROR, STATUS, STEP_TYPE } from "../Constants";

export const processCompiler = ({ data, properties, path, context, memo, compiledMemo }) => {
    const process = properties.process;
    const gizmo = properties.gizmo;
    // console.log('process',process)
    const processGizmo = process.id && compiledMemo[process.id]?.[path]?.gizmo ? compiledMemo[process.id][path].gizmo : {};
    // console.log({process, gizmo, processGizmo});
    let errorCode = null;

    if (!process.id) {
        errorCode = ERROR.MISSING_PARAMETER
    } else if (gizmo.id !== processGizmo.id) {
        errorCode = ERROR.MISMATCHED_GIZMO
    }

    // Retrieve agent. For now, assume that this is always the first robotAgentType;
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];

    let newCompiled = {
        shouldBreak: false,
        // Process always required, and if there is a gizmo, needs to match the gizmo corresponding to the process
        status: gizmo.id === processGizmo.id && process.id ? STATUS.VALID : STATUS.FAILED,
        errorCode,
        otherPropertyUpdates: {},
        steps: [],
        events: []
    };

    // Define some statuses that are relevant
    // const gizmoStartStatus = status === STATUS.VALID && gizmo ? {[gizmo.id]:{running:true}} : {};
    // const processStartStatus = status === STATUS.VALID && process ? {[process.id]:{running:true},...gizmoStartStatus} : {};

    
    const stepData = {
        gizmo: gizmo.id ? gizmo.id : null,
        process: process.id ? process.id : null
    }

    const gizmoIdleState = newCompiled.status === STATUS.VALID && gizmo.id ? { [gizmo.id]: {busy: false} } : {};
    const processIdleState = newCompiled.status === STATUS.VALID && process.id ? { [process.id]: {busy: false}, ...gizmoIdleState } : {};
    const gizmoBusyState = newCompiled.status === STATUS.VALID && gizmo.id ? { [gizmo.id]: {busy: true} } : {};
    const processBusyState = newCompiled.status === STATUS.VALID && process.id ? { [process.id]: {busy: true}, ...gizmoBusyState } : {};
    const robotIdleState = robot ? {[robot.id]: {busy: false}} : {};
    const robotBusyState = robot ? {[robot.id]: {busy: true}} : {};

    const processSpawnables = process.properties ? compiledMemo[process.id][path].outputs : [];
    const processDestroyables = process.properties ? compiledMemo[process.id][path].inputs : [];
    // console.log({process})
    const requiredTime = process.id ? compiledMemo[process.id][path].processTime : 0;
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
                            thing: compiledMemo[destroyable.id][path].thing.id,
                            inputOutput: destroyable.id,
                            position: compiledMemo[destroyable.id][path].position,
                            rotation: compiledMemo[destroyable.id][path].rotation,
                            relativeTo: compiledMemo[destroyable.id][path].relativeTo ? compiledMemo[destroyable.id][path].relativeTo : null
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
                            thing: compiledMemo[spawnable.id][path].thing.id,
                            inputOutput: spawnable.id,
                            position: compiledMemo[spawnable.id][path].position,
                            rotation: compiledMemo[spawnable.id][path].rotation,
                            relativeTo: compiledMemo[spawnable.id][path].relativeTo ? compiledMemo[spawnable.id][path].relativeTo : null
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