import { ERROR, STATUS, STEP_TYPE } from "../Constants";
import { eventsToStates, statesToSteps } from ".";

export const machineCompiler = ({data, properties, memo}) => {
    const machine = properties.machine;
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];
    
    const status = machine.id ? STATUS.VALID : STATUS.FAILED;
    const errorCode = machine.id ? null : ERROR.MISSING_PARAMETER;

    const events = status === STATUS.VALID ? [
        {
            condition: robot ? {
                [robot.id]: { busy: false }
            } : {},
            onTrigger: [
                {
                    stepType: STEP_TYPE.LANDMARK,
                    data: {machine: machine.id,label: 'Machine Initialized'},
                    effect: {[machine.id] : { initialized: true }},
                    source: data.id,
                    delay: 0,
                }
            ],
            source: data.id
        }
    ] : [];

    const newCompiled = {
        status,
        errorCode,
        events,
        steps: statesToSteps(eventsToStates(events))
    }
    return newCompiled
}