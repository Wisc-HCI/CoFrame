import { eventsToStates, statesToSteps } from ".";
import { STATUS, STEP_TYPE } from "../Constants";

export const delayCompiler = ({data, memo}) => {

    // Retrieve agent. For now, assume that this is always the first robotAgentType;
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];

    const events = [
        {
            condition: {
                [robot.id]: { busy: false }
            },
            onTrigger: [
                {
                    stepType: STEP_TYPE.ACTION_START,
                    data: {},
                    effect: { [robot.id] : { busy: true } },
                    source: data.id,
                    delay: 0,
                },
                {
                    stepType: STEP_TYPE.ACTION_END,
                    data: {},
                    effect: { 
                        [robot.id] : { busy: false } },
                    source: data.id,
                    delay: data.properties.duration * 1000,
                }
            ],
            source: data.id
        }
    ]

    const newCompiled = {
        status: STATUS.VALID,
        events,
        steps: statesToSteps(eventsToStates(events))
    }
    return newCompiled
}

// - If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
// - If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
// - If a node is STATUS.VALID and has no children, return the previously calculated value from data.