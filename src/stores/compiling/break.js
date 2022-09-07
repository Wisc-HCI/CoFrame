import { STATUS, STEP_TYPE } from "../Constants";
import { eventsToStates, statesToSteps } from ".";

export const breakCompiler = ({ data, memo }) => {

    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];

    const events = [
        {
            condition: {
                [robot.id]: { busy: false }
            },
            onTrigger: [
                {
                    stepType: STEP_TYPE.LANDMARK,
                    data: {label: 'Program Break'},
                    effect: {},
                    source: data.id,
                    delay: 0,
                }
            ],
            source: data.id
        }
    ]

    const newCompiled = {
        shouldBreak: true,
        status: STATUS.VALID,
        events,
        steps: statesToSteps(eventsToStates(events))
    }
    return newCompiled
}