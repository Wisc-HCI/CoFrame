import { STATUS, ERROR, STEP_TYPE, ROOT_PATH } from "../Constants";
import { range, mapValues } from 'lodash';
import { findLastSatisfiedFromReference } from "../helpers";
import { eventsToStates, statesToSteps } from ".";

export const gripperMotionCompiler = ({ data, properties, memo }) => {
    
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];
    let status = STATUS.VALID;
    const delta = properties.positionEnd - properties.positionStart;
    if (properties.speed === 0) {
        return {
            status: STATUS.FAILED,
            errorCode: ERROR.DOES_NOTHING,
            shouldBreak: false,
            steps: []
        }
    }

    const thing = properties.thing;
    const duration = 1000 * Math.abs(delta) / properties.speed;
    const changePerTime = delta / duration;
    const closing = properties.positionEnd < properties.positionStart;

    // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe, 
    // but we pre-process them beforehand so it is fine. We also always assume root execution 
    // (which is fine for robots/humans/grippers).

    const grippers = Object.values(memo).filter(v => v.type === 'gripperType');

    let innerSteps = [];

    range(0, duration, 100).map(time => {
        let frameData = {};
        let gripperValues = {};
        grippers.forEach(gripper => {
            console.log(gripper)
            const gripperValue = time * changePerTime + properties.positionStart;
            const idx = findLastSatisfiedFromReference(gripper.properties.compiled[ROOT_PATH].gripperIndex, v => v >= gripperValue);
            const links = mapValues(gripper.properties.compiled[ROOT_PATH].gripperFrames, frameSet => frameSet[idx]);
            frameData = { ...frameData, ...links };
            gripperValues[gripper.id] = gripperValue;
        })
        innerSteps.push({
            stepType: STEP_TYPE.SCENE_UPDATE,
            data: { links: frameData, thing, gripperValues, closing },
            effect: {},
            source: data.id,
            delay:time
        })
    })

    const initialStep = {
        stepType: STEP_TYPE.ACTION_START,
        effect:{[robot.id]: { busy: true }},
        data: { agent: 'robot', id: data.id, closing, thing },
        source: data.id,
        delay: 0
    }
    const finalStep = {
        stepType: STEP_TYPE.ACTION_END,
        effect:{[robot.id]: { busy: false }},
        data: { agent: 'robot', id: data.id, closing, thing },
        source: data.id,
        delay: duration
    }

    const events = [
        {
            condition: {
                [robot.id]: { busy: false }
            },
            onTrigger: [
                initialStep,
                ...innerSteps,
                finalStep
            ],
            source: data.id
        }
    ]

    const newCompiled = {
        status,
        shouldBreak: false,
        events,
        steps: statesToSteps(eventsToStates(events))
    }
    return newCompiled
}