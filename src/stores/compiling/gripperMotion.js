import { STATUS, STEP_TYPE } from "../Constants";
import { range, mapValues } from 'lodash';
import { findLastSatisfiedFromReference } from "../helpers";
import { eventsToStates, statesToSteps } from ".";

export const gripperMotionCompiler = ({ data, properties, path, memo }) => {
    const rootPath = JSON.stringify(['root']);
    const robot = Object.values(memo).filter(v => v.type === 'robotAgentType')[0];
    let status = STATUS.VALID;
    const delta = properties.positionEnd - properties.positionStart;
    if (properties.speed === 0) {
        status = STATUS.FAILED;
        return {
            status,
            shouldBreak: false,
            steps: []
        }
    }
    const duration = 1000 * Math.abs(delta) / properties.speed;
    const changePerTime = delta / duration;

    // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe, 
    // but we pre-process them beforehand so it is fine. We also always assume root execution 
    // (which is fine for robots/humans/grippers).

    const grippers = Object.values(memo).filter(v => v.type === 'gripperType');

    let innerSteps = [

    ]

    range(0, duration, 100).map(time => {
        let frameData = {};
        let gripperValues = {};
        grippers.forEach(gripper => {
            console.log(gripper)
            const gripperValue = time * changePerTime + properties.positionStart;
            const idx = findLastSatisfiedFromReference(gripper.properties.compiled[rootPath].gripperIndex, v => v >= gripperValue);
            const links = mapValues(gripper.properties.compiled[rootPath].gripperFrames, frameSet => frameSet[idx]);
            frameData = { ...frameData, ...links };
            gripperValues[gripper.id] = gripperValue;
        })
        innerSteps.push({
            stepType: STEP_TYPE.SCENE_UPDATE,
            data: { links: frameData, thing: properties.thing.id, gripperValues },
            effect: {},
            source: data.id,
            delay:time
        })
    })

    const initialStep = {
        stepType: STEP_TYPE.ACTION_START,
        effect:{[robot.id]: { busy: true }},
        data: { agent: 'robot', id: data.id },
        source: data.id,
        delay: 0
    }
    const finalStep = {
        stepType: STEP_TYPE.ACTION_END,
        effect:{[robot.id]: { busy: false }},
        data: { agent: 'robot', id: data.id },
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