import { STATUS, STEP_TYPE } from "../Constants";
import { range, mapValues } from 'lodash';
import { findLastSatisfiedFromReference } from "../helpers";

export const gripperMotionCompiler = ({ data, properties, path, memo }) => {
    const rootPath = JSON.stringify(['root']);
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

    let innerAnimations = [

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
        innerAnimations.push({
            stepType: STEP_TYPE.SCENE_UPDATE,
            data: { links: frameData, thing: properties.thing.id, gripperValues },
            source: data.id,
            time
        })
    })

    const initialStep = {
        stepType: STEP_TYPE.ACTION_START,
        data: { agent: 'robot', id: data.id },
        source: data.id,
        time: 0
    }
    const finalStep = {
        stepType: STEP_TYPE.ACTION_END,
        data: { agent: 'robot', id: data.id },
        source: data.id,
        time: duration
    }

    const newCompiled = {
        status,
        shouldBreak: false,
        steps: [
            initialStep,
            ...innerAnimations,
            finalStep
        ]
    }
    return newCompiled
}