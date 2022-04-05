import { STATUS, STEP_TYPE } from "../Constants";
import { equals } from "./index";

export const simpleCompiler = ({ data, properties, objectTypes, context, path, memo, solver, module, urdf, worldModel }) => {

    let newCompiled = {
        shouldBreak: false,
        status: STATUS.VALID,
        otherPropertyUpdates: {},
        steps: []
    };
    let lastUpdateTime = 0;

    properties.children.some(child => {
        const childData = child.properties.compiled[path];
        if (childData.shouldBreak) {
            newCompiled.shouldBreak = true;
            return true
        }
        if (childData.status === STATUS.FAILED) {
            newCompiled.status = STATUS.FAILED;
        }
        childData.steps.forEach(innerStep=>{
            let stepTime = 0;
            if (typeof innerStep.time === 'object') {
                newCompiled.slice.reverse().some(previouslyProcessedStep=>{
                    if (previouslyProcessedStep.stepType === STEP_TYPE.LANDMARK && equals(previouslyProcessedStep.data,innerStep.time)) {
                        stepTime = previouslyProcessedStep.time;
                        return false
                    } else if (previouslyProcessedStep.time < lastUpdateTime) {
                        return true
                    } else {
                        return false
                    }
                });

            } else {
                stepTime = innerStep.time
            }
            if (innerStep.stepType !== STEP_TYPE.LANDMARK) {
                // Ignore landmarks
                lastUpdateTime += stepTime
            }
            newCompiled.steps.push({...innerStep,time: lastUpdateTime})
        })
        return false
    })


    return newCompiled
}