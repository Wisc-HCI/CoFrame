import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, compilers } from './index';
import { merge } from 'lodash';
import { equals } from "./index";

export const simpleCompiler = ({ data, properties, objectTypes, context, path, memo, solver, module, urdf, worldModel }) => {

    let newCompiled = {steps: []};
    let otherPropertyUpdates = null;
    let status = STATUS.VALID;
    let updated = false;
    let shouldBreak = false;
    let lastUpdateTime = 0;

    console.log('HANDLING SIMPLE COMPILER', status)

    properties.children.forEach(child => {
        console.log(child)
    })

    // let lastUpdateTime = 0;
    // pathInnerSteps.forEach(v => {
    //     let stepTime = 0;
    //     if (typeof v.time === "object") {
    //         // encode the timing based on the landmark specified
    //         const tempSteps = newCompiled[path].slice().reverse();
    //         tempSteps.some(s => {
    //             if (s.stepType === STEP_TYPE.LANDMARK && equals(s.data, v.time)) {
    //                 stepTime = s.time
    //                 return false
    //             } else if (s.time < lastUpdateTime) {
    //                 return true
    //             } else {
    //                 return false
    //             }
    //         })
    //     } else {
    //         stepTime = v.time
    //     }
    //     if (v.stepType !== STEP_TYPE.LANDMARK) {
    //         // Ignore landmarks
    //         lastUpdateTime = stepTime
    //     }
    //     newCompiled[path].push({ ...v, time: stepTime + lastTime })
    // })
    // if (pathInnerSteps.length > 0) {
    //     lastTime = lastTime + lastUpdateTime
    // }


    return { newCompiled, otherPropertyUpdates, status, updated, shouldBreak }
}