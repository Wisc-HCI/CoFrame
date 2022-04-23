import { STATUS, STEP_TYPE } from "../Constants";
import lodash from 'lodash';
import { eventsToStates, statesToSteps } from ".";

export const simpleCompiler = ({ data, properties, objectTypes, context, path, memo, module, urdf, worldModel }) => {

    let newCompiled = {
        shouldBreak: false,
        status: STATUS.VALID,
        otherPropertyUpdates: {},
        events: [],
        steps: []
    };

    console.log(newCompiled.steps)

    properties.children.some(child => {
        // console.log({child,currentSteps:newCompiled.steps.length})
        const childData = child.properties.compiled[path];
        if (childData.shouldBreak) {
            newCompiled.shouldBreak = true;
            return true
        }
        if (childData.status === STATUS.FAILED) {
            newCompiled.status = STATUS.FAILED;
        }
        newCompiled.events = lodash.concat(newCompiled.events,childData.events);
        return false
    })
    const states = eventsToStates(newCompiled.events);
    console.log({states,events:newCompiled.events})
    newCompiled.steps = statesToSteps(eventsToStates(newCompiled.events))

    return newCompiled
}