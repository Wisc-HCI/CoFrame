import { ERROR, STATUS } from "../Constants";
import lodash from 'lodash';
import { eventsToStates, statesToSteps } from ".";

export const simpleCompiler = ({ data, properties, routes, objectTypes, context, path, memo, module, urdf, worldModel }) => {

    let newCompiled = {
        shouldBreak: false,
        status: STATUS.VALID,
        errorCode: null,
        otherPropertyUpdates: {},
        events: [],
        steps: []
    };
    // console.log('data (simple)',{data,properties,routes})
    properties.children.some((child,idx) => {
        // console.log({child,currentSteps:newCompiled.steps.length})
        // console.log(child);
        let childPath = routes.children[idx];
        const childData = child.properties.compiled[childPath];
        if (childData.shouldBreak) {
            newCompiled.shouldBreak = true;
            return true
        }
        if (childData.status === STATUS.FAILED || childData.status === STATUS.WARN && newCompiled.status !== STATUS.FAILED) {
            // Status is failed/warned, so the parent is also.
            newCompiled.status = childData.status;
            newCompiled.errorCode = ERROR.CHILD_FAILED
        };
        newCompiled.events = lodash.concat(newCompiled.events,childData.events);
        return false
    })
    // const states = eventsToStates(newCompiled.events);
    // console.log({states,events:newCompiled.events})
    newCompiled.steps = statesToSteps(eventsToStates(newCompiled.events))

    return newCompiled
}