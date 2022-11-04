import { ERROR, STATUS } from "../Constants";
import lodash from 'lodash';
import { eventsToStates, statesToSteps } from ".";

export const goalCompiler = ({ data, properties, routes, objectTypes, context, path, memo, compiledMemo, module, urdf, worldModel }) => {
    // console.log('simple',data.id)
    let newCompiled = {
        shouldBreak: false,
        status: STATUS.VALID,
        errorCode: null,
        otherPropertyUpdates: {},
        events: [],
        steps: []
    };

    properties.example.some((child,idx) => {
        let childPath = routes.example[idx];
        const childData = compiledMemo[child.id][childPath];
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