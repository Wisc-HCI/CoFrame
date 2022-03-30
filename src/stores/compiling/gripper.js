import { STATUS } from "../Constants";
import { mapValues } from 'lodash';
import { findLastSatisfiedFromReference } from "../helpers";

export const gripperCompiler = ({data, properties, module, worldModel}) => {
    const idx = findLastSatisfiedFromReference(properties.gripperIndex, v => v >= properties.initialGripState);
    const links = mapValues(properties.gripperFrames,frameSet=>frameSet[idx]);
    return { links, status: STATUS.VALID }
}