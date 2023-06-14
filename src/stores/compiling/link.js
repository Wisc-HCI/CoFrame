import { quaternionToEuler } from "../../helpers/geometry";
import { STATUS } from "../Constants";

export const linkCompiler = ({path, properties, compiledMemo}) => {
    // console.log('props',{properties,compiledMemo})
    const transform = compiledMemo[properties.agent.id]?.[path]?.links?.[properties.frameKey];
    if (transform) {
        return {status:STATUS.VALID,otherPropertyUpdates: {...transform, rotation: quaternionToEuler(transform.rotation)}}
    }
    return {status:STATUS.VALID,otherPropertyUpdates: transform}
}