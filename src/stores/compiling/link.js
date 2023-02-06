import { STATUS } from "../Constants";

export const linkCompiler = ({path, properties, compiledMemo}) => {
    // console.log('props',{properties,compiledMemo})
    const transform = compiledMemo[properties.agent.id]?.[path]?.links?.[properties.frameKey];
    // console.log('LINK COMPILING',transform)

    return {status:STATUS.VALID,otherPropertyUpdates:transform}
}