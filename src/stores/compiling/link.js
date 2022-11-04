import { STATUS } from "../Constants";

export const linkCompiler = ({data, path, properties, compiledMemo}) => {
    // console.log(properties)
    const transform = compiledMemo[properties.agent]?.[path]?.links?.[properties.frameKey];
    // console.log('LINK COMPILING',transform)

    return {status:STATUS.VALID,otherPropertyUpdates:transform}
}