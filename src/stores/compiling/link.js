import { STATUS } from "../Constants";

export const linkCompiler = ({data,path,properties}) => {
    // console.log(properties)
    const transform = properties.agent?.properties?.compiled?.[path]?.links?.[properties.frameKey];
    // console.log('LINK COMPILING',transform)

    return {status:STATUS.VALID,otherPropertyUpdates:transform}
}