import { STATUS } from "../Constants";

export const linkCompiler = ({data,path,properties}) => {
    const transform = properties.agent.properties.compiled ? properties.agent.properties.compiled[path].links[data.id] : null;
    console.log('LINK COMPILING',transform)

    return {status:STATUS.VALID,otherPropertyUpdates:transform}
}