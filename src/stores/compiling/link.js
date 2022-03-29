import { STATUS } from "../Constants";

export const linkCompiler = ({path,properties}) => {
    const transform = properties.robot;
    console.log('LINK COMPILING',transform)

    return {status:STATUS.VALID}
}