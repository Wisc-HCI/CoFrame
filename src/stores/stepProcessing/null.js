import { STATUS } from "../Constants";

export const nullSteps = (data, objectTypes, context, solver, module, urdf) => {
    return [[], {}, STATUS.VALID, false]
}