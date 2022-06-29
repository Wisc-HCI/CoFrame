import { ERROR, STATUS } from "../Constants";
import { equals } from "."; 

const checkChild = (child,nullValid) => {
    if ((equals(child,{})||child===null) && !nullValid) {
        return [STATUS.FAILED, ERROR.MISSING_PARAMETER]
    } else if (child.properties) {
        return [child.properties.status,child.properties.status === STATUS.VALID ? null : ERROR.CHILD_FAILED]
    } else {
        return [STATUS.VALID, null]
    }
}

export const propertyCompiler = ({data,properties,objectTypes}) => {
    // console.log('propertyCompiler',properties)
    let status = STATUS.VALID;
    let errorCode = null;
    const objectSpec = objectTypes[data.type];
    Object.keys(properties).forEach(propKey=>{
        // console.log('considering ',propKey);
        if (objectSpec.properties[propKey].isList) {
            properties[propKey].forEach(entry=>{
                let [innerStatus, innerErrorCode] = checkChild(entry,objectSpec.properties[propKey].nullValid);
                if (innerStatus === STATUS.FAILED || innerStatus === STATUS.WARN && status !== STATUS.FAILED) {
                    // Status is failed/warned, so the parent is also.
                    status = innerStatus;
                    errorCode = innerErrorCode;
                };
            })
        } else {
            let [innerStatus, innerErrorCode] = checkChild(properties[propKey],objectSpec.properties[propKey].nullValid);
            if (innerStatus === STATUS.FAILED || innerStatus === STATUS.WARN && status !== STATUS.FAILED) {
                // Status is failed/warned, so the parent is also.
                status = innerStatus;
                errorCode = innerErrorCode;
            };
        }
    })
    return {...properties,status,errorCode}
}