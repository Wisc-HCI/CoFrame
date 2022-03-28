import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { merge } from 'lodash';
import { STATUS, COMPILE_FUNCTIONS, REFERENCEABLE_OBJECTS } from "../Constants";

const basicAgentData = {
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        isList: false,
        fullWidth: true
      },
      position: {
        name: 'Position',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: { x: 0, y: 0, z: 0 },
        isList: false,
        fullWidth: true
      },
      rotation: {
          name: 'Rotation',
          type: SIMPLE_PROPERTY_TYPES.IGNORED,
          default: { w: 1, x: 0, y: 0, z: 0 },
          isList: false,
          fullWidth: true
      },
      relativeTo: {
          name: 'Relative To Object',
          accepts: REFERENCEABLE_OBJECTS,
          default: null,
          isList: false,
          nullValid: true
      },
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      compileFn: {
        name: 'Compile Function',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: COMPILE_FUNCTIONS.AGENT
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED
      }
    }
  }

const robotAgentFeatures = {
    name: 'Robot Agent',
    properties: {
      description: {default: 'Robot Agent'},
      initialJointState: {
        name: 'Initial Joint State',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: []
      },
      initialGripState: {
        name: 'Initial Grip State',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: 50
      },
      updateFields: {default: ['initialJointState','initialGripState','position','rotation','relativeTo']}
    }
}

const humanAgentFeatures = {
    name: 'Human Agent',
    properties: {
      description: {default: 'Human Agent'},
      updateFields: {default: ['position','rotation','relativeTo']}
    }
}

const agentTypes = {
    robotAgentType: merge(robotAgentFeatures,basicAgentData),
    humanAgentType: merge(humanAgentFeatures,basicAgentData)
}
  
export default agentTypes