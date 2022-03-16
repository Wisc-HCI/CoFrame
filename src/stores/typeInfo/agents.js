import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { merge } from 'lodash';
import { STATUS, STEP_CALCULATOR } from "../Constants";

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
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      computeSteps: {
        name: 'Compute Steps',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STEP_CALCULATOR.NULL
      },
      steps: {
        name: 'Steps',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      }
    }
  }

const robotAgentFeatures = {
    name: 'Robot Agent',
    properties: {description: {default: 'Robot Agent'}}
}

const humanAgentFeatures = {
    name: 'Human Agent',
    properties: {description: {default: 'Human Agent'}}
}

const agentTypes = {
    robotAgentType: merge(robotAgentFeatures,basicAgentData),
    humanAgentType: merge(humanAgentFeatures,basicAgentData)
}
  
export default agentTypes