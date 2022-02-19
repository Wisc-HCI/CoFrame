import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { merge } from 'lodash';

const basicAgentData = {
    name: 'robot-agent',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        isList: false,
        fullWidth: true
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