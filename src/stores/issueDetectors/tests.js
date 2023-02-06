import {performCompileProcess} from '../compiling/performCompileProcess';
import rawObjectTypes from '../typeInfo';
import {pick, omit, mapValues, filter} from "lodash";
import { instanceTemplateFromSpec, DATA_TYPES } from "simple-vp";
import {
    findCollisionIssues, 
    findEndEffectorPoseIssues, 
    findOccupancyIssues, 
    findPinchPointIssues, 
    findThingSafetyIssues 
} from "./safety";
import { 
    findEmptyBlockIssues, 
    findMissingBlockIssues, 
    findMissingParameterIssues, 
    findUnusedFeatureIssues, 
    findUnusedSkillIssues, 
    findProcessLogicIssues, 
    findThingFlowIssues 
} from "./quality";
import { 
    findEndEffectorSpeedIssues, 
    findJointSpeedIssues, 
    findPayloadIssues, 
    findReachabilityIssues, 
    findSpaceUsageIssues 
} from "./performance";
import { findCycleTimeIssues, findIdleTimeIssues, findReturnOnInvestmentIssues } from "./business";
import { createEnvironmentModel } from '../../helpers/geometry';

const detectors = [
    findCollisionIssues, 
    findEndEffectorPoseIssues, 
    findOccupancyIssues, 
    findPinchPointIssues, 
    findThingSafetyIssues,
    findEmptyBlockIssues, 
    findMissingBlockIssues, 
    findMissingParameterIssues, 
    findUnusedFeatureIssues, 
    findUnusedSkillIssues, 
    findProcessLogicIssues, 
    findThingFlowIssues,
    findEndEffectorSpeedIssues, 
    findJointSpeedIssues, 
    findPayloadIssues, 
    findReachabilityIssues, 
    findSpaceUsageIssues,
    findCycleTimeIssues, 
    findIdleTimeIssues, 
    findReturnOnInvestmentIssues
];

const issueSettings = {
    'eePoseWarn': {id: 'eePoseWarn', frame: 'safety', name: "End Effector Pose Warning Level", value: 2, min: 0},
    'eePoseErr': {id: 'eePoseErr', frame: 'safety', name: "End Effector Pose Error Level", value: 5, min: 0},
    'collisionWarn': {id: 'collisionWarn', frame: 'safety', name: "Collision Warning Level", value: 0.05, min: 0, max: 1},
    'collisionErr': {id: 'collisionErr', frame: 'safety', name: "Collision Error Level", value: 0, min: 0, max: 1},
    'occupancyWarn': {id: 'occupancyWarn', frame: 'safety', name: "Occupancy Warning Level", value: 0.05, min: 0, max: 1},
    'occupancyErr': {id: 'occupancyErr', frame: 'safety', name: "Occupancy Error Level", value: 0, min: 0, max: 1},
    'jointMaxSpeed': {id: 'jointMaxSpeed', frame: 'performance', name: "Max Joint Speed", value: 10, min: 0},
    'jointSpeedWarn': {id: 'jointSpeedWarn', frame: 'performance', name: "Joint Speed Warning Level (% of max speed)", value: 0.1, min: 0, max: 1},
    'jointSpeedErr': {id: 'jointSpeedErr', frame: 'performance', name: "Joint Speed Error Level (% of max speed)", value: 0.5, min: 0, max: 1},
    'eeSpeedWarn': {id: 'eeSpeedWarn', frame: 'performance', name: "End Effector Speed Warning Level", value: 0.3, min: 0, max: 1},
    'eeSpeedErr': {id: 'eeSpeedErr', frame: 'performance', name: "End Effector Speed Error Level", value: 0.45, min: 0, max: 1},
    'payloadWarn': {id: 'payloadWarn', frame: 'performance', name: "Robot Payload Warning Level", value: 2.5},
    'payloadErr': {id: 'payloadErr', frame: 'performance', name: "Robot Payload Error Level", value: 3},
    'spaceUsageWarn': {id: 'spaceUsageWarn', frame: 'performance', name: "Space Usage (%) Warning Level", value: 1, min: 0, max: 100},
    'spaceUsageErr': {id: 'spaceUsageErr', frame: 'performance', name: "Space Usage (%) Error Level", value: 20, min: 0, max: 100},
    'productValue': {id: 'productValue', frame: 'business', name: "Product Value", value: 10, min: 0},
    'productCost': {id: 'productCost', frame: 'business', name: "Product Cost", value: 5, min: 0},
    'roiAccelError': {id: 'roiAccelError', frame: 'business', name: "ROI Acceleration Error Level", value: 10, min: 0},
}

const cleanedObjectType = (objectType) =>
  pick(objectType, ["name", "properties", "type"]);

const cleanedProgram = (programData,objectTypes) => mapValues(programData, (d) => {
    if (d.dataType === DATA_TYPES.INSTANCE) {
      const defaultv = instanceTemplateFromSpec(
        d.type,
        objectTypes[d.type],
        false
      );
      return {
        ...d,
        properties: {
          ...defaultv.properties,
          ...omit(d.properties, [
            "status",
            "compiled",
            "compileFn",
            "updateFields",
          ]),
        },
      };
    } else {
      return d;
    }
  });

const mergedProgram = (newData,oldData) => {
    Object.keys(newData).forEach((entry) => {
        Object.keys(newData[entry].properties)
          .forEach((field) => {
            if (field !== 'compiled') {
              oldData[entry].properties[field] =
                newData[entry].properties[field];
            }
          
          // console.log(`setting ${entry}/${field} to ${newData[entry].properties[field]}`)
        });
      });
    return oldData
}

export const performIssueTest = async (jsonData) => {
    const {tabs, activeTab, ...programData} = jsonData;
    const objectTypes = mapValues(
        rawObjectTypes,
        cleanedObjectType
      )
    const result = await performCompileProcess({programData:cleanedProgram(programData,rawObjectTypes), compiledData:{}, objectTypes});

    let issues = {};
    const updatedProgram = mergedProgram(result.data,programData);
    let program = filter(updatedProgram, function (v) {return v.type === "programType"})[0];
    let environmentModel = createEnvironmentModel(result.data);
    detectors.forEach(detector=>{
        issues = {...detector({
            programData: updatedProgram, 
            programSpec: {objectTypes}, 
            program: program, 
            stats: [],
            settings: issueSettings,
            environmentModel: environmentModel,
            compiledData: result.compiledData
        })[0],...issues}
    })
    return issues;
}