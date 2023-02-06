import {
    findCollisionIssues, 
    findEndEffectorPoseIssues, 
    findOccupancyIssues, 
    findPinchPointIssues, 
    findThingSafetyIssues 
} from "./issueDetectors/safety";
import { 
    findEmptyBlockIssues, 
    findMissingBlockIssues, 
    findMissingParameterIssues, 
    findUnusedFeatureIssues, 
    findUnusedSkillIssues, 
    findProcessLogicIssues, 
    findThingFlowIssues 
} from "./issueDetectors/quality";
import { 
    findEndEffectorSpeedIssues, 
    findJointSpeedIssues, 
    findPayloadIssues, 
    findReachabilityIssues, 
    findSpaceUsageIssues 
} from "./issueDetectors/performance";
import { findCycleTimeIssues, findIdleTimeIssues, findReturnOnInvestmentIssues } from "./issueDetectors/business";
import { objectMap } from "./helpers";
import lodash from 'lodash';
import { createEnvironmentModel } from "../helpers/geometry";
import useCompiledStore from './CompiledStore';
import { arrayEqual } from "../helpers/performance";

export const ReviewSlice = (set, get) => ({
    // Issues are stored in a flat lookup
    stats: [],
    // Settings for the issue detectors
    issueSettings: {
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
    },
    issues: {
        /* 
        Each issue has the following structure:
        {
            id: str (unique uuid for this issue)
            requiresChanges: bool (whether the issue is one that requires changes to the program and a refresh. Similar to warning vs. error)
            title: str (short title for issue)
            description: str (text string for error information, displayed in issue)
            complete: bool (mainly for issues that don't require changes, whether it has been marked as complete)
            focus: array[str] (what items to focus on in the gui when issue is selected)
            featuredDocs: {id: str (markdown)},
            graphData: {
                series : [{x: ..., '...': ..., ....},...],
                xAxisLabel: '',
                yAxisLabel: '',
                title: '',
                thresholds : [{range: [0, 1], color: '', label: ''}],
                units: '',
                decimal: 0,
                isTimeseries: true/false
            }
            sceneData: 🤷
            code: str (whatever we want)
        }
        */
    },
    frameNames: {
      safety: "Safety Concerns",
      quality: "Program Quality",
      performance: "Robot Performance",
      business: "Business Objectives",
    },
    frames: {
        "safety": {
            key: "safety",
            title: "Safety Concerns",
            sections: [
                "endEffectorPoses",
                "thingSafety",
                "pinchPoints",
                "collisions",
                "occupancy",
            ],
        },
        "quality": {
            key: "quality",
            title: "Program Quality",
            sections: [
                "missingBlocks",
                "missingParameters",
                "processLogic",
                "thingFlow",
                "unusedSkills",
                "unusedFeatures",
                "emptyBlocks",
            ],
        },
        "performance": {
            key: "performance",
            title: "Robot Performance",
            sections: [
                "reachability",
                "jointSpeed",
                "endEffectorSpeed",
                "payload",
                "spaceUsage",
            ],
        },
        "business": {
            key: "business",
            title: "Business Objectives",
            sections: ["cycleTime", "idleTime", "returnOnInvestment"],
        }
    },
    sections: {
        endEffectorPoses:{
            name:'End Effector Poses',
            updater:findEndEffectorPoseIssues,
            dependencies:['reachability'],
            issues:[]
        },
        pinchPoints:{
            name:'Pinch Points',
            updater:findPinchPointIssues,
            dependencies:[],
            issues:[]
        }, 
        collisions:{
            name:'Collisions',
            updater:findCollisionIssues,
            dependencies:[],
            issues:[]
        },
        occupancy:{
            name:'Occupancy',
            updater:findOccupancyIssues,
            dependencies:['spaceUsage'],
            issues:[]
        },
        thingSafety:{
            name:'Thing Safety',
            updater:findThingSafetyIssues,
            dependencies:['endEffectorPoses'],
            issues:[]
        },
        missingBlocks:{
            name:'Missing Blocks',
            updater:findMissingBlockIssues,
            dependencies:[],
            issues:[]
        },
        missingParameters:{
            name:'Missing Parameters',
            updater:findMissingParameterIssues,
            dependencies:[],
            issues:[]
        },
        processLogic:{
            name:'Process Logic',
            updater:findProcessLogicIssues,
            dependencies:['missingBlocks','missingParameters'],
            issues:[]
        },
        thingFlow:{
            name:'Thing Flow',
            updater:findThingFlowIssues,
            dependencies:['missingBlocks','missingParameters'],
            issues:[]
        },
        unusedSkills:{
            name:'Unused Skills',
            updater:findUnusedSkillIssues,
            dependencies:[],
            issues:[]
        },
        unusedFeatures:{
            name:'Unused Features',
            updater:findUnusedFeatureIssues,
            dependencies:[],
            issues:[]
        },
        emptyBlocks:{
            name:'Empty Blocks',
            updater:findEmptyBlockIssues,
            dependencies:[],
            issues:[]
        },
        reachability:{
            name:'Reachability',
            updater:findReachabilityIssues,
            dependencies:[],
            issues:[]
        },
        jointSpeed:{
            name:'Joint Speed',
            updater:findJointSpeedIssues,
            dependencies:['reachability'],
            issues:[]
        },
        endEffectorSpeed:{
            name:'End Effector Speed',
            updater:findEndEffectorSpeedIssues,
            dependencies:['reachability'],
            issues:[]
        },
        payload:{
            name:'Payload',
            updater:findPayloadIssues,
            dependencies:['thingSafety'],
            issues:[]
        },
        spaceUsage:{
            name:'Space Usage',
            updater:findSpaceUsageIssues,
            dependencies:[],
            issues:[]
        },
        cycleTime:{
            name:'Cycle Time',
            updater:findCycleTimeIssues,
            dependencies:['processLogic'],
            issues:[]
        },
        idleTime:{
            name:'Idle Time',
            updater:findIdleTimeIssues,
            dependencies:[],
            issues:[]
        },
        returnOnInvestment:{
            name:'Return on Investment',
            updater:findReturnOnInvestmentIssues,
            dependencies:['cycleTime','idleTime'],
            issues:[]
        }
    },
    setIssueCompletion: (uuid,complete) => set(state=>{
        state.issues[uuid].complete = complete
    }),
    refresh: () => set(state=>{
        let newIssues = {};
        let program = lodash.filter(state.programData, function (v) {return v.type === "programType"})[0];
        let allNewStats = {};
        let environmentModel = createEnvironmentModel(state.programData);
        const compiled = useCompiledStore.getState();
        // console.log(Object.keys(compiled))
        Object.entries(state.sections).forEach(([sectionKey,section])=>{
            // Use the predefined updater to get the new issues
            let [newSectionIssues, newStats] = state.sections[sectionKey].updater({
                programData: state.programData, 
                programSpec: state.programSpec, 
                program: program, 
                stats: state.stats,
                settings: state.issueSettings,
                environmentModel: environmentModel,
                compiledData: compiled
            });
            // Augment allNewStats with the new incoming stats.
            allNewStats = {...allNewStats,...newStats};
            // Enumerate the existing issues
            // For any issues that are already completed, exist in the new set, 
            // and don't now require changes, set them as complete.
            section.issues.forEach(issueKey=>{
                let existingIssue = state.issues[issueKey];
                if (existingIssue.complete) {
                    Object.entries(newSectionIssues).forEach(([newIssueKey,newIssue])=>{
                        if (arrayEqual(newIssue.focus, existingIssue.focus) && !newIssue.requiresChanges) {
                            newSectionIssues[newIssueKey].complete = true;
                        }
                    });
                };
            })
            // Set the new issues for that section
            state.sections[sectionKey].issues = Object.keys(newSectionIssues);
            newIssues = {...newIssues, ...objectMap(newSectionIssues,(issue)=>({...issue,code:sectionKey}))};
        });
        // Update the issues set.
        state.issues = newIssues;
        // Update the stats set.
        state.stats.push(allNewStats);
        // Reset the reviewable changes counter
        state.reviewableChanges = 0;
    }),
    updateIssueSetting: (newIssueSetting) => set(state => {
        state.issueSettings[newIssueSetting.id] = newIssueSetting;
    }),
    setFeaturedDocs: (docs, active) => set({featuredDocs: docs,activeDoc: active})
});
