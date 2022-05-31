import { findCollisionIssues, findEndEffectorPoseIssues, findOccupancyIssues, findPinchPointIssues, findThingMovementIssues } from "./issueDetectors/safety";
import { findEmptyBlockIssues, findMissingBlockIssues, findMissingParameterIssues, findUnusedFeatureIssues, findUnusedSkillIssues, findMachineLogicIssues } from "./issueDetectors/quality";
import { findEndEffectorSpeedIssues, findJointSpeedIssues, findPayloadIssues, findReachabilityIssues, findSpaceUsageIssues } from "./issueDetectors/performance";
import { findCycleTimeIssues, findIdleTimeIssues, findReturnOnInvestmentIssues } from "./issueDetectors/business";
import { objectMap } from "./helpers";
import lodash from 'lodash';

export const ReviewSlice = (set, get) => ({
    // Issues are stored in a flat lookup
    stats: [],
    // Settings for the issue detectors
    issueSettings: {
        'eePoseWarn': {uuid: 'eePoseWarn', name: "End Effector Pose Warning Level", value: 2, min: 0},
        'eePoseErr': {uuid: 'eePoseErr', name: "End Effector Pose Error Level", value: 5, min: 0},
        'collisionWarn': {uuid: 'collisionWarn', name: "Collision Warning Level", value: 0.1, min: 0, max: 1},
        'collisionErr': {uuid: 'collisionErr', name: "Collision Error Level", value: 1, min: 0, max: 1},
        'occupancyWarn': {uuid: 'occupancyWarn', name: "Occupancy Warning Level", value: 0.8, min: 0, max: 1},
        'occupancyErr': {uuid: 'occupancyErr', name: "Occupancy Error Level", value: 1, min: 0, max: 1},
        'jointMaxSpeed': {uuid: 'jointMaxSpeed', name: "Max Joint Speed", value: 10, min: 0},
        'jointSpeedWarn': {uuid: 'jointSpeedWarn', name: "Joint Speed Warning Level (% of max speed)", value: 0.1, min: 0, max: 1},
        'jointSpeedErr': {uuid: 'jointSpeedErr', name: "Joint Speed Error Level (% of max speed)", value: 0.5, min: 0, max: 1},
        'eeSpeedWarn': {uuid: 'eeSpeedWarn', name: "End Effector Speed Warning Level", value: 0.3, min: 0, max: 1},
        'eeSpeedErr': {uuid: 'eeSpeedErr', name: "End Effector Speed Error Level", value: 0.45, min: 0, max: 1},
        'spaceUsageWarn': {uuid: 'spaceUsageWarn', name: "Space Usage Warning Level", value: 0.01, min: 0, max: 1},
        'spaceUsageErr': {uuid: 'spaceUsageErr', name: "Space Usage Error Level", value: 0.2, min: 0, max: 1},
        'productValue': {uuid: 'productValue', name: "Product Value", value: 10, min: 0},
        'productCost': {uuid: 'productCost', name: "Product Cost", value: 5, min: 0},
        'roiAccelError': {uuid: 'roiAccelError', name: "ROI Acceleration Error Level", value: 10, min: 0},
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
            focus: {id:str, type:str} (what type of item to focus on in the gui when issue is selected)
            graphData: {series : [{x: ..., '...': ..., ....},...], lineColors: ['#00000', ...],  xAxisLabel: '', yAxisLabel: '', title: ''}
            sceneData: 🤷
            code: str (whatever we want)
        }
        */
    },
    sections: {
        endEffectorPoses:{
            name:'End Effector Poses',
            updater:findEndEffectorPoseIssues,
            dependencies:['reachability'],
            // dependencies:[],
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
        thingMovement:{
            name:'Thing Movement',
            updater:findThingMovementIssues,
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
        machineLogic:{
            name:'Machine Logic',
            updater:findMachineLogicIssues,
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
            dependencies:['thingMovement'],
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
            dependencies:['machineLogic'],
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
        let program = lodash.filter(get().programData, function (v) {return v.type === "programType"})[0];
        let allNewStats = {};
        Object.entries(state.sections).forEach(([sectionKey,section])=>{
            if (["returnOnInvestment", "idleTime", "cycleTime", "pinchPoints"].includes(sectionKey)) {
                // Use the predefined updater to get the new issues
                let [newSectionIssues, newStats] = state.sections[sectionKey].updater({
                    state: get().programData, 
                    program: program, 
                    stats: state.stats,
                    settings: state.issueSettings
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
                            if (newIssue.focus.id === existingIssue.focus.id && !newIssue.requiresChanges) {
                                newSectionIssues[newIssueKey].complete = true;
                            }
                        })
                    };
                })
                // Set the new issues for that section
                state.sections[sectionKey].issues = Object.keys(newSectionIssues);
                newIssues = {...newIssues, ...objectMap(newSectionIssues,(issue)=>({...issue,code:sectionKey}))};
            }
        })
        // Update the issues set.
        state.issues = newIssues;
        // Update the stats set.
        state.stats.push(allNewStats);
    }),
    updateIssueSetting: (newIssueSetting) => set(state => {
        state.issueSettings[newIssueSetting.uuid] = newIssueSetting;
    })
});
