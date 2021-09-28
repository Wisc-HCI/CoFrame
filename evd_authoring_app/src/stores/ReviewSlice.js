import { findCollisionIssues, findEndEffectorPoseIssues, findOccupancyIssues, findPinchPointIssues, findThingMovementIssues } from "./issueDetectors/safety";
import { findEmptyBlockIssues, findMissingBlockIssues, findMissingParameterIssues, findUnusedFeatureIssues, findUnusedSkillIssues, findMachineLogicIssues } from "./issueDetectors/quality";
import { findEndEffectorSpeedIssues, findJointSpeedIssues, findPayloadIssues, findReachabilityIssues, findSpaceUsageIssues } from "./issueDetectors/performance";
import { findCycleTimeIssues, findIdleTimeIssues, findReturnOnInvestmentIssues } from "./issueDetectors/business";
// import { objectMap } from "./helpers";

export const ReviewSlice = (set, get) => ({
    // Issues are stored in a flat lookup
    stats: [],
    issues: {
        /* 
        Each issue has the following structure:
        {
            uuid: str (unique uuid for this issue)
            requiresChanges: bool (whether the issue is one that requires changes to the program and a refresh. Similar to warning vs. error)
            title: str (short title for issue)
            description: str (text string for error information, displayed in issue)
            complete: bool (mainly for issues that don't require changes, whether it has been marked as complete)
            focus: {uuid:str, type:str} (what type of item to focus on in the gui when issue is selected)
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
            // dependencies:[],
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
            dependencies:[],
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
        //state.issues = {};
        let newIssues = {};
        let unrolledProgram = get().executablePrimitives[state.uuid];
        let allNewStats = {};
        Object.entries(state.sections).forEach(([sectionKey,section])=>{
            // Use the predefined updater to get the new issues
            let [newSectionIssues, newStats] = state.sections[sectionKey].updater({program:state,unrolled:unrolledProgram,stats:state.stats});
            // Augment allNewStats with the new incoming stats.
            allNewStats = {...allNewStats,...newStats};
            // Enumerate the existing issues
            // For any issues that are already completed, exist in the new set, 
            // and don't now require changes, set them as complete.
            section.issues.forEach(issueKey=>{
                let existingIssue = state.issues[issueKey];
                if (existingIssue.complete) {
                    Object.entries(newSectionIssues).forEach(([newIssueKey,newIssue])=>{
                        //console.log(`${newIssue.focus.type} ${existingIssue.focus.type} ${newIssue.focus.uuid} ${existingIssue.focus.uuid}`)
                        if (newIssue.focus.uuid === existingIssue.focus.uuid && !newIssue.requiresChanges) {
                            newSectionIssues[newIssueKey].complete = true;
                        }
                    })
                };
            })
            // Set the new issues for that section
            state.sections[sectionKey].issues = Object.keys(newSectionIssues);
            newIssues = {...newIssues, ...newSectionIssues};
        })
        // Update the issues set.
        state.issues = newIssues;
        // Update the stats set.
        state.stats.push(allNewStats);
        console.log(newIssues)
    })
});
