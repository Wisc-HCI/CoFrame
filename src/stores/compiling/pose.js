import { STATUS, ROOT_BOUNDS } from "../Constants";
import { Quaternion, Matrix4 } from 'three';
import { distance, likStateToData, createStaticEnvironment, queryWorldPose, quaternionLog, poseToGoalPosition } from "../helpers";
import { DATA_TYPES } from "simple-vp";


export const poseCompiler = ({data, properties, path, memo, module, worldModel}) => {

    // Create a reachabilty object to indicate who can reach this pose;
    let reachability = {};
    let states = {};
    let rootPath = JSON.stringify(['root']);
    let goalPose = null;
    
    // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe, 
    // but we pre-process them beforehand so it is fine. We also always assume root execution 
    // (which is fine for robots/humans/grippers).

    const grippers = Object.values(memo).filter(v=>v.type==='gripperType');

    const staticEnvironment = createStaticEnvironment(worldModel);

    Object.values(memo)
        .filter(v=>v.type==='robotAgentType')
        .forEach(robot=>{

            reachability[robot.id] = {};
            states[robot.id] = {}

            // Retrieve the robot's position/orientation
            const basePose = queryWorldPose(worldModel,robot.id);
            const quatLog = quaternionLog(basePose.rotation);
            const rootBounds = [
                {value:basePose.position.x,delta:0.0},{value:basePose.position.y,delta:0.0},{value:basePose.position.z,delta:0.0}, // Translational
                {value:quatLog[0],delta:0.0},{value:quatLog[1],delta:0.0},{value:quatLog[2],delta:0.0}  // Rotational  
            ];
            const origin = {
                translation:[basePose.position.x,basePose.position.y,basePose.position.z],
                rotation:[basePose.rotation.x,basePose.rotation.y,basePose.rotation.z,basePose.rotation.w]
            };

            // Get a list of links in the robot;
            const robotLinks = robot.properties.compiled[rootPath].linkInfo.map(link=>link.name);
            const urdf = robot.properties.compiled[rootPath].urdf;
            const robotInitialJointState = robot.properties.compiled[rootPath].initialJointState;

            // Enumerate grippers and search for ones that are based on this robot.
            grippers.forEach(gripper=>{
                if (robotLinks.includes(gripper.properties.relativeTo)) {

                    // Check to see if there are previously calculated joint values for this pose.
                    const initialJointState = data.properties.compiled?.[path]?.states[robot.id]?.[gripper.id]?.joints  
                        ? data.properties.compiled[path].states[robot.id][gripper.id].joints 
                        : robotInitialJointState;

                    // Set up the objectives based on the attachment link
                    const attachmentLink = gripper.properties.relativeTo;
                    const objectives = [
                        {type:'PositionMatch',name:"EE Position",link:attachmentLink,weight:50},
                        {type:'OrientationMatch',name:"EE Rotation",link:attachmentLink,weight:25},
                        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
                    ];
                    
                    // Find the position we need in the attachment link to match the desired pose gripper position
                    goalPose = poseToGoalPosition(worldModel,gripper.id,attachmentLink,properties); 
                    // console.log('goal:',goalPose)

                    // Construct the joint state information
                    let state = {};

                    // Construct the solver
                    const initialState = {origin,joints:initialJointState};
                    const solver = new module.Solver(
                        urdf,
                        objectives,
                        rootBounds, 
                        staticEnvironment, 
                        initialState, 
                        false, 
                        5, 
                        50
                    );

                    // Construct the goals
                    const pos = goalPose.position;
                    const rot = goalPose.rotation;
                    const goals = [
                        {Translation:[pos.x,pos.y,pos.z]},
                        {Rotation:[rot.x,rot.y,rot.z,rot.w]},
                        null
                    ]
                    let goalAchieved = false;
                    let currentTime = Date.now();

                    while (Date.now() - currentTime < 10000 && !goalAchieved) {
                        state = solver.solve(goals, [50, 30, 2]);
                        const p = state.frames[attachmentLink].translation;
                        const r = state.frames[attachmentLink].rotation;
                        const achievedPos = {x:p[0],y:p[1],z:p[2]}
                        const goalQuat = new Quaternion(rot.x,rot.y,rot.z,rot.w)
                        const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
                        const translationDistance = distance(achievedPos,pos);
                        const rotationalDistance = goalQuat.angleTo(achievedQuat);
                        console.log({translationDistance,rotationalDistance})
                        // if (translationDistance < 0.03 && rotationalDistance < 0.01) {
                        //     goalAchieved = true
                        // }
                        if (translationDistance < 0.03) {
                            goalAchieved = true
                        }
                        
                    }
                    console.log(goalAchieved)
                    reachability[robot.id][gripper.id] = goalAchieved;
                    states[robot.id][gripper.id] = likStateToData(state, worldModel, robot.id)

                }
            })
        })
    const newCompiled = {goalPose, states, reachability, status: STATUS.VALID, otherPropertyUpdates:{states,reachability}};
    return newCompiled
}