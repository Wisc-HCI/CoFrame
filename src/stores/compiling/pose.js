import { STATUS, ROOT_BOUNDS } from "../Constants";
import { Quaternion, Matrix4 } from 'three';
import { distance, likStateToData, createEnvironmentModel, queryWorldPose, quaternionLog, poseToGoalPosition } from "../helpers";
import { DATA_TYPES } from "simple-vp";


export const poseCompiler = ({data, properties, objectTypes, context, path, memo, module, worldModel}) => {

    // Create a reachabilty object to indicate who can reach this pose;
    let reachabilty = {};
    let rootPath = JSON.stringify(['root']);
    let goalPose = null;
    
    // Enumerate the robotTypes currently in the memo. This is technically unsafe, 
    // but we pre-process them beforehand so it is fine. We also always assume root execution 
    // (which is fine for robots/humans/grippers).

    const grippers = Object.values(memo).filter(v=>v.type==='gripperType');

    console.log({data,grippers})

    Object.values(memo)
        .filter(v=>v.type==='robotAgentType')
        .forEach(robotInfo=>{
            // Check to see if there are previously calculated joint values for this pose.
            const initialJointState = data.properties.compiled[path] && data.properties.compiled[path][robotInfo.id]?.joints 
                ? data.properties.compiled[path][robotInfo.id].joints 
                : robotInfo.properties.initialJointState;

            // Retrieve the robot's position/orientation
            const basePose = queryWorldPose(worldModel,robotInfo.id);
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
            const robotLinks = robotInfo.properties.compiled[rootPath].linkInfo.map(link=>link.name);

            // Enumerate grippers and search for ones that are based on this robot.
            grippers.forEach(gripper=>{
                if (robotLinks.includes(gripper.properties.relativeTo)) {

                    // Set up the objectives based on the attachment link
                    const attachmentLink = gripper.properties.relativeTo;
                    const objectives = [
                        {type:'PositionMatch',name:"EE Position",link:attachmentLink,weight:50},
                        {type:'OrientationMatch',name:"EE Rotation",link:attachmentLink,weight:25},
                        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
                    ];
                    console.log('getting goal position...',{worldModel,gripperId:gripper.id,attachmentLink,props:data.properties})
                    // console.log(worldModel)
                    // Find the position we need in the attachment link to match the desired pose gripper position
                    goalPose = poseToGoalPosition(worldModel,gripper.id,attachmentLink,data.properties);
                    console.log('goal:',goalPose)
                }
            })

            

            
            // 
            // const newCompiled = likStateToData(fwdsolver.currentState,worldModel,data.id);
        }
    )
    // const solver = new module.Solver(properties.urdf,[
    //     {type:'PositionMatch',name:"EE Position",link:"wrist_3_link",weight:50},
    //     {type:'OrientationMatch',name:"EE Rotation",link:"wrist_3_link",weight:25},
    //     {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
    //   ],ROOT_BOUNDS, createStaticEnvironment({}), null, false, 1, 450);
    // const currentTime = Date.now();
    // let status = STATUS.FAILED;
    // let state = {};
    // const pos = properties.position;
    // const rot = properties.rotation;
    // const goals = [
    //     {Translation:[pos.x,pos.y,pos.z]},
    //     {Rotation:[rot.w,rot.x,rot.y,rot.z]},
    //     null
    // ]
    // while (Date.now() - currentTime < 5000 && status === STATUS.FAILED) {
    //     state = solver.solve(goals, [50, 25, 2]);
    //     const p = state.frames.wrist_3_link.translation;
    //     const r = state.frames.wrist_3_link.rotation;
    //     const achievedPos = {x:p[0],y:p[1],z:p[2]}
    //     const goalQuat = new Quaternion(rot.x,rot.y,rot.z,rot.w)
    //     const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
    //     const translationDistance = distance(achievedPos,pos);
    //     const rotationalDistance = goalQuat.angleTo(achievedQuat);
    //     if (translationDistance < 0.01 && rotationalDistance < 0.01) {
    //         status = STATUS.VALID
    //     }
    // }

    const newCompiled = {goalPose, status: STATUS.VALID};
    return newCompiled
}