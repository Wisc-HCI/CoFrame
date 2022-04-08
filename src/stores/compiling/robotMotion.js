import { STATUS, STEP_TYPE } from "../Constants";
import { distance, likStateToData, createStaticEnvironment, queryWorldPose, quaternionLog, poseToGoalPosition, timeGradientFunction, timeGradientFunctionOneTailStart, timeGradientFunctionOneTailEnd } from "../helpers";
import { merge } from 'lodash';
import { Quaternion, Vector3 } from 'three';

const FRAME_TIME = 20;

const addOrMergeAnimation = (animations,idx,newLinks,newJoints,newProximity,reached,robotId,gripperId,actionId) => {
    if (animations.length > idx) {
        const currentLinks = animations[idx].data.links;
        const currentJoints = animations[idx].data.joints;
        const currentProximity = animations[idx].data.proximity;
        const currentReachability = animations[idx].data.reachability;
        animations[idx].data = {
            links:{...currentLinks,...newLinks},
            joints:{...currentJoints,...newJoints},
            proximity:[...currentProximity,...newProximity],
            reachability: merge(currentReachability,{[robotId]:{[gripperId]:reached}})
        }
    } else {
        animations.push({
            stepType: STEP_TYPE.SCENE_UPDATE,
            data: { links: newLinks, joints:newJoints, proximity:newProximity, reachability: {[robotId]:{[gripperId]:reached}} },
            source: actionId,
            time: idx * FRAME_TIME // We assume 50 frames / second
        })
    };
    return animations
}

const createIKGoals = (goalPose1,goalPose2,jointState1,jointState2,duration) => {
    const numGoals = Math.ceil(duration / FRAME_TIME);
    let goals = [];
    let idx = 0;
    const goalPos1 = new Vector3(goalPose1.position.x,goalPose1.position.y,goalPose1.position.z);
    const goalPos2 = new Vector3(goalPose2.position.x,goalPose2.position.y,goalPose2.position.z);
    const goalQuat1 = new Quaternion(goalPose1.rotation.x,goalPose1.rotation.y,goalPose1.rotation.z,goalPose1.rotation.w);
    const goalQuat2 = new Quaternion(goalPose2.rotation.x,goalPose2.rotation.y,goalPose2.rotation.z,goalPose2.rotation.w);
    const interpPos = new Vector3(0,0,0);
    const interpQuat = new Quaternion(0,0,0,1);
    let jointGoals = {...jointState1};
    while (idx <= numGoals) {
        const percent = idx/numGoals;
        interpPos = interpPos.lerpVectors(goalPos1,goalPos2,percent);
        interpQuat = interpQuat.slerpQuaternions(goalQuat1,goalQuat2,percent);
        Object.keys(jointGoals).forEach(jointKey=>{
            jointGoals[jointKey] = percent * jointState2[jointKey] + (1-percent)*jointState1[jointKey]
        })
        goals.push({
            values:[
                null,
                null,
                {Translation:[interpPos.x,interpPos.y,interpPos.z]},
                {Rotation:[interpQuat.x,interpQuat.y,interpQuat.z,interpQuat.w]},
                ...Object.keys(jointState1).map(joint=>({Scalar:jointState2[joint]}))
            ],
            weights:[10,5,50,25,...Object.keys(jointGoals).map(()=>timeGradientFunctionOneTailEnd(idx,10,-9,30,numGoals))],
            joints:jointGoals,
            position:{x:interpPos.x,y:interpPos.y,z:interpPos.z},
            rotation:{x:interpQuat.x,y:interpQuat.y,z:interpQuat.z,w:interpQuat.w}
        })
    }

}

const createJointGoals = (jointState1,jointState2,duration) => {
    const numGoals = Math.ceil(duration / FRAME_TIME);
    let goals = [];
    let idx = 0;
    let jointGoals = {...jointState1};
    while (idx <= numGoals) {
        const percent = idx/numGoals;
        Object.keys(jointGoals).forEach(jointKey=>{
            jointGoals[jointKey] = percent * jointState2[jointKey] + (1-percent)*jointState1[jointKey]
        })
        goals.push({
            values:[
                null,
                null,
                ...Object.keys(jointState1).map(joint=>({Scalar:jointGoals[joint]}))
            ],
            weights:[10,5,...Object.keys(jointGoals).map(()=>20)],
            joints:jointGoals
        })
    }
}

const createIKSensitivityTester = (attachmentLink,jointNames,endJoints,duration) => {

    const tester = (state,goal,idx) => {
        const p = state.frames[attachmentLink].translation;
        const r = state.frames[attachmentLink].rotation;
        const achievedPos = {x:p[0],y:p[1],z:p[2]}
        const goalQuat = new Quaternion(goal.rotation.x,goal.rotation.y,goal.rotation.z,goal.rotation.w)
        const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
        const translationDistance = distance(achievedPos,goal.position);
        const rotationalDistance = goalQuat.angleTo(achievedQuat);
        let passed = translationDistance <= timeGradientFunction(idx,0.01,0.2,30,duration) && rotationalDistance <= timeGradientFunction(idx,0.005,4,30,duration);
        if (!passed) {
            return false
        }
        return !jointNames.some(jointName=>{
            return Math.abs(state.joints[jointName] - endJoints[jointName]) > timeGradientFunctionOneTailEnd(idx,0.05,2*Math.PI,30,duration)
        })
    }

    return tester;
}

const createJointSensitivityTester = (jointNames) => {
    const tester = (state,goal,idx) => {
        return !jointNames.some(jointName=>{
            return Math.abs(state.joints[jointName] - goal.joints[jointName]) > 0.03
        })
    }

    return tester;
}

export const robotMotionCompiler = ({data, properties, context, path, memo, solver, module, worldModel, urdf}) => {
    const rootPath = JSON.stringify(['root']);
    const trajectory = properties.trajectory;
    /* 
    trajectory: {
      name: "Trajectory",
      accepts: ["trajectoryType"],
      default: null,
      isList: false
    },
    velocity: {
      name: 'Velocity',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 1,
      min: 0.01,
      max: 5
    },
    motionType: {
      name: 'Motion Type',
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: ['IK', 'Joint'],
      default: 'IK'
    },
    */

    let status = STATUS.VALID;
    if (
        !trajectory.id 
        || properties.velocity < 0.01
        || (trajectory.properties && trajectory.properties.status === STATUS.FAILED) 
       ) {
        return {
            status: STATUS.FAILED,
            shouldBreak: false,
            steps: []
        }
    }

    const staticEnvironment = createStaticEnvironment(worldModel);

    // const delta = properties.positionEnd - properties.positionStart;
    // if (properties.speed === 0) {
    //     status = STATUS.FAILED;
    //     return {
    //         status,
    //         shouldBreak: false,
    //         steps: []
    //     }
    // }
    // const duration = 1000 * Math.abs(delta) / properties.speed;
    // const changePerTime = delta / duration;

    // // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe, 
    // // but we pre-process them beforehand so it is fine. We also always assume root execution 
    // // (which is fine for robots/humans/grippers).

    const grippers = Object.values(memo).filter(v => v.type === 'gripperType');
    const robots = Object.values(memo).filter(v => v.type === 'robotAgentType');

    const motionType = properties.motionType;
    const velocity = properties.velocity;

    let innerAnimations = []

    const poses = [
        trajectory.properties.compiled[path].startLocation.properties.compiled[path],
        ...trajectory.properties.compiled[path].waypoints.map(wp=>wp.properties.compiled[path]),
        trajectory.properties.compiled[path].endLocation.properties.compiled[path],
    ]

    console.log(poses);


    robots.forEach(robot=>{
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
        grippers.forEach(gripper=>{
            let reachableChildren = true;
            poses.forEach(pose=>{
                if (!pose.reachability[robot.id][gripper.id]) {
                    reachableChildren = false;
                }
            })
            if (reachableChildren && robotLinks.includes(gripper.properties.relativeTo)) {
                let firstLinks = poses[0].states[robot.id][gripper.id].links;
                let firstJoints = poses[0].states[robot.id][gripper.id].joints;
                let firstProximity = poses[0].states[robot.id][gripper.id].links
                // load the first frame
                innerAnimations = addOrMergeAnimation(innerAnimations,0,firstLinks,firstJoints,firstProximity,reached,robot.id,gripper.id,data.id);

                let reached = true;

                // Instantiate the initial state and solver;
                const attachmentLink = gripper.properties.relativeTo;
                const standardObjectives = [
                    {type:'SmoothnessMacro',name:'Smoothness',weight:10},
                    {type:'CollisionAvoidance',name:"Collision Avoidance",weight:5}
                ]
                const objectives = motionType === 'IK' ?  [
                    ...standardObjectives,
                    ...[
                        {type:'PositionMatch',name:"EE Position",link:attachmentLink,weight:50},
                        {type:'OrientationMatch',name:"EE Rotation",link:attachmentLink,weight:25}
                        ],
                    ...Object.keys(firstJoints).map(jointKey=>({
                        type:'JointMatch',
                        name:`JointHelper:${jointKey}`,
                        joint:jointKey,
                        weight:4
                    }))
                ] : [
                    ...standardObjectives,
                    ...Object.keys(firstJoints).map(jointKey=>({
                        type:'JointMatch',
                        name:`JointControl:${jointKey}`,
                        joint:jointKey,
                        weight:20
                    }))
                ];
                
                // Find the position we need in the attachment link to match the desired pose gripper position
                
                // console.log('goal:',goalPose)

                // Construct the joint state information
                let state = {};

                // Construct the solver
                const initialState = {origin,joints:firstJoints};
                const solver = new module.Solver(
                    urdf,
                    objectives,
                    rootBounds, 
                    staticEnvironment, 
                    initialState, 
                    false, 
                    1, 
                    450
                );

                // Enumerate pairs of poses.
                let poseStack = [...poses];
                let idx = 0;
                while (poseStack.length > 1 && reached) {
                    const pose1 = poseStack[0];
                    const pose2 = poseStack[1];

                    const goalPose1 = pose1.goalPose; 
                    const goalPose2 = pose2.goalPose;
                    const jointState1 = pose1.states[robot.id][gripper.id].joints;
                    const jointState2 = pose2.states[robot.id][gripper.id].joints;

                    const dist = distance(goalPose1.position,goalPose2.position);
                    const duration = 1000 * dist / velocity;

                    let goals = motionType === 'IK' 
                        ? createIKGoals(goalPose1,goalPose2,jointState1,jointState2,duration) 
                        : createJointGoals(jointState1,jointState2,duration);

                    const sensitivityTester = motionType === 'IK' 
                        ? createIKSensitivityTester(attachmentLink,Object.keys(firstJoints),goals.length) 
                        : createJointSensitivityTester(Object.keys(firstJoints),goals.length);

                    while (goals.length > 0 && reached) {
                        const goal = goals[0];
                        state = solver.solve(goal.values, goal.weights);
                        reached = sensitivityTester(state,goal);
                        goals.slice(1)
                    }

                    const stateData = likStateToData(state, worldModel, robot.id)
                    
                    innerAnimations = addOrMergeAnimation(
                        innerAnimations,
                        idx,
                        stateData.links,
                        stateData.joints,
                        stateData.proximity,
                        reached,
                        robot.id,
                        gripper.id,
                        data.id
                    );

                    // Pop off the first item in the poseStack
                    poseStack = poseStack.slice(1);
                }
                
                idx += 1;

            } else {
                status = STATUS.FAILED
            }
        })
    })

    const initialStep = {
        stepType: STEP_TYPE.ACTION_START,
        data: { agent: 'robot', id: data.id },
        source: data.id,
        time: 0
    }
    const finalStep = {
        stepType: STEP_TYPE.ACTION_END,
        data: { agent: 'robot', id: data.id },
        source: data.id,
        time: innerAnimations.length > 0 ? innerAnimations[innerAnimations.length-1].time : 0
    }

    const newCompiled = {
        status,
        shouldBreak: false,
        steps: [
            initialStep,
            ...innerAnimations,
            finalStep
        ]
    }
    return newCompiled
}