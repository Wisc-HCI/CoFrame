import { DATA_TYPES } from "simple-vp";
import { generateUuid } from "../generateUuid";
import { pickBy } from "lodash";
import {
  ERROR,
  MAX_DESTROY_ITEM_DIFF,
  MAX_GRIPPER_DISTANCE_DIFF,
  MAX_GRIPPER_ROTATION_DIFF,
  ROOT_PATH,
  STATUS,
  STEP_TYPE,
} from "../Constants";
import {
  addGraspPointToModel,
  addToEnvironModel,
  createEnvironmentModel,
  distance,
  getAllChildrenFromModel,
  getUserDataFromModel,
  queryWorldPose,
  updateEnvironModel,
} from "../../helpers/geometry";
import { Quaternion } from "three";

const getArrayAsEnglish = (array) => {
  if (array.length === 0) {
    return "none";
  } else if (array.length === 1) {
    return array[0];
  } else if (array.length === 2) {
    return `${array[0]} and ${array[1]}`;
  } else {
    return array
      .map((v, i) => {
        if (i === array.length - 1) {
          return `and ${v}`;
        } else {
          return v;
        }
      })
      .join(", ");
  }
};

const getAcceptedAsText = (propInfo, programSpec) => {
  const links = propInfo.accepts.map(
    (a) => `[${programSpec.objectTypes[a].name}](${a})`
  );
  return getArrayAsEnglish(links);
};

const missingBlockDoc = `This block is missing a required inner block. You will need to drag in a suitable block from the block drawer. 
> [primary]To see what types of blocks are supported in certain other blocks, hover over the currently empty region. When multiple valid options are present, you can click the region to show all valid blocks.
`;

const getMissingParameterDoc = (
  propInfo,
  programSpec
) => `This block is missing a required inner parameter block. You will need to drag in a suitable parameter from the block drawer, or the enclosing skill's scope (if applicable). 
${getAcceptedAsText(propInfo, programSpec)}.
> [primary]To see what types of parameters are supported in certain other blocks, hover over the currently empty region. When multiple valid options are present, you can click the region to show all valid parameters.
`;

const unusedSkillDoc = `Skills behave like the functions of other programming languages, and are by default organized in the "Skills" program tab. Call blocks can be placed in the [Program](programType) or other skills. This skill specifically has been defined, but is never used. To reduce bloat of the program, consider deleting the the skill if you don't plan on using it later.
`;

const unusedFeatureDoc = `Features act as parameters to other blocks and come in a variety of types. If you don't plan on using this feature later, consider deleting it from the block drawer to reduce program bloat.
`;

const emptyBlockDoc = `Some blocks, such as the [Program](programType), [Skills](skillType), and [Hierarchicals](hierarchicalType) support sequences of other actions. However, this block contains an empty sequence, and therefore does nothing. To make the program more readable, consider deleting this block unless you plan on adding to it later.
`;

const processLogicNoInitDoc = `Processes are crucial components of successful programs, but must be organized correctly to function. In your current program, the specified machine has not been initialized. Before usage, each machine that is used must be initialized once per program. To address this issue, add a [Machine Init](machineInitType) action parameterized with the corresponding [Machine](machineType) before usage.
`;

const processLogicDoubleStartDoc = `Processes are crucial components of successful programs, but must be organized correctly to function. In your current program, you have started the machine twice without using it. This is likely an error, and can be addressed by removing the unnecessary [Process Start](processStartType) action.
`;

const processLogicNoStartDoc = `Processes are crucial components of successful programs, but must be organized correctly to function. In your current program, you have run a [Process Wait](processWaitType) with a process that hasn't been started. To address this, consider doing one of the following:
- Remove the highlighted [Process Wait](processWaitType) block
- Add a [Process Start](processStartType) with the corresponding process before the specified wait.
`;

const processLogicMachineBusyDoc = `Processes are crucial components of successful programs, but must be organized correctly to function. In your current program, you have started a machine that is already busy doing a separate process. To avoid this type of error, make sure that c
`;

const processLogicDoubleInitDoc = `Processes are crucial components of successful programs, but must be organized correctly to function. In your current program, you have initialized a machine twice. You only need to initialize each machine once in the program. To address this warning, consider removing the second initialization action.
`;

const thingFlowWrongThingDoc = `It appears that this release is not configured with the object being currently grasped. To address this issue, consider one of the following changes:
- Switch the [Thing](thingType) or [Tool](toolType) in this action to match the one in the previous [Move Gripper](moveGripperType) Action. 
- Switch the [Thing](thingType) or [Tool](toolType) in the previous action to match the one in this [Move Gripper](moveGripperType) Action. 
`;

const thingFlowNoThingDoc = `While configured with a [Thing](thingType) or [Tool](toolType), it appears that the specified object has not yet been spawned by the time you are attempting to grasp it.

This could be a result of not performing some [Process](processType) that produces the object. In this case, add a [Process Start](processStartType) action configured with a process that produces the object, such that the process finishes before the grasp is initiated.

If a [Process Start](processStartType) has already been added to produce the relevant object, it is possible that the [Process](processType) has not completed by the time the grasp is initiated. In this case, consider some of the following options:
- Add a [Process Wait](processWaitType) to make the robot wait until the [Process](processType) has completed.
- Add a [Delay](delayType) to make the robot wait a specified amount of time. If this option is chosen, make sure to time the delay accordingly.
- Have the robot perform other activities while waiting for the process to finish. These actions can include [Move Trajectory](moveTrajectoryType), or [Move Gripper](moveGripperType).
`;

const thingFlowReleaseFailDoc = `By the end of this action, the gripper's end configuration grasping width is less than or equal to the grasping width required by the [Thing](thingType) or [Tool](toolType) it is trying to grasp. This would result in the gripper clamping down too much for an effective grasp. To address this issue, adjust the End Position parameter of the action to the value required by the object.
`;

const thingFlowGraspFailDoc = `By the end of this action, the gripper's end configuration grasping width is greater than to the grasping width required by the [Thing](thingType) or [Tool](toolType) it is trying to grasp. This would result in the gripper clamping down too little for an effective grasp. To address this issue, adjust the End Position parameter of the action to the value required by the object.
`;

const thingFlowAlreadyGraspedDoc = `In this action you are attempting to grasp a [Thing](thingType) or [Tool](toolType), but the gripper is already gripping something. To address this issue, make sure to first release the object with a [Move Gripper](moveGripperType) Action before gripping something else.
`;

const thingFlowNoGraspedReleaseDoc = `In this action you are attempting to release a [Thing](thingType) or [Tool](toolType), but it isn't currently grasping it. To address this issue, make sure to first grasp the object with a [Move Gripper](moveGripperType) Action before releasing it. 
`;

const thingFlowNoMovementDoc = `In this action, you are attempting to alter the gripper state, but there is no difference between the _start position_ and the _end position_. While not true error, it is a no-op and therefore should be addressed. To fix this warning, either remove the action or adjust the _start position_ or _end position_ to be different.
`;

const thingFlowGripperPositionMismatch = `In this action, you are attempting to alter the gripper state, but a previous action has moved the gripper state to something other than the _start position_ specified in this error. To address this issue, check the previous [Move Gripper](moveGripperType) action (or the [robot's gripper](gripperAgentType) start configuration), and make sure this matches the configuration of this action.
`;

export const findMissingBlockIssues = ({ programData }) => {
  let issues = {};
  // Enumerate all primitives and notify if they are missing any trajectories
  Object.values(programData)
    .filter((v) => v.type === "moveTrajectoryType")
    .forEach((moveTrajectory) => {
      if (moveTrajectory.properties.trajectory === null) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          featuredDocs: { [moveTrajectory.id]: missingBlockDoc },
          title: `Missing trajectory block`,
          description: `A 'Move Trajectory' action is missing a required trajectory.`,
          complete: false,
          focus: [moveTrajectory.id],
          graphData: null,
        };
      }
    });
  // More missing blocks could be added later
  return [issues, {}];
};

export const findMissingParameterIssues = ({ programData, programSpec }) => {
  let issues = {};
  // Enumerate all primitives
  Object.values(programData).forEach((primitive) => {
    // Enumearate the parameter types
    if (primitive.properties) {
      Object.keys(primitive.properties).forEach((parameterName) => {
        let nullIsValid = programSpec.objectTypes[primitive.type].properties[
          parameterName
        ]
          ? programSpec.objectTypes[primitive.type].properties[parameterName]
              .nullValid
          : false;
        if (!nullIsValid && primitive.properties[parameterName] === null) {
          if (parameterName === "thing") {
            const uuid = generateUuid("issue");
            issues[uuid] = {
              id: uuid,
              requiresChanges: true,
              title: `Missing Thing parameter in action`,
              featuredDocs: {
                [uuid]: getMissingParameterDoc(
                  programSpec.objectTypes[primitive.type].properties[
                    parameterName
                  ],
                  programSpec
                ),
              },
              description: `This action does not have a defined Thing, and needs this value to be functional.`,
              complete: false,
              focus: [primitive.id],
              graphData: null,
            };
          }
          if (parameterName === "machine") {
            const uuid = generateUuid("issue");
            issues[uuid] = {
              id: uuid,
              requiresChanges: true,
              title: `Missing Machine parameter in action`,
              featuredDocs: {
                [uuid]: getMissingParameterDoc(
                  programSpec.objectTypes[primitive.type].properties[
                    parameterName
                  ],
                  programSpec
                ),
              },
              description: `This action does not have a defined Machine, and needs this value to be functional.`,
              complete: false,
              focus: [primitive.id],
              graphData: null,
            };
          }
          if (
            primitive.dataType === DATA_TYPES.INSTANCE &&
            (parameterName === "startLocation" ||
              parameterName === "endLocation")
          ) {
            const uuid = generateUuid("issue");
            const isTrajectory = primitive.type === "trajectoryType";
            const location =
              parameterName === "startLocation" ? "Start" : "End";
            issues[uuid] = {
              id: uuid,
              requiresChanges: true,
              title: isTrajectory
                ? `Missing ` + location + ` Location in trajectory`
                : `Missing Location parameter in action`,
              featuredDocs: {
                [uuid]: getMissingParameterDoc(
                  programSpec.objectTypes[primitive.type].properties[
                    parameterName
                  ],
                  programSpec
                ),
              },
              description: isTrajectory
                ? `This trajectory needs a ` +
                  location +
                  ` Location to be specified to be functional.`
                : `This action does not have a defined Location, and needs this value to be functional.`,
              complete: false,
              focus: [primitive.id],
              graphData: null,
            };
          }
          if (parameterName === "trajectory") {
            const uuid = generateUuid("issue");
            issues[uuid] = {
              id: uuid,
              requiresChanges: true,
              title: `Missing Trajectory parameter in action`,
              featuredDocs: {
                [uuid]: getMissingParameterDoc(
                  programSpec.objectTypes[primitive.type].properties[
                    parameterName
                  ],
                  programSpec
                ),
              },
              description: `This action does not have a defined Trajectory, and needs this value to be functional.`,
              complete: false,
              focus: [primitive.id],
              graphData: null,
            };
          }
          if (parameterName === "process") {
            const uuid = generateUuid("issue");
            issues[uuid] = {
              id: uuid,
              requiresChanges: true,
              title: `Missing Process parameter in action`,
              featuredDocs: {
                [uuid]: getMissingParameterDoc(
                  programSpec.objectTypes[primitive.type].properties[
                    parameterName
                  ],
                  programSpec
                ),
              },
              description: `This action does not have a defined Process, and needs this value to be functional.`,
              complete: false,
              focus: [primitive.id],
              graphData: null,
            };
          }
        }
      });
    }
  });

  Object.values(programData)
    .filter(
      (v) =>
        ["processStartType", "processWaitType"].includes(v.type) &&
        v.properties.status === STATUS.FAILED &&
        v.properties.errorCode === ERROR.MISMATCHED_GIZMO
    )
    .forEach((failed) => {
      const uuid = generateUuid("issue");
      issues[uuid] = {
        id: uuid,
        requiresChanges: true,
        title:
          failed.type === "processStartType"
            ? `Missing Gizmo in Process-Start`
            : `Missing Gizmo in Process-Wait`,
        description:
          failed.type === "processStartType"
            ? `Missing required gizmo for process-start`
            : `Missing required gizmo for process-wait`,
        featuredDocs: {
          [uuid]: getMissingParameterDoc(
            programSpec.objectTypes[v.type].properties.gizmo,
            programSpec
          ),
        },
        complete: false,
        focus: [failed.id],
        graphData: null,
        sceneData: null,
        code: null,
      };
    });

  return [issues, {}];
};

export const findUnusedSkillIssues = ({ programData }) => {
  // Right now we do a naive search of skill calls and indicate any that have no calls in the program.
  // However, this may not catch all cases where they are not used, in cases where that skill calls occurs in an unused skill.
  // This could be improved in the future.

  let issues = {};
  const allSkills = Object.values(programData)
    .filter((v) => v.type === "skillType" && v.dataType !== DATA_TYPES.CALL)
    .map((v) => {
      return v.id;
    });
  let usedSkills = [];
  Object.values(programData)
    .filter((v) => v.type === "skillType" && v.dataType === DATA_TYPES.CALL)
    .forEach((primitive) => {
      if (primitive.ref) {
        usedSkills.push(primitive.ref);
      }
    });

  const unusedSkills = allSkills.filter(
    (skill) => usedSkills.indexOf(skill) < 0
  );
  unusedSkills.forEach((skill_uuid) => {
    const skill = programData[skill_uuid];
    const uuid = generateUuid("issue");
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Skill "${skill.name}"`,
      featuredDocs: { [uuid]: unusedSkillDoc },
      description: `This skill is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [skill_uuid],
      graphData: null,
    };
  });
  return [issues, {}];
};

export const findUnusedFeatureIssues = ({ programData }) => {
  // Right now we do a naive search of primitives and indicate any that have no references to features in the program.
  // However, this may not catch all cases where they are not used, in cases where that usage occurs in an unused skill.
  // This could be improved in the future.

  let issues = {};
  const allLocations = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "locationType"
      );
    })
  );
  const allWaypoints = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "waypointType"
      );
    })
  );
  const allMachines = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "machineType"
      );
    })
  );
  const allThingPlaceholders = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "thingType"
      );
    })
  );
  const allTrajectories = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "trajectoryType"
      );
    })
  );
  const allProcesses = Object.keys(
    pickBy(programData, function (v) {
      return (
        (v.dataType === DATA_TYPES.INSTANCE ||
          v.dataType === DATA_TYPES.ARGUMENT) &&
        v.type === "processType"
      );
    })
  );

  let usedLocations = [];
  let usedWaypoints = [];
  let usedMachines = [];
  let usedThingPlaceholders = [];
  let usedTrajectories = [];
  let usedProcesses = [];

  // First, enumerate primitives and check for usage
  Object.values(programData).forEach((primitive) => {
    if (primitive.dataType !== DATA_TYPES.CALL && primitive.properties) {
      // First, handle the cases where the primitives are simple
      Object.keys(primitive.properties).forEach((paramKey) => {
        if (paramKey === "thing" && 
            primitive.properties[paramKey] !== null && 
            programData[primitive.properties[paramKey]]) {
          let thing = programData[primitive.properties[paramKey]].ref;
          usedThingPlaceholders.push(thing);
        } else if (
          paramKey === "machine" &&
          primitive.properties[paramKey] !== null && 
          programData[primitive.properties[paramKey]]
        ) {
          let machine = programData[primitive.properties[paramKey]].ref;
          usedMachines.push(machine);
        } else if (
          paramKey === "process" &&
          primitive.properties[paramKey] !== null && 
          programData[primitive.properties[paramKey]]
        ) {
          let process = programData[primitive.properties[paramKey]].ref;
          usedProcesses.push(process);
        } else if (
          paramKey === "trajectory" &&
          primitive.properties[paramKey] !== null && 
          programData[primitive.properties[paramKey]]
        ) {
          let trajectory =
            programData[primitive.properties[paramKey]].dataType ===
            DATA_TYPES.INSTANCE
              ? programData[primitive.properties[paramKey]].id
              : programData[primitive.properties[paramKey]].ref;
          usedTrajectories.push(trajectory);
        } else if (
          paramKey === "startLocation" &&
          primitive.properties[paramKey] !== null && 
          programData[primitive.properties[paramKey]]
        ) {
          let location = programData[primitive.properties[paramKey]].ref;
          usedLocations.push(location);
        } else if (
          paramKey === "endLocation" &&
          primitive.properties[paramKey] !== null && 
          programData[primitive.properties[paramKey]]
        ) {
          let location = programData[primitive.properties[paramKey]].ref;
          usedLocations.push(location);
        } else if (
          paramKey === "waypoints" &&
          primitive.properties[paramKey] !== [] && 
          programData[primitive.properties[paramKey]]
        ) {
          primitive.properties[paramKey].forEach((wp) => {
            let waypoint = programData[wp].ref;
            usedWaypoints.push(waypoint);
          });
        }
      });
    } else if (primitive.dataType === DATA_TYPES.CALL) {
      // In cases with skill-calls, check the corresponding skills for the types and do matchmaking
      const skillInfo = programData[primitive.ref];
      if (skillInfo && skillInfo.arguments) {
        skillInfo.arguments.forEach((argument) => {
          if (
            primitive.properties[argument] !== null &&
            programData[primitive.properties[argument]]
          ) {
            if (programData[argument].type === "machineType") {
              usedMachines.push(
                programData[primitive.properties[argument]].ref
              );
            } else if (programData[argument].type === "locationType") {
              usedLocations.push(
                programData[primitive.properties[argument]].ref
              );
            } else if (programData[argument].type === "waypointType") {
              usedWaypoints.push(
                programData[primitive.properties[argument]].ref
              );
            } else if (programData[argument].type === "thingType") {
              usedThingPlaceholders.push(
                programData[primitive.properties[argument]].ref
              );
            } else if (programData[argument].type === "trajectoryType") {
              usedTrajectories.push(
                programData[primitive.properties[argument]].ref
              );
            } else if (programData[argument].type === "processType") {
              usedProcesses.push(
                programData[primitive.properties[argument]].ref
              );
            }
          }
        });
      }
    }
  });

  const unusedLocations = allLocations.filter(
    (uuid) => usedLocations.indexOf(uuid) < 0
  );
  const unusedWaypoints = allWaypoints.filter(
    (uuid) => usedWaypoints.indexOf(uuid) < 0
  );
  const unusedMachines = allMachines.filter(
    (uuid) => usedMachines.indexOf(uuid) < 0
  );
  const unusedThingPlaceholders = allThingPlaceholders.filter(
    (uuid) => usedThingPlaceholders.indexOf(uuid) < 0
  );
  const unusedTrajectories = allTrajectories.filter(
    (uuid) => usedTrajectories.indexOf(uuid) < 0
  );
  const unusedProcesses = allProcesses.filter(
    (uuid) => usedProcesses.indexOf(uuid) < 0
  );

  unusedLocations.forEach((unused_feature_uuid) => {
    const location = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Location "${location.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This location is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  unusedWaypoints.forEach((unused_feature_uuid) => {
    const waypoint = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Waypoint "${waypoint.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This waypoint is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  unusedMachines.forEach((unused_feature_uuid) => {
    const machine = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Machine "${machine.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This machine is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  unusedThingPlaceholders.forEach((unused_feature_uuid) => {
    const placeholder = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    // May need to modify this issue (specifically the focus)
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Thing "${placeholder.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This thing is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  unusedTrajectories.forEach((unused_feature_uuid) => {
    const trajectory = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    // May need to modify this issue (specifically the focus)
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Trajectory "${trajectory.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This trajectory is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  unusedProcesses.forEach((unused_feature_uuid) => {
    const process = programData[unused_feature_uuid];
    const uuid = generateUuid("issue");
    // May need to modify this issue (specifically the focus)
    issues[uuid] = {
      id: uuid,
      requiresChanges: false,
      title: `Unused Process "${process.name}"`,
      featuredDocs: { [uuid]: unusedFeatureDoc },
      description: `This process is not used by the program. Consider removing it for simplicity.`,
      complete: false,
      focus: [unused_feature_uuid],
      graphData: null,
    };
  });

  return [issues, {}];
};

export const findEmptyBlockIssues = ({ programData, program }) => {
  let issues = {};
  // Enumerate skills and return warnings about ones that have a primitiveIds list of length 0
  Object.values(programData)
    .filter((v) => v.dataType !== DATA_TYPES.CALL && v.type === "skillType")
    .forEach((skill) => {
      if (skill.properties.children.length === 0) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: false,
          title: `Empty Skill: ${skill.name}`,
          featuredDocs: { [uuid]: emptyBlockDoc },
          description: `This skill is empty and contains no actions.`,
          complete: false,
          focus: [skill.id],
          graphData: null,
        };
      }
    });
  // Enumerate hierarchical and return warnings about ones that have a primitiveIds list of length 0
  Object.values(programData)
    .filter((v) => v.type === "hierarchicalType")
    .forEach((primitive) => {
      if (primitive.properties.children.length === 0) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: false,
          title: `Empty Action or Structure`,
          featuredDocs: { [uuid]: emptyBlockDoc },
          description: `This structure is empty and contains no actions. Consider removing.`,
          complete: false,
          focus: [primitive.id],
          graphData: null,
        };
      }
    });
  // Enumerate the program and return a warning if primitiveIds list is of length 0
  if (program.properties.children.length === 0) {
    const uuid = generateUuid("issue");
    issues[uuid] = {
      id: uuid,
      requiresChanges: true,
      title: `Program is empty`,
      featuredDocs: { [uuid]: emptyBlockDoc },
      description: `The program is currently empty. Add actions to make the program perform tasks.`,
      complete: false,
      focus: [program.id],
      graphData: null,
    };
  }

  return [issues, {}];
};

export const findProcessLogicIssues = ({
  program,
  programData,
  compiledData,
}) => {
  //init , started, waiting, stopped
  let issues = {};
  let machineState = {};
  let trackedActions = [];

  // "Initialize" tools for tracking.
  Object.values(programData)
    .filter((v) => v.type === "toolType")
    .forEach((tool) => {
      machineState[tool.id] = "init";
    });

  compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach((step) => {
    let source = programData[step.source];

    // Cleanup for process/action end
    if (step.type === STEP_TYPE.PROCESS_END) {
      // Reset the machine's state to the init configuration
      if (step.data.gizmo) {
        machineState[step.data.gizmo] = "init";
      }
    }

    if (step.type === STEP_TYPE.ACTION_END) {
      // Remove the process since it is no longer running
      trackedActions = trackedActions.filter((ta) => ta !== step.source);
    }

    // Check the machine has been initialized twice
    if (step.type === STEP_TYPE.LANDMARK && source.type === "machineInitType") {
      if (machineState[step.data.machine] === "init") {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: false,
          title: `Machine is already initialized`,
          featuredDocs: { [uuid]: processLogicDoubleInitDoc },
          description: `Machine has already been initialized`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "machineDoubleInit",
        };
      } else {
        machineState[step.data.machine] = "init";
      }
    } else if (
      step.type === STEP_TYPE.PROCESS_START &&
      source.type === "processStartType"
    ) {
      const needsGizmo =
        step.data.gizmo !== null && step.data.gizmo !== undefined;

      // Check that the machine has been initialized once
      if (needsGizmo && machineState[step.data.gizmo] === undefined) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          title: `Machine needs to be initialized first`,
          featuredDocs: { [uuid]: processLogicNoInitDoc },
          description: `Cannot run a process-start before the corresponding machine's machine-initialize`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "machineNoInitStart",
        };
      }

      // Check that the process is not already running
      if (trackedActions.includes(step.source)) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          title: `Process Already Running`,
          featuredDocs: { [uuid]: processLogicDoubleStartDoc },
          description: `Cannot run a process more than once before it finishes`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "processDoubleStart",
        };
      }
      // Check that the machine isn't busy
      else if (needsGizmo && machineState[step.data.gizmo] === "running") {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          title: `Machine needs to be initialized first`,
          featuredDocs: { [uuid]: processLogicMachineBusyDoc },
          description: `Cannot start a machine that is already running`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "machineDoubleStart",
        };
      }

      // Add the action to the set of running processes
      trackedActions.push(source.id);
      if (needsGizmo) {
        machineState[step.data.gizmo] = "running";
      }
    } else if (
      step.type === STEP_TYPE.ACTION_START &&
      source.type === "processWaitType"
    ) {
      const needsGizmo =
        step.data.gizmo !== null && step.data.gizmo !== undefined;

      // Check that the machine has been initialized
      if (needsGizmo && machineState[step.data.gizmo] === undefined) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          title: `Machine needs to be initialized first`,
          featuredDocs: { [uuid]: processLogicNoInitDoc },
          description: `Cannot run a process-wait before the initializing the corresponding machine`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "machineNoInitWait",
        };
        trackedActions.push(source.id);
      }

      // Check that the process is running
      if (!trackedActions.includes(source.id)) {
        const uuid = generateUuid("issue");
        issues[uuid] = {
          id: uuid,
          requiresChanges: true,
          title: `Waiting on Non-Running Process`,
          featuredDocs: { [uuid]: processLogicNoStartDoc },
          description: `Process-wait is running on a process that hasn't started`,
          complete: false,
          focus: [source.id],
          graphData: null,
          sceneData: null,
          code: "waitNoProcess",
        };
      }
    }
  });

  return [issues, {}];
};

export const findThingFlowIssues = ({ program, programData, compiledData }) => {
  let issues = {};
  let currentGrippedThing = "";
  let currentGraspPoint = "";
  let graspWidth = -1;
  let inMoveGripper = false;
  let lastMoveGripperData = {};
  let samePositionMoveGrippers = [];
  let itemExistsError = [];

  // Tracks lists of things indexed by their spawned type (example: all "blade"s are in a list indexed by "blade")
  let trackedByType = {};

  // Array of all tracking thing ids - used to know whether a given id is a thing or not
  let thingList = [];

  // Create an updatable model of the environment usng the program data
  // This is updated from the compiled data as it's encountered
  let programModel = createEnvironmentModel(programData);

  let moveGripperOrder = [];
  let gripperId = Object.values(programData).filter(
    (d) => d.type === "gripperType" && d.dataType === DATA_TYPES.INSTANCE
  )[0].id;

  compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach((step) => {
    let source = programData[step.source];

    // Thing is created/spawned
    if (step.type === STEP_TYPE.SPAWN_ITEM) {
      // Create ID for tracking, and add to array
      let id = generateUuid("thing");
      thingList.push(id);

      // Use the inputOutput (that spawned the thing) as the original position
      let ioPosition = queryWorldPose(programModel, step.data.inputOutput, "");

      // Add thing to the program model
      programModel = addToEnvironModel(
        programModel,
        "world",
        id,
        ioPosition.position,
        ioPosition.rotation
      );

      // Add thing grasp points to the program model
      let graspPoints = programData[step.data.thing].properties.graspPoints;
      graspPoints.forEach((graspId) => {
        let gID = generateUuid("graspPoint");
        programModel = addGraspPointToModel(
          programModel,
          id,
          gID,
          programData[graspId].properties.position,
          programData[graspId].properties.rotation,
          programData[graspId].properties.gripDistance
        );
      });

      // Add thing to the tracking
      if (!(step.data.thing in trackedByType)) {
        trackedByType[step.data.thing] = [
          {
            id: id,
          },
        ];
      } else {
        trackedByType[step.data.thing].push({
          id: id,
        });
      }
    }

    // Thing is consumed/destroyed
    if (step.type === STEP_TYPE.DESTROY_ITEM) {
      // Find and remove the tracked thing
      let bucket = trackedByType[step.data.thing];
      // Use whatever the last item to have been grabbed as a base
      let id = null;

      // Add a temp item to the model to see where to look for the position
      programModel = addToEnvironModel(programModel, 
          step.data?.relativeTo?.id ? step.data.relativeTo.id : 'world', 
          "tempIOPlacement", 
          step.data.position, 
          step.data.rotation
      );
      let tempItemPosition = queryWorldPose(programModel, "tempIOPlacement");

      if (bucket.length === 1) {
          let thingPos = queryWorldPose(programModel, trackedByType[step.data.thing][0].id);

          if (distance(tempItemPosition.position, thingPos.position) <= MAX_DESTROY_ITEM_DIFF) {
              id = trackedByType[step.data.thing][0].id;
              delete trackedByType[step.data.thing];
          }
      } else {
          // Remove the item that was most recently tracked
          let lst = trackedByType[step.data.thing];
          let idx = -1;

          for (let i = 0; i < lst.length; i++) {
              let thingPos = queryWorldPose(programModel, lst[i].id);
              if (distance(tempItemPosition.position, thingPos.position) <= MAX_DESTROY_ITEM_DIFF) {
                  idx = i;
                  id = lst[i].id;
                  i = lst.length;
              }
          }
          if (idx >= 0) {
              trackedByType[step.data.thing].splice(idx, 1);
          }
      }
    }

    // Update all links in the model
    if (step.type === STEP_TYPE.SCENE_UPDATE) {
      Object.keys(step.data.links).forEach((link) => {
        // Update the program model for each link
        programModel = updateEnvironModel(
          programModel,
          link,
          step.data.links[link].position,
          step.data.links[link].rotation
        );
      });

      if (currentGrippedThing !== "") {
        // Get world pose of the gripper offset and update the grasped object to this pose
        let gripperOffset = queryWorldPose(
          programModel,
          gripperId + "-gripOffset",
          ""
        );
        programModel = updateEnvironModel(
          programModel,
          currentGrippedThing,
          gripperOffset.position,
          gripperOffset.rotation
        );
      }
    }

    // Update until we get the last update of the move gripper action
    if (
      step.type === STEP_TYPE.SCENE_UPDATE &&
      source.type === "moveGripperType"
    ) {
      if (!inMoveGripper) {
        inMoveGripper = true;
      }
      lastMoveGripperData = { ...step };
    }

    // Once out of the move gripper action, use the last data point to calculate everything
    if (
      inMoveGripper &&
      !(
        step.type === STEP_TYPE.SCENE_UPDATE &&
        source.type === "moveGripperType"
      )
    ) {
      inMoveGripper = false;

      if (!moveGripperOrder.includes(lastMoveGripperData.source)) {
        moveGripperOrder.push(lastMoveGripperData.source);
      }

      let mgSource = programData[lastMoveGripperData.source];
      let thing =
        programData[
          lastMoveGripperData.data.thing.id
            ? lastMoveGripperData.data.thing.id
            : lastMoveGripperData.data.thing
        ];

      if (thing) {
        // Update model positions of all links in the gripper
        Object.keys(lastMoveGripperData.data.links).forEach((link) => {
          programModel = updateEnvironModel(
            programModel,
            link,
            lastMoveGripperData.data.links[link].position,
            lastMoveGripperData.data.links[link].rotation
          );
        });

        // Get the gripper offset position/rotation
        let gripperOffset = queryWorldPose(
          programModel,
          gripperId + "-gripOffset",
          ""
        );
        let gripperRotation = new Quaternion(
          gripperOffset.rotation.x,
          gripperOffset.rotation.y,
          gripperOffset.rotation.z,
          gripperOffset.rotation.w
        );

        // Gripper is closing
        if (
          mgSource.properties.positionStart > mgSource.properties.positionEnd
        ) {
          let bucket = trackedByType[thing.id];

          // Add tool to bucket
          if (!bucket && thing.type === "toolType") {
            trackedByType[thing.id] = [
              {
                id: thing.id,
              },
            ];
            bucket = trackedByType[thing.id];

            // Add tool grasp points to the program model
            let graspPoints = thing.properties.graspPoints;
            graspPoints.forEach((graspId) => {
              let gID = generateUuid("graspPoint");
              programModel = addGraspPointToModel(
                programModel,
                thing.id,
                gID,
                programData[graspId].properties.position,
                programData[graspId].properties.rotation,
                programData[graspId].properties.gripDistance
              );
            });
          }

          if (bucket) {
            for (let i = 0; i < bucket.length; i++) {
              // Get all potential grasp locations for a given thing
              let graspPointIDs = getAllChildrenFromModel(
                programModel,
                bucket[i].id
              );
              // Search over the grasp locations and determine whether any are within some
              // tolerance of the gripper's offset position/rotation
              let selectedId = "";
              let selectedGraspWidth = -1;

              if (graspPointIDs) {
                graspPointIDs.forEach((graspPointID) => {
                  let graspPosition = queryWorldPose(
                    programModel,
                    graspPointID,
                    ""
                  );
                  let tmpGraspWidth = getUserDataFromModel(
                    programModel,
                    graspPointID,
                    "width"
                  );
                  let graspRotation = new Quaternion(
                    graspPosition.rotation.x,
                    graspPosition.rotation.y,
                    graspPosition.rotation.z,
                    graspPosition.rotation.w
                  );
                  let distTol =
                    distance(graspPosition.position, gripperOffset.position) <=
                    MAX_GRIPPER_DISTANCE_DIFF;
                  let rotTol =
                    graspRotation.angleTo(gripperRotation) <=
                    MAX_GRIPPER_ROTATION_DIFF;

                  if (distTol && rotTol) {
                    selectedId = graspPointID;
                    selectedGraspWidth = tmpGraspWidth;
                  }
                });

                // Update grasped thing
                if (
                  selectedId !== "" &&
                  mgSource.properties.positionEnd === selectedGraspWidth &&
                  currentGrippedThing === ""
                ) {
                  currentGrippedThing = bucket[i].id;
                  currentGraspPoint = selectedId;
                  graspWidth = selectedGraspWidth;
                }

                // width is not enough
                if (
                  selectedId !== "" &&
                  mgSource.properties.positionEnd > selectedGraspWidth
                ) {
                  const id = generateUuid("issue");
                  issues[id] = {
                    id: id,
                    requiresChanges: false,
                    title: "Failed to grasp thing",
                    featuredDocs: { [mgSource.id]: thingFlowGraspFailDoc },
                    description:
                      "The move gripper's end position is greater than the grasping width of the thing",
                    complete: false,
                    focus: [mgSource.id],
                    graphData: null,
                    sceneData: null,
                    code: null,
                  };
                }

                // width is too much
                if (
                  selectedId !== "" &&
                  mgSource.properties.positionEnd < selectedGraspWidth
                ) {
                  const id = generateUuid("issue");
                  issues[id] = {
                    id: id,
                    requiresChanges: true,
                    title: "Gripper width is below grasp threshold",
                    featuredDocs: { [mgSource.id]: thingFlowGraspFailDoc },
                    description:
                      "The move gripper's end position is less than the grasping width of the thing",
                    complete: false,
                    focus: [mgSource.id],
                    graphData: null,
                    sceneData: null,
                    code: null,
                  };

                  // update grip
                  if (currentGrippedThing === "") {
                    currentGrippedThing = bucket[i].id;
                    currentGraspPoint = selectedId;
                    graspWidth = selectedGraspWidth;
                  }
                }

                // gripping the wrong thing
                if (
                  selectedId !== "" &&
                  currentGrippedThing !== "" &&
                  bucket[i].id !== currentGrippedThing
                ) {
                  const id = generateUuid("issue");
                  issues[id] = {
                    id: id,
                    requiresChanges: true,
                    title: "Grasping multiple things",
                    featuredDocs: { [mgSource.id]: thingFlowAlreadyGraspedDoc },
                    description:
                      "Another thing is currently grasped by the robot.",
                    complete: false,
                    focus: [mgSource.id],
                    graphData: null,
                    sceneData: null,
                    code: null,
                  };
                }
              }
            }
          } else if (!itemExistsError.includes(mgSource.id)) {
            itemExistsError.push(mgSource.id);
            const id = generateUuid("issue");
            issues[id] = {
              id: id,
              requiresChanges: true,
              title: "Thing has not been created",
              featuredDocs: { [mgSource.id]: thingFlowNoThingDoc },
              description:
                "Attempting to grasp thing that has not yet been spawned.",
              complete: false,
              focus: [mgSource.id],
              graphData: null,
              sceneData: null,
              code: null,
            };
          }
          // Gripper is opening
        } else if (
          mgSource.properties.positionStart < mgSource.properties.positionEnd
        ) {
          let bucket = trackedByType[thing.id];
          let bucketContainsCurrentGrip = bucket
            ? bucket.map((e) => e.id).some((e) => e === currentGrippedThing)
            : false;

          // width is still grasping
          if (
            bucketContainsCurrentGrip &&
            graspWidth !== -1 &&
            mgSource.properties.positionEnd <= graspWidth
          ) {
            const id = generateUuid("issue");
            issues[id] = {
              id: id,
              requiresChanges: false,
              title: "Failed to release thing",
              featuredDocs: { [mgSource.id]: thingFlowReleaseFailDoc },
              description:
                "Move gripper's end position is less than or equal to the grasping width of the thing",
              complete: false,
              focus: [mgSource.id],
              graphData: null,
              sceneData: null,
              code: null,
            };
          }

          // releasing wrong thing
          if (currentGrippedThing !== "" && !bucketContainsCurrentGrip) {
            const id = generateUuid("issue");
            issues[id] = {
              id: id,
              requiresChanges: false,
              title: `Releasing incorrect thing`,
              featuredDocs: { [mgSource.id]: thingFlowWrongThingDoc },
              description: `Gripper is releasing the incorrect thing`,
              complete: false,
              focus: [mgSource.id],
              graphData: null,
              sceneData: null,
              code: null,
            };
          }

          // releasing something before grabbing something
          if (currentGrippedThing === "" && thing && thing.id) {
            const id = generateUuid("issue");
            issues[id] = {
              id: id,
              requiresChanges: false,
              title: `Incorrect thing release`,
              featuredDocs: { [mgSource.id]: thingFlowNoGraspedReleaseDoc },
              description: "Robot has not previously grasped a thing",
              complete: false,
              focus: [mgSource.id],
              graphData: null,
              sceneData: null,
              code: null,
            };
          }

          // release thing
          if (
            bucketContainsCurrentGrip &&
            graspWidth !== -1 &&
            mgSource.properties.positionEnd > graspWidth
          ) {
            currentGrippedThing = "";
            currentGraspPoint = "";
            graspWidth = -1;
          }

          // Gripper has the same start and end positions
        }
      }

      if (
        !samePositionMoveGrippers.includes(mgSource.id) &&
        mgSource.properties.positionStart === mgSource.properties.positionEnd
      ) {
        samePositionMoveGrippers.push(mgSource.id);
        const id = generateUuid("issue");
        issues[id] = {
          id: id,
          requiresChanges: false,
          title: `Gripper position did not change`,
          featuredDocs: { [mgSource.id]: thingFlowNoMovementDoc },
          description: `Gripper start and end positions are the same.`,
          complete: false,
          focus: [mgSource.id],
          graphData: null,
          sceneData: null,
          code: null,
        };
      }
    }
  });

  // Find inconsistencies in gripper motion
  let previousEnd = -1;
  let currentStart = -1;
  let idx = 0;
  moveGripperOrder.forEach((moveGripperId) => {
    let source = programData[moveGripperId];

    if (idx === 0) {
      previousEnd = source.properties.positionEnd;
    } else {
      currentStart = source.properties.positionStart;

      if (currentStart !== previousEnd) {
        const id = generateUuid("issue");
        issues[id] = {
          id: id,
          requiresChanges: true,
          title: "Gripper position mismatch",
          featuredDocs: { [moveGripperId]: thingFlowGripperPositionMismatch },
          description:
            "Move gripper start position does not match previous move gripper end position",
          complete: false,
          focus: [moveGripperId],
          graphData: null,
          sceneData: null,
          code: null,
        };
      }

      previousEnd = source.properties.positionEnd;
    }

    idx += 1;
  });

  return [issues, {}];
};
