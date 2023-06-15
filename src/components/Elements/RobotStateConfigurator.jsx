import React, {
  useCallback,
  useState,
  useEffect,
  memo,
  startTransition,
} from "react";
import { JointInput } from "./JointInput";
import { Collapse } from "./Collapse";
import {
  Select,
  IconButton,
  MenuItem,
  Chip,
  OutlinedInput,
  FormControl,
  InputLabel,
  Card,
  CardHeader,
  Stack,
  Checkbox,
  useTheme,
  Collapse as MuiCollapse,
} from "@mui/material";
import { FiAlertCircle, FiEdit2, FiSave, FiX } from "react-icons/fi";
import useStore from "../../stores/Store";
import { shallow } from "zustand/shallow";
import { pickBy, mapValues } from "lodash";
// import { SimplePositionInput } from "../Detail/PositionInput";
// import { SimpleRotationInput } from "../Detail/RotationInput";
import { Solver } from "@people_and_robots/lively";
import init, { computePose } from "coframe-rust";
import {
  createEnvironmentModel,
  createStaticEnvironment,
  eulerFromQuaternion,
  getGoalTransformer,
  queryWorldPose,
  eulerToQuaternion,
  quaternionToEuler,
} from "../../helpers/geometry";
import { likStateToData } from "../../helpers/conversion";
import { Matrix4, Quaternion, Vector3, Object3D } from "three";
import { poseToColor } from "../../helpers/computedSlice";
import {
  MAX_POSE_DISTANCE_DIFF,
  MAX_POSE_ROTATION_DIFF,
} from "../../stores/Constants";
import { stringEquality } from "../../helpers/performance";
import { PoseCopier } from "./PoseCopier";
import VectorInput from "./VectorInput";

init();

Object3D.DEFAULT_UP = new Vector3(0, 0, 1);
const DEFAULT_POSE = {
  position: { x: 0, y: 0, z: 0 },
  rotation: { x: 0, y: 0, z: 0 },
};

const getLivelyInput = (
  origin,
  robot,
  initialJointState,
  attachmentLink,
  staticEnvionment
) => {
  const baseEuler = eulerFromQuaternion(
    [
      origin.rotation[3],
      origin.rotation[0],
      origin.rotation[1],
      origin.rotation[2],
    ],
    "sxyz"
  );
  // const quatLog = quaternionLog(basePose.rotation);
  const rootBounds = [
    { value: origin.translation[0], delta: 0.0 },
    { value: origin.translation[1], delta: 0.0 },
    { value: origin.translation[2], delta: 0.0 }, // Translational
    { value: baseEuler[0], delta: 0.0 },
    { value: baseEuler[1], delta: 0.0 },
    { value: baseEuler[2], delta: 0.0 }, // Rotational
  ];

  let solver = new Solver(
    robot.properties.urdf,
    {
      position: {
        type: "PositionMatch",
        name: "PositionMatch",
        link: attachmentLink,
        weight: 200,
      },
      rotation: {
        type: "OrientationMatch",
        name: "OrientationMatch",
        link: attachmentLink,
        weight: 120,
      },
      collision: {
        name: "CollisionAvoidance",
        type: "CollisionAvoidance",
        weight: 3,
      },
      jointLimit: {
        name: "JointLimits",
        type: "JointLimits",
        weight: 5,
      },
    },
    rootBounds,
    staticEnvionment,
    initialJointState ? { origin, joints: initialJointState } : null
  );
  solver.computeAverageDistanceTable();
  return solver;
};

const checkResult = (goal, achieved) => {
  const position1 = new Vector3(
    goal.position.x,
    goal.position.y,
    goal.position.z
  );
  const position2 = new Vector3(
    achieved.position.x,
    achieved.position.y,
    achieved.position.z
  );
  const rotation1 = new Quaternion(
    goal.rotation.x,
    goal.rotation.y,
    goal.rotation.z,
    goal.rotation.w
  );
  const rotation2 = new Quaternion(
    achieved.rotation.x,
    achieved.rotation.y,
    achieved.rotation.z,
    achieved.rotation.w
  );

  return (
    position1.distanceTo(position2) < MAX_POSE_DISTANCE_DIFF &&
    rotation1.angleTo(rotation2) < MAX_POSE_ROTATION_DIFF
  );
};

export const SingleRobotConfigurator = ({ robot }) => {
  const [activeCombinationIdx, setActiveCombinationIdx] = useState(0);
  const [combinations, activeCombination] = useStore(
    useCallback(
      (state) => {
        let combinations = [];
        let gripperOptions = Object.values(state.programData).filter(
          (v) => v.type && v.type === "gripperType"
        );
        gripperOptions.forEach((gripperInfo) => {
          let attachmentLink =
            state.programData[gripperInfo.properties.relativeTo];
          if (attachmentLink.properties.agent === robot) {
            combinations.push({
              robot: state.programData[robot],
              gripper: gripperInfo,
            });
          }
        });

        return [combinations, combinations[activeCombinationIdx]];
      },
      [robot, activeCombinationIdx]
    ),
    shallow
  );

  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty,
    shallow
  );

  const [joints, setJoints] = useState(
    activeCombination.robot.properties.initialJointState
  );

  const onSetData = (robotState) => {
    setJoints(robotState.joints);
    updateItemSimpleProperty(
      activeCombination.robot.id,
      "initialJointState",
      robotState.joints
    );
  };

  return (
    <RobotStateConfigurator
      combinations={combinations}
      activeCombination={activeCombination}
      showOptions={true}
      setCombination={(option) =>
        setActiveCombinationIdx(combinations.indexOf(option))
      }
      jointState={joints}
      pose={null}
      jointLimits={activeCombination.robot.properties.jointLimit}
      onSetData={onSetData}
    />
  );
};

export const PoseConfigurator = ({ pose }) => {
  console.log("pose", pose);
  const [activeCombinationIdx, setActiveCombinationIdx] = useState(0);
  const [combinations, activeCombination] = useStore(
    useCallback(
      (state) => {
        let combinations = [];
        let robotOptions = Object.values(state.programData)
          .sort()
          .filter((v) => v.type && v.type === "robotAgentType");
        let gripperOptions = Object.values(state.programData)
          .sort()
          .filter((v) => v.type && v.type === "gripperType");
        robotOptions.forEach((robotInfo) => {
          gripperOptions.forEach((gripperInfo) => {
            const attachmentLink =
              state.programData[gripperInfo.properties.relativeTo];
            if (attachmentLink.properties.agent === robotInfo.id) {
              combinations.push({
                robot: robotInfo,
                gripper: gripperInfo,
              });
            }
          });
        });

        return [combinations, combinations[activeCombinationIdx]];
      },
      [activeCombinationIdx]
    ),
    stringEquality
  );

  const updateItemSimpleProperties = useStore(
    (state) => state.updateItemSimpleProperties,
    shallow
  );

  const onSetData = (robotState, reached, gripperGoalPose) => {
    console.log("onSetData");
    // TODO: In the future, with multiple agents, you will have to enumerate the others that aren't being updated in this
    // specific call, since the goal pose might have changed, resulting in cascading reachability and joint-space changes
    let newReachability = { ...pose.properties.reachability };
    let newStates = { ...pose.properties.states };
    let gripperStatesData = newStates[activeCombination.robot.id]
      ? { ...newStates[activeCombination.robot.id] }
      : {};
    let gripperReachabilityData = newReachability[activeCombination.robot.id]
      ? { ...newReachability[activeCombination.robot.id] }
      : {};
    gripperStatesData[activeCombination.gripper.id] = robotState;
    gripperReachabilityData[activeCombination.gripper.id] = reached;
    newReachability = {
      ...newReachability,
      [activeCombination.robot.id]: gripperReachabilityData,
    };
    newStates = {
      ...newStates,
      [activeCombination.robot.id]: gripperStatesData,
    };

    updateItemSimpleProperties(pose.id, {
      reachability: newReachability,
      states: newStates,
      position: gripperGoalPose.position,
      rotation: gripperGoalPose.rotation,
    });
    // updateItemSimpleProperty(pose.id, "states", newStates);
    // updateItemSimpleProperty(pose.id, "position", gripperGoalPose.position);
    // updateItemSimpleProperty(pose.id, "rotation", gripperGoalPose.rotation);
  };

  return (
    <RobotStateConfigurator
      combinations={combinations}
      showOptions={true || combinations.length > 1}
      activeCombination={activeCombination}
      setCombination={(option) => {
        console.log("set combination hook");
        setActiveCombinationIdx(combinations.indexOf(option));
      }}
      jointState={
        pose.properties.states[activeCombination.robot.id]?.[
          activeCombination.gripper.id
        ]?.joints || {}
      }
      pose={{
        position: pose.properties.position,
        rotation: pose.properties.rotation,
      }}
      jointLimits={activeCombination.robot.properties.jointLimit}
      onSetData={onSetData}
    />
  );
};

const sceneFromState = ({
  stateData,
  goalPose,
  visualType = "location",
  editMode = "inactive",
  attachmentLink = "undefined",
  frame = "safety",
  occupancyZones = {},
  reachable = true,
  linkParentMap = {},
  invTransformer = () => {}
}) => {
  console.log("gen tfs and items");
  let tfs = {};
  let items = {};

  // Update the tfs in the robot scene
  Object.keys(stateData.links).forEach((linkId) => {
    tfs[linkId] = {
      frame: linkParentMap[linkId],
      position: stateData.links[linkId].position,
      rotation: stateData.links[linkId].rotation,
      scale: { x: 1, y: 1, z: 1 },
      highlighted: editMode !== "inactive",
    };
  });
  
  let g = invTransformer(goalPose);
  let offsetPos = new Vector3(g.position.x, g.position.y, g.position.z);
  let newPos = offsetPos.lerp(new Vector3(goalPose.position.x, goalPose.position.y, goalPose.position.z), 0.5);
  let goalQuat = eulerToQuaternion(goalPose.rotation);

  // Create a (waypoint or location marker)
  items[`${attachmentLink}-tag-robotstateconfig`] = {
    id: `${attachmentLink}-tag-robotstateconfig`,
    frame: "world",
    shape: visualType === "location" ? "flag" : "tag",
    position: goalPose.position,
    rotation: goalQuat,
    scale: { x: -0.25, y: 0.25, z: 0.25 },
    highlighted: editMode !== "inactive",
    hidden: false,
    color: poseToColor(
      { properties: {position:goalPose.position,rotation:goalQuat}, reachable: reachable },
      frame,
      true,
      occupancyZones
    ),
  };

  items[`${attachmentLink}-pointer-robotstateconfig`] = {
    id: `${attachmentLink}-pointer-robotstateconfig`,
    frame: "world",
    shape:
      visualType === "location"
        ? "package://app/meshes/LocationMarker.stl"
        : "package://app/meshes/OpenWaypointMarker.stl",
    position: {x: newPos.x, y: newPos.y, z: newPos.z},
    rotation: goalQuat,
    scale: { x: 1, y: 1, z: 1 },
    highlighted: editMode !== "inactive",
    showName: false,
    hidden: false,
    color: poseToColor(
      { properties: {position:goalPose.position,rotation:goalQuat}, reachable: reachable },
      frame,
      true,
      occupancyZones
    ),
    transformMode:
      editMode === "position"
        ? "translate"
        : editMode === "rotation"
        ? "rotate"
        : null,
  };
  console.log({ tfs, items });
  return { tfs, items };
};

export const RobotStateConfigurator = memo(
  ({
    combinations = [
      {
        robot: {
          name: "undefined",
          properties: { initialJointState: {}, jointLimit: {} },
        },
        gripper: { name: "undefined" },
      },
    ],
    setCombination = (option) => {},
    activeCombination = {
      robot: {
        name: "undefined",
        properties: { initialJointState: {}, jointLimit: {} },
      },
      gripper: { name: "undefined" },
    },
    jointState = {},
    pose = null,
    jointLimits = {},
    showOptions = false,
    visualType = "location",
    onSetData = (robotState, reached, gripperGoalPose) => {},
  }) => {
    const theme = useTheme();
    const frame = useStore((state) => state.frame, shallow);
    const occupancyZones = useStore(
      (state) => pickBy(state.programData, (item) => item.type === "zoneType"),
      shallow
    );
    const [editing, setEditing] = useState(false);
    const [editMode, setEditMode] = useState(null);

    const fwdTransformer = getGoalTransformer(
      activeCombination.gripper.properties.gripPositionOffset,
      eulerToQuaternion(activeCombination.gripper.properties.gripRotationOffset),
      false
    );

    const invTransformer = getGoalTransformer(
      activeCombination.gripper.properties.gripPositionOffset,
      eulerToQuaternion(activeCombination.gripper.properties.gripRotationOffset),
      true
    );

    const [configuratorState, setConfiguratorState] = useState(useCallback(()=>{
      console.info("updating initial");
      const programData = useStore.getState().programData;
      const model = createEnvironmentModel(programData);

      // Store static environment later
      const staticEnvironment = createStaticEnvironment(model);

      const basePose = queryWorldPose(model, activeCombination.robot.id);

      // Save origin to state
      const origin = {
        translation: [
          basePose.position.x,
          basePose.position.y,
          basePose.position.z,
        ],
        rotation: [
          basePose.rotation.x,
          basePose.rotation.y,
          basePose.rotation.z,
          basePose.rotation.w,
        ],
      };
      

      let solver = getLivelyInput(
        origin,
        activeCombination.robot,
        jointState || {},
        activeCombination.gripper.properties.relativeTo,
        staticEnvironment
      );

      // Other fields that get set:
      let reached = false;
      let goalJoints = jointState;
      let gripperGoalPose = pose || DEFAULT_POSE;
      let stateData = { links: {} };

      if (Object.keys(jointState).length === 0) {
        // If there is no joint-state info, just do IK and populate stuff
        console.log("populating from pose");
        const posePosition = pose.position;
        const poseQuaternion = eulerToQuaternion(pose.rotation);
        const g = invTransformer({position:posePosition, rotation:poseQuaternion});
        let result = computePose(
          activeCombination.robot.properties.urdf,
          {
            translation: [g.position.x, g.position.y, g.position.z],
            rotation: [g.rotation.x, g.rotation.y, g.rotation.z, g.rotation.w],
          },
          origin,
          activeCombination.gripper.properties.relativeTo,
          staticEnvironment
        );
        stateData = likStateToData(
          result.state,
          activeCombination.robot.id,
          activeCombination.robot.properties.linkParentMap
        );
        // const tmp = result.state.frames[activeCombination.gripper.properties.relativeTo].world;
        // const agp = fwdTransformer(
        //   {position:tmp.position,rotation:quaternionToEuler(tmp.rotation)}
        // );
        // setGripperGoalPose(pose);
        goalJoints = mapValues(result.state.joints, (v) => Number(v.toFixed(3)))
        reached = result.status === "Success";
      } else if (!pose) {
        // If there is no pose info, do FK and populate the gripper goal pose
        console.log("populating from joints");
        const currentState = solver.currentState;
        const state = solver.forward({
          origin: currentState.origin,
          joints: jointState,
        });
        stateData = likStateToData(
          state,
          activeCombination.robot.id,
          activeCombination.robot.properties.linkParentMap
        );
        const agp = fwdTransformer(
          state.frames[activeCombination.gripper.properties.relativeTo].world
        );
        gripperGoalPose = {position:agp.position,rotation:quaternionToEuler(agp.rotation)};
        reached = true;
      } else {
        // Otherwise, compute based on FK, and compare to the pose specified
        console.log("populating from joints and checking against pose");
        const currentState = solver.currentState;
        const state = solver.forward({
          origin: currentState.origin,
          joints: jointState,
        });
        stateData = likStateToData(
          state,
          activeCombination.robot.id,
          activeCombination.robot.properties.linkParentMap
        );
        const agp = fwdTransformer(
          state.frames[activeCombination.gripper.properties.relativeTo].world
        );
        reached = checkResult(agp, {position:gripperGoalPose.position,rotation:eulerToQuaternion(gripperGoalPose.rotation)});
        
      }

      console.log("INITIALIZED!")

      return {
        reached,
        goalJoints,
        gripperGoalPose,
        stateData,
        origin,
        staticEnvironment,
        solver
      }
    },[combinations,activeCombination,jointState,pose]))

    const [robotColor, gripperColor] = useStore(
      (state) => [
        state.programSpec.objectTypes.robotAgentType.instanceBlock.color,
        state.programSpec.objectTypes.gripperType.instanceBlock.color,
      ],
      shallow
    );
    const partialSceneState = useStore(
      (state) => state.partialSceneState,
      shallow
    );
    const setCustomMoveHook = useStore(
      (state) => state.setCustomMoveHook,
      shallow
    );
    const setCaptureFocus = useStore((state) => state.setCaptureFocus, shallow);

    // Disallows swapping to other locations/waypoints, must exit the detail window to remove capturefocus
    setCaptureFocus(true);


    

    useEffect(() => {
      console.log("handling editing change", editing);
      if (editing) {
        setCustomMoveHook((id, source, worldTransform, localTransform) => {
          const goal = {
            position: worldTransform.position,
            rotation: quaternionToEuler(worldTransform.quaternion),
          };
          onNewPoseGoal(goal);
        });
      } else {
        setEditMode(null);
        // setSolver(null);
        setCustomMoveHook(null);
      }
    }, [editing, editMode]);

    useEffect(() => {
      console.log("detecting change in visuals");
      const { tfs, items } = sceneFromState({
        stateData: configuratorState.stateData,
        goalPose: configuratorState.gripperGoalPose,
        visualType,
        editMode,
        attachmentLink: activeCombination.gripper.properties.relativeTo,
        frame,
        occupancyZones,
        reachable: configuratorState.reached,
        linkParentMap: activeCombination.robot.properties.linkParentMap,
        invTransformer
      });
      partialSceneState({ tfs, items });
    }, [
      configuratorState.stateData,
      configuratorState.gripperGoalPose,
      visualType,
      editMode,
      activeCombination.gripper.properties.relativeTo,
      frame,
      occupancyZones,
      configuratorState.reached,
      activeCombination.robot.properties.linkParentMap,
    ]);

    const toggleEditing = () => {
      if (editing) {
        onSetData(configuratorState.stateData, configuratorState.reached, configuratorState.gripperGoalPose);
      }
      setEditing(!editing);
    };
    const cancelEditing = () => {
      setEditing(false);
      setup();
    };

    const onNewPoseGoal = (newPose) => {
      
        console.log("new pose goal", newPose);
        const g = invTransformer({position:newPose.position, rotation:eulerToQuaternion(newPose.rotation)});
        let sd = {};
        let result = computePose(
          activeCombination.robot.properties.urdf,
          {
            translation: [g.position.x, g.position.y, g.position.z],
            rotation: [g.rotation.x, g.rotation.y, g.rotation.z, g.rotation.w],
          },
          configuratorState.origin,
          activeCombination.gripper.properties.relativeTo,
          configuratorState.staticEnvironment
        );
        sd = likStateToData(
          result.state,
          activeCombination.robot.id,
          activeCombination.robot.properties.linkParentMap
        );
        // const agp = fwdTransformer(
        //   result.state.frames[activeCombination.gripper.properties.relativeTo]
        //     .world
        // );
        setConfiguratorState((prev)=>({
          ...prev,
          reached: result.code === "Success",
          goalJoints: mapValues(result.state.joints, (v) => Number(v.toFixed(3))),
          gripperGoalPose: newPose,
          stateData: sd
        }))
    };

    const onNewJointGoal = (newJoints) => {
        console.log("new joint goal", newJoints);
        if (configuratorState.solver && editMode === "joints") {
          console.log("updating from joints...");
          const currentState = configuratorState.solver.currentState;
          const state = configuratorState.solver.forward({
            origin: currentState.origin,
            joints: newJoints,
          });
          configuratorState.solver.reset(state, {});
          const sd = likStateToData(
            state,
            activeCombination.robot.id,
            activeCombination.robot.properties.linkParentMap
          );
          const agp = fwdTransformer(
            state.frames[activeCombination.gripper.properties.relativeTo].world
          );

          setConfiguratorState((prev)=>({
            ...prev,
            reached: true,
            goalJoints: mapValues(newJoints, (v) => Number(v.toFixed(3))),
            gripperGoalPose: {position:agp.position,rotation:quaternionToEuler(agp.rotation)},
            stateData: sd
          }))
        }
    };

    return (
      <Collapse
        defaultOpen
        header={
          showOptions && combinations.length > 1 ? (
            <FormControl>
              <InputLabel
                size="small"
                id="demo-multiple-chip-label"
                style={{ left: 5, bottom: 5 }}
              >
                Agent-Robot Pair
              </InputLabel>
              <Select
                size="small"
                input={
                  <OutlinedInput
                    label="Agent-Robot Pair"
                    style={{ borderRadius: 100, height: 35 }}
                  />
                }
                renderValue={(selected) => {
                  return (
                    <>
                      <Chip
                        size="small"
                        style={{ backgroundColor: robotColor }}
                        label={selected.robot.name}
                      />{" "}
                      <Chip
                        size="small"
                        style={{ backgroundColor: gripperColor }}
                        label={selected.gripper.name}
                      />
                    </>
                  );
                }}
                value={activeCombination}
                onChange={(event) => setCombination(event.target.value)}
              >
                {combinations.map((combination) => (
                  <MenuItem value={combination}>
                    <Chip size="small" label={combination.robot.name} />{" "}
                    <Chip size="small" label={combination.gripper.name} />
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
          ) : (
            "Robot Configuration"
          )
        }
        spacing={2}
        extra={
          <>
            {editing && (
              <IconButton
                aria-label="clear configuration"
                size="small"
                onClick={() => cancelEditing()}
                color="error"
              >
                <FiX />
              </IconButton>
            )}
            <IconButton
              aria-label={editing ? "save configuration" : "edit configuration"}
              size="small"
              onClick={() => toggleEditing()}
              color="primary"
            >
              {editing ? <FiSave /> : <FiEdit2 />}
            </IconButton>
          </>
        }
      >
        <Stack gap={1}>
          <Card
            style={{
              padding: "10px 5px 5px 5px",
              backgroundColor: "#151515",
              boxShadow: ["position", "rotation", "pose"].includes(editMode)
                ? `inset 0px 0px 1px 2px ${theme.palette.primary.main}`
                : null,
            }}
          >
            <CardHeader
              titleTypographyProps={{ variant: "body1" }}
              title={
                <Stack
                  direction="row"
                  alignItems="center"
                  justifyContent="space-between"
                >
                  <div>
                    <Checkbox
                      size="small"
                      checked={
                        editing &&
                        ["position", "rotation", "pose"].includes(editMode)
                      }
                      disabled={!editing}
                      onChange={() => {
                        setEditMode(
                          ["position", "rotation", "pose"].includes(editMode)
                            ? null
                            : "pose"
                        );
                      }}
                    />
                    Set By Endpoint
                  </div>
                  <MuiCollapse in={!configuratorState.reached} orientation="horizontal">
                    <Chip
                      size="small"
                      avatar={
                        <FiAlertCircle
                          style={{ color: "black", borderRadius: 100 }}
                        />
                      }
                      label="Not Reached!"
                      color="warning"
                    />
                  </MuiCollapse>
                </Stack>
              }
            />
            <Stack gap={2}>
              <VectorInput
                label="Position"
                value={[
                  configuratorState.gripperGoalPose.position.x, 
                  configuratorState.gripperGoalPose.position.y, 
                  configuratorState.gripperGoalPose.position.z
                ]}
                onChange={(v) => {
                  // console.log("onChangeHandle",v)
                  const newPose = {
                    ...configuratorState.gripperGoalPose,
                    position: {x:v[0], y:v[1], z:v[2]},
                  };
                  console.log("updating from position...", newPose);
                  onNewPoseGoal(newPose);
                }}
                disabled={!editing || !["position", "pose"].includes(editMode)}
                active={editMode === "position"}
                onToggleActivity={(a) => {
                  setEditMode(a ? "position" : "pose");
                }}
              />

              <VectorInput
                label="Rotation"
                value={[
                  configuratorState.gripperGoalPose.rotation.x, 
                  configuratorState.gripperGoalPose.rotation.y, 
                  configuratorState.gripperGoalPose.rotation.z
                ]}
                onChange={(v) => {
                  // console.log("onChangeHandle",v)
                  const newPose = {
                    ...configuratorState.gripperGoalPose,
                    rotation: {x:v[0], y:v[1], z:v[2]},
                  };
                  console.log("updating from rotation...", newPose);
                  onNewPoseGoal(newPose);
                }}
                disabled={!editing || !["rotation", "pose"].includes(editMode)}
                active={editMode === "rotation"}
                onToggleActivity={(a) => {
                  setEditMode(a ? "rotation" : "pose");
                }}
              />  

              {/* <SimpleRotationInput
                value={gripperGoalPose.rotation}
                disabled={!editing || !["rotation", "pose"].includes(editMode)}
                active={editMode === "rotation"}
                onToggleActivity={(a) => {
                  setEditMode(a ? "rotation" : "pose");
                }}
                onChange={(e) => {
                  const newPose = {
                    ...gripperGoalPose,
                    rotation: e.target.value,
                  };
                  console.log("updating from rotation...", newPose);
                  onNewPoseGoal(newPose);
                }}
              /> */}
            </Stack>
          </Card>
          <Card
            style={{
              padding: "10px 5px 5px 5px",
              backgroundColor: "#151515",
              boxShadow:
                editMode === "joints"
                  ? `inset 0px 0px 1px 2px ${theme.palette.primary.main}`
                  : null,
            }}
          >
            <CardHeader
              titleTypographyProps={{ variant: "body1" }}
              title={
                <>
                  <Checkbox
                    size="small"
                    checked={editing && editMode === "joints"}
                    disabled={!editing}
                    onChange={() => {
                      setEditMode(editMode === "joints" ? null : "joints");
                    }}
                  />
                  Set By Joints
                </>
              }
            />
            <Stack gap={2}>
              <JointInput
                jointState={configuratorState.goalJoints}
                robotJointInfo={jointLimits}
                disabled={!editing || editMode !== "joints"}
                onChange={onNewJointGoal}
              />
            </Stack>
          </Card>
          <PoseCopier
            disabled={!editing}
            onSelect={(poseData) => {
              setConfiguratorState((prev)=>({
                ...prev,
                gripperGoalPose: {
                  position: poseData.properties.position,
                  rotation: poseData.properties.rotation,
                },
                goalJoints: poseData.properties.states[activeCombination.robot.id][
                  activeCombination.gripper.id
                ].joints,
                reached: poseData.properties.reachability[activeCombination.robot.id][
                  activeCombination.gripper.id
                ],
                stateData:poseData.properties.states[activeCombination.robot.id][
                  activeCombination.gripper.id
                ]
              }))
            }}
          />
        </Stack>
      </Collapse>
    );
  }
);
