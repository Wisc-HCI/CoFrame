import React, {
  useCallback,
  useState,
  useEffect,
  useLayoutEffect,
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
  Alert,
  Checkbox,
  useTheme,
  Collapse as MuiCollapse,
  Badge,
} from "@mui/material";
import {
  FiAlertCircle,
  FiAlertTriangle,
  FiEdit2,
  FiSave,
  FiX,
} from "react-icons/fi";
import useStore from "../../stores/Store";
import { shallow } from "zustand/shallow";
import { pickBy, mapValues } from "lodash";
import { useInterval } from "../useInterval";
import { SimplePositionInput } from "../Detail/PositionInput";
import { SimpleRotationInput } from "../Detail/RotationInput";
import { Solver } from "@people_and_robots/lively";
import {
  createEnvironmentModel,
  createStaticEnvironment,
  eulerFromQuaternion,
  poseToGoalPosition,
  queryWorldPose,
  attachmentToEEPose,
  distance,
} from "../../helpers/geometry";
import { likStateToData } from "../../helpers/conversion";
import { Matrix4, Quaternion, Vector3, Object3D } from "three";
import { poseToColor } from "../../helpers/computedSlice";

Object3D.DEFAULT_UP = new Vector3(0, 0, 1);
const DEFAULT_POSE = {
  position: { x: 0, y: 0, z: 0 },
  rotation: { x: 0, y: 0, z: 0, w: 1 },
};

const getGoalTransformer = (positionOffset, rotationOffset, invert) => {
  let m = new Matrix4().compose(
    new Vector3(positionOffset.x, positionOffset.y, positionOffset.z),
    new Quaternion(
      rotationOffset.x,
      rotationOffset.y,
      rotationOffset.z,
      rotationOffset.w
    ),
    new Vector3(1.0, 1.0, 1.0)
  );

  if (invert) {
    m = m.invert();
  }

  const transformer = (pose) => {
    let originalM = new Matrix4().compose(
      new Vector3(
        pose.translation ? pose.translation[0] : pose.position.x,
        pose.translation ? pose.translation[1] : pose.position.y,
        pose.translation ? pose.translation[2] : pose.position.z
      ),
      new Quaternion(
        Array.isArray(pose.rotation) ? pose.rotation[0] : pose.rotation.x,
        Array.isArray(pose.rotation) ? pose.rotation[1] : pose.rotation.y,
        Array.isArray(pose.rotation) ? pose.rotation[2] : pose.rotation.z,
        Array.isArray(pose.rotation) ? pose.rotation[3] : pose.rotation.w
      ),
      new Vector3(1.0, 1.0, 1.0)
    );

    let newM = new Matrix4().multiplyMatrices(originalM, m);
    let position = new Vector3().setFromMatrixPosition(newM);
    let rotation = new Quaternion().setFromRotationMatrix(newM);
    return {
      position: { x: position.x, y: position.y, z: position.z },
      rotation: { x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w },
    };
  };
  return transformer;
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
        weight: 100,
      },
      rotation: {
        type: "OrientationMatch",
        name: "OrientationMatch",
        link: attachmentLink,
        weight: 60,
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
    position1.distanceTo(position2) < 0.01 &&
    rotation1.angleTo(rotation2) < 0.01
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

  const [pose, setPose] = useState({
    position: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0, w: 1 },
  });
  const [joints, setJoints] = useState(
    activeCombination.robot.properties.initialJointState
  );

  const onSetData = (poseData, jointStateData) => {
    setPose(poseData);
    setJoints(jointStateData);
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
  const combinations = useStore(
    useCallback(
      (state) => {
        let combinations = [];
        let robotOptions = pickBy(state, (v) => v.type == "robotAgentType");
        let gripperOptions = pickBy(
          state,
          (v) => v.type === "gripperAgentType"
        );
        Object.values(robotOptions).forEach((robotInfo) => {
          Object.values(gripperOptions).forEach((gripperInfo) => {
            let attachmentLink = state[gripperInfo.properties.attachmentLink];
            if (attachmentLink.properties.agent === robotInfo.id) {
              combinations.push((state[robot], gripperInfo));
            }
          });
        });

        return combinations;
      },
      [robot]
    ),
    shallow
  );

  return (
    <RobotStateConfigurator
      combinations={combinations}
      showOptions={true || combinations.length > 1}
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
    };
  });

  // Create a (waypoint or location marker)
  items[`${attachmentLink}-tag-robotstateconfig`] = {
    id: `${attachmentLink}-tag-robotstateconfig`,
    frame: "world",
    shape: visualType === "location" ? "flag" : "tag",
    position: goalPose.position,
    rotation: { w: 1, x: 0, y: 0, z: 0 },
    scale: { x: -0.25, y: 0.25, z: 0.25 },
    highlighted: false,
    hidden: false,
    color: poseToColor(
      { properties: goalPose, reachable: reachable },
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
    position: goalPose.position,
    rotation: goalPose.rotation,
    scale: { x: 1, y: 1, z: 1 },
    highlighted: false,
    showName: false,
    hidden: false,
    color: poseToColor(
      { properties: goalPose, reachable: reachable },
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

export const RobotStateConfigurator = ({
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
  onSetData = (poseData, jointStateData) => {},
}) => {
  const theme = useTheme();
  const frame = useStore((state) => state.frame, shallow);
  const occupancyZones = useStore(
    (state) => pickBy(state.programData, (item) => item.type === "zoneType"),
    shallow
  );
  const [editing, setEditing] = useState(false);
  const [editMode, setEditMode] = useState(null);
  const [reached, setReached] = useState(true);

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

  const [origin, setOrigin] = useState({
    translation: [0, 0, 0],
    rotation: [0, 0, 0, 1],
  });

  const [staticEnvironment, setStaticEnvironment] = useState([]);

  const fwdTransformer = getGoalTransformer(
    activeCombination.gripper.properties.gripPositionOffset,
    activeCombination.gripper.properties.gripRotationOffset,
    false
  );

  const invTransformer = getGoalTransformer(
    activeCombination.gripper.properties.gripPositionOffset,
    activeCombination.gripper.properties.gripRotationOffset,
    true
  );

  const [goalJoints, setGoalJoints] = useState(jointState);
  const [gripperGoalPose, setGripperGoalPose] = useState(pose || DEFAULT_POSE);
  const [stateData, setStateData] = useState({ links: {} });

  const setup = () => {
    console.info("updating initial");
    const programData = useStore.getState().programData;
    const model = createEnvironmentModel(programData);
    const env = createStaticEnvironment(model);
    const basePose = queryWorldPose(model, activeCombination.robot.id);

    const o = {
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
    setOrigin(o);
    setStaticEnvironment(env);

    let s = getLivelyInput(
      o,
      activeCombination.robot,
      goalJoints,
      activeCombination.gripper.properties.relativeTo,
      env
    );

    if (Object.keys(jointState).length === 0) {
      // If there is no joint-state info, just do IK and populate stuff
      console.log("populating from pose");
      const g = invTransformer(pose);
      const state = s.solve(
        {
          position: {
            Translation: [g.position.x, g.position.y, g.position.z],
          },
          rotation: {
            Rotation: [g.rotation.x, g.rotation.y, g.rotation.z, g.rotation.w],
          },
        },
        {},
        Date.now() / 1000,
        []
      );
      const sd = likStateToData(
        state,
        activeCombination.robot.id,
        activeCombination.robot.properties.linkParentMap
      );
      const agp = fwdTransformer(
        state.frames[activeCombination.gripper.properties.relativeTo].world
      );
      setGripperGoalPose(pose);
      setGoalJoints(mapValues(state.joints, (v) => Number(v.toFixed(3))));
      setReached(checkResult(agp, pose));
      setStateData(sd);
    } else if (!pose) {
      // If there is no pose info, do FK and populate the gripper goal pose
      console.log("populating from joints");
      const currentState = s.currentState;
      const state = s.forward({
        origin: currentState.origin,
        joints: jointState,
      });
      const sd = likStateToData(
        state,
        activeCombination.robot.id,
        activeCombination.robot.properties.linkParentMap
      );
      const agp = fwdTransformer(
        state.frames[activeCombination.gripper.properties.relativeTo].world
      );
      setGripperGoalPose(agp);
      setGoalJoints(jointState);
      setReached(true);
      setStateData(sd);
    } else {
      // Otherwise, compute based on FK, and compare to the pose specified
      console.log("populating from joints and checking against pose");
      const currentState = s.currentState;
      const state = s.forward({
        origin: currentState.origin,
        joints: jointState,
      });
      const sd = likStateToData(
        state,
        activeCombination.robot.id,
        activeCombination.robot.properties.linkParentMap
      );
      const agp = fwdTransformer(
        state.frames[activeCombination.gripper.properties.relativeTo].world
      );
      setReached(checkResult(agp, gripperGoalPose));
      setStateData(sd);
      setGoalJoints(jointState);
    }
  };
  useEffect(setup, [pose, jointState, activeCombination.robot, activeCombination.gripper]);

  const [solver, setSolver] = useState(null);

  useEffect(() => {
    console.log("handling editing change", editing);
    if (editing) {
      setSolver(
        getLivelyInput(
          origin,
          activeCombination.robot,
          goalJoints,
          activeCombination.gripper.properties.relativeTo,
          staticEnvironment
        )
      );
      setCustomMoveHook((id, source, worldTransform, localTransform) => {
        const goal = {
          position: worldTransform.position,
          rotation: worldTransform.quaternion,
        };
        onNewPoseGoal(goal);
      });
      setCaptureFocus(true);
    } else {
      setEditMode(null);
      setSolver(null);
      setCustomMoveHook(null);
      setCaptureFocus(false);
    }
  }, [editing, editMode]);

  useEffect(() => {
    console.log("detecting change in visuals");
    const { tfs, items } = sceneFromState({
      stateData,
      goalPose: gripperGoalPose,
      visualType,
      editMode,
      attachmentLink: activeCombination.gripper.properties.relativeTo,
      frame,
      occupancyZones,
      reachable: reached,
      linkParentMap: activeCombination.robot.properties.linkParentMap,
    });
    partialSceneState({ tfs, items });
  }, [
    stateData,
    gripperGoalPose,
    visualType,
    editMode,
    activeCombination.gripper.properties.relativeTo,
    frame,
    occupancyZones,
    reached,
    activeCombination.robot.properties.linkParentMap,
  ]);

  const toggleEditing = () => {
    if (editing) {
        onSetData(gripperGoalPose,goalJoints)
    }
    setEditing(!editing);
  };
  const cancelEditing = () => {
    setEditing(false);
    setup();
  }

  const onNewPoseGoal = (newPose) => {
    const g = invTransformer(newPose);
    const state = solver.solve(
      {
        position: {
          Translation: [g.position.x, g.position.y, g.position.z],
        },
        rotation: {
          Rotation: [g.rotation.x, g.rotation.y, g.rotation.z, g.rotation.w],
        },
      },
      {},
      Date.now() / 1000,
      []
    );
    const sd = likStateToData(
      state,
      activeCombination.robot.id,
      activeCombination.robot.properties.linkParentMap
    );
    const agp = fwdTransformer(
      state.frames[activeCombination.gripper.properties.relativeTo].world
    );
    setGripperGoalPose(newPose);
    setGoalJoints(mapValues(state.joints, (v) => Number(v.toFixed(3))));
    setReached(checkResult(agp, newPose));
    setStateData(sd);
  };

  const onNewJointGoal = (newJoints) => {
    if (solver && editMode === "joints") {
      console.log("updating from joints...");
      const currentState = solver.currentState;
      const state = solver.forward({
        origin: currentState.origin,
        joints: newJoints,
      });
      solver.reset(state, {});
      const sd = likStateToData(
        state,
        activeCombination.robot.id,
        activeCombination.robot.properties.linkParentMap
      );
      const agp = fwdTransformer(
        state.frames[activeCombination.gripper.properties.relativeTo].world
      );
      setGripperGoalPose(agp);
      setGoalJoints(newJoints);
      setReached(true);
      setStateData(sd);
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
                <MuiCollapse in={!reached} orientation="horizontal">
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
            <SimplePositionInput
              value={gripperGoalPose.position}
              onChange={(e) => {
                const newPose = {
                  ...gripperGoalPose,
                  position: e.target.value,
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
            <SimpleRotationInput
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
            />
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
              jointState={goalJoints}
              robotJointInfo={jointLimits}
              disabled={!editing || editMode !== "joints"}
              onChange={onNewJointGoal}
            />
          </Stack>
        </Card>
      </Stack>
    </Collapse>
  );
};
