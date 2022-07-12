use serde::{Serialize, Deserialize};
use crate::compiler::blocks::location::LocationBlock;
use crate::compiler::blocks::waypoint::WaypointBlock;

// use std::collections::HashMap;

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type")]
pub enum Block {
    // InputOutput(InputOutputBlock),
    // Program(ProgramBlock),
    #[serde(rename = "locationType")]
    Location(LocationBlock),
    #[serde(rename = "waypointType")]
    Waypoint(WaypointBlock),
    // Thing(ThingBlock),
    // Machine(MachineBlock),
    // Fixture(FixtureBlock),
    // Zone(ZoneBlock),
    // Tool(ToolBlock),
    // Trajectory(TrajectoryBlock),
    // Hierarchical(HierarchicalBlock),
    // Skill(SkillBlock),
    // Mesh(MeshBlock),
    // Process(ProcessBlock),
    // Delay(DelayBlock),
    // MoveGripper(MoveGripperBlock),
    // MachineInit(MachineInitBlock),
    // ProcessStart(ProcessStartBlock),
    // ProcessWait(ProcessWaitBlock),
    // MoveTrajectory(MoveTrajectoryBlock),
    // Breakpoint(BreakpointBlock),
    // RobotAgent(RobotAgentBlock),
    // HumanAgent(HumanAgentBlock),
    // Gripper(GripperBlock),
    // CollisionBody(CollisionBodyBlock),
    // CollisionShape(CollisionShapeBlock)
}