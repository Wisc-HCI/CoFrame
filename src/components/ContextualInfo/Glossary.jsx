import { TipText } from "../TipContent";

export const Glossary = {
    Program: ({primaryColor})=> <TipText color={primaryColor} previewText='Program' extraText="Entire sequence of Actions and Skills that are executed by the robot."/>,
    Actions: ({primaryColor})=> <TipText color={primaryColor} previewText='Actions' extraText="The smallest functional activity that a robot can perform. Examples include 'Move Trajectory', 'Grasp', and 'Initialize Machine'."/>,
    SkillCalls: ({primaryColor}) => <TipText color={primaryColor} previewText='Skill-Calls' extraText="Special actions that executes a procedure that is specified in a Skill block."/>,
    Skills: ({primaryColor}) => <TipText color={primaryColor} previewText='Skills' extraText="A modular collection of actions that can be executed by using a Skill-Call."/>,
    Kinematics: ({primaryColor}) => <TipText color={primaryColor} previewText='Kinematics' extraText="The physical structure (joints and links) and how they impact the movement of the robot."/>,
    Things: ({primaryColor}) => <TipText color={primaryColor} previewText='Things' extraText="Parts or pieces that have been defined in the setup tab. These may be created, consumed, or modified by machines."/>,
    Regions: ({primaryColor}) => <TipText color={primaryColor} previewText='Regions' extraText="Geometrical representations of a zone with some variability in either position or rotation."/>,
    Machines: ({primaryColor}) => <TipText color={primaryColor} previewText='Machines' extraText="Fixed tools in the environment that are capable of modifying Things according to a set of known recipes."/>,
    MoveTrajectoryPrimitives: ({primaryColor}) => <TipText color={primaryColor} previewText='Move Trajectory Primitives' extraText="A type of action that can be added and customized in the Program Editor. To transport Things, you must first Grasp them."/>,
    Trajectories: ({primaryColor}) => <TipText color={primaryColor} previewText='Trajectories' extraText="A specification of the robot's motion, utilizing a starting and ending Location, as well as a sequence of transitional waypoints between them. These are specified within the Program Editor as a block."/>
}