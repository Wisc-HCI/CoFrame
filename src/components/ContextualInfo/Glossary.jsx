import { TipText } from "../Elements/TipContent";

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
    Trajectories: ({primaryColor}) => <TipText color={primaryColor} previewText='Trajectories' extraText="A specification of the robot's motion, utilizing a starting and ending Location, as well as a sequence of transitional waypoints between them. These are specified within the Program Editor as a block."/>,
    Processes: ({primaryColor}) => <TipText color={primaryColor} previewText='Processes' extraText="Processes are like recipes with a given set of inputs, outputs, and a duration."/>,
    Tools: ({primaryColor}) => <TipText color={primaryColor} previewText='Tools' extraText="Tools are objects in the environment that can assist in performing processes."/>,
    RobotAgents: ({primaryColor}) => <TipText color={primaryColor} previewText='Robot Agents' extraText="Robot Agents are capable of moving about the space and performing actions."/>,
    Grippers: ({primaryColor}) => <TipText color={primaryColor} previewText='Grippers' extraText="Grippers are instruments at the end of a robot arm capable of grasping and moving tools and things."/>,
    Fixtures: ({primaryColor}) => <TipText color={primaryColor} previewText='Fixtures' extraText="Fixtures are objects in the environment that aren’t interacted with directly."/>,
    Inputs: ({primaryColor}) => <TipText color={primaryColor} previewText='Inputs' extraText=" Inputs and Outputs specify things that can be used as components of a process."/>,
    Outputs: ({primaryColor}) => <TipText color={primaryColor} previewText='Outputs' extraText=" Inputs and Outputs specify things that can be used as components of a process."/>
}