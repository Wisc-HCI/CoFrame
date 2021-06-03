# Expert View Dashboard - Script (EvDScript)

EvD_Script is an imperative domain-specific robot programming language used within EvD to model simple robot arm applications. The design of the language is built around an AST-like structure. Each node (Node.py and its derivatives) has a well-specified set of hooks for program construction, change feedback propagation, serialization/deserialization, and execution.

Refer to the following diagram, which breaks down EvD_Script into the inheritance hierarchy.

<img src="https://github.com/Wisc-HCI/Expert_View_Dashboard/evd_ros_backend/evd_ros_core/_docs/_imgs/evd_script_hierarchy.png" width="200" />

## The Program and The Context
Broadly, the language is broken down into first-class data types (*data*), first-class safety objects (*environment*), and program operation (*program*). 

As mentioned the program is an AST of nodes. At the root is a *Program Node*. This node contains *Environment* (a special kind of *Context*) and a list of executable nodes. A context is a storage / container node for all the referenced data in the program. This includes data like *Locations*, *Machines*, and *Trajectories*. An environment is a context that also stores safety objects like *Pinch-Points* and *Collision-Meshes*. In EvD, there is only one environment and it also is the only context. This is because information stored is merely a representation of the real-world and it wouldn't make sense for it to be suddenly out of scope.

### Data
All data is stored in the root context. Data either is concerned with representing spatial information (Location, Waypoint, Region(s), Pose, Position, Orientation), defining objects (Thing, ThingType, Machine, MachineRecipe), or capturing robot movement (Trajectory, Trace, TraceDataPoint, Grade). 

### Environment
Environment objects allow for expert reasoning about the workspace. For instance, a human OccupancyZone can represent where the human will likely stand during the task. Each object is a EnvironmentNode type and these nodes are stored in the Envrionment (Context).

### Program
Programs are composed of Skills, Primitives, and Control_Flow operations. A skill is a collection of other skills or primitives that have a particular order of execution. In fact, a program is a special kind of skill. Primitives are basic actions such as MoveTrajectory. A special subcategory of actions are MachineOperations, which control other devices communicating with the robot. Lastly, control flow provides looping and breakpoint behavior to programs. At this time, branching and conditionals are stubbed but not supported.

## The Cache
Since the program is an AST it has many node branchs of arbitrary length. This can be problematic when searching for a node with a particular UUID (how nodes are uniquely identified). Thus, EvD_Script also maintains a flattened structure called the cache. Its job is to store all nodes as a lookup by UUID. Furthermore,
the cache keeps track of the node's type such that one can hint to the type when trying to search for the node.

## Changes and Tracebacks
When a node in the program is changed it invokes an internal change update callback on its parent. This behavior is expected on any public property modification unless otherwise documented. On the callback, each node will append its information to a trace and then invoke its parent's callback function until
the root node (parent == None) is found.

If the root node is a program then it will call a user defined trace callback function provided on object construction (or supplied after). Generally, users shouldn't write this callback function themselves. Instead they should use the AttributeTraceProcessor which affords subscription based listening on traces for specific object modification. 

When a node is changed it may also notify the *Orphans* module. This module is responsible for repairing state (missing objects / references) after a program is modified. This module needs to be run external to the evd change update loop as it is a very expensive process.

## Serialization / Deserialization
EvDScript does not handle the low-level format (e.g., JSON or Binary) but it does generate an intermediate python dictionary level data structure. This object follows the same tree structure as EvD_Script composed of nodes and provides the same parameterization. The main difference is all of the "nice" conversion methods, checking, callbacks, etc. are missing.

The serialization process is intended to be pretty simple. Each node walks its chain of inheritance and appends fields to a dictionary. When a node has a child node it must call the serialization on that node as well.

Deserialization is a bit trickier. Nodes *MUST* be deserialized with the NodeParser. This is due to the NodeParser first checking for existing node objects in the cache with the same UUID. If it finds an object already exists it merely updates the values. This helps with preserving object reference links. If a node is not in the cache then it is freshly constructed. 

After derserialization is complete, the user must call `late_construct_update()` on the root node. Due to how the Context is deserialized, certain objects do not yet have all the proper linking. Notably, *MoveTrajectory* maintains a local context patch that needs to be applied after deserialization.

## RVIZ
The python variant of EvDScript has limited support for RVIZ object visualization.

Nodes that provide methods to generate markers inherit from interfaces defined in the *Visualizable* module.