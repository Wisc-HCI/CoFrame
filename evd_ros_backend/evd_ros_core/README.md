# evd_ros_core
Expert View Dashboard (EvD) is an educational environment used to train operators
for collaborative robotic workcells.

This package is the ROS backend server that runs an implemented task. Consider this package
to be a library to hook into when building your specific applications.

To see the system overview check out the repository [README](../../README.md).

## Core Architecture
The goal of the core package is to centralize Evd behavior between multiple tasks.
If one is writing one-off code then it should not go here. Likewise, very low-level
cobot control probably should not be implemented in here.

The curx of Core is the data_server. This node maintains the current EvDscript state
for the entire EvD environment. Access to this data_server is easily achieved with
the data client interface.

EvD also provides a trace processor and its asociated graders to convert trajectories
and waypoint/locations into traces and joint states, respectively.

EvD also tracks issues and pending jobs, whcih can be routed into the frontend
interface.

## Core Interfaces
### Data Client Interface
TODO

###  Issue Client Interface
TODO

### Program Runner
TODO

### Robot Control Interface
TODO

### Robot Interface
TODO

## EvD Script
TODO

## Nodes

### Application TF Handler
This node generates transforms to connect external application into ros frame.

TF Frames:
- application_camera
- application (ros frame)
- control_target

Subscribers:
- application/ros_frame
  - PoseStamped
    - header.frame_id should == 'app'
    - Relative to ROS_FRAME (often base_link or world)
- application/camera_pose
  - PoseStamped
    - header.frame_id should == 'app'
- application/control_target_pose
  - PoseStamped
    - header.frame_id should == 'app'

### Data Server
Handles the evd_script application storage and program implementation logic.

Interfaces:
- *IssueClientInterface*

Publishers:
- data_server/update
  - evd_ros_core/UpdateData
    - Pushes entire program out on changes in the data_server
- data_server/has_changes
  - evd_ros_core/UpdateData
    - Pushes flag information out on changes in data_server but does not push out program (computationally lighter).

Services:
- data_server/load_application_data
  - evd_ros_core/LoadData
    - Loads a program from local file on the backend. Filenames can be retrieved with get_application_options
- data_server/save_application_data
  - evd_ros_core/SaveData
    - Saves a program to local file on the backend. 
- data_server/get_application_options
  - evd_ros_core/GetOptions
    - Gets all programs stored locally on backend. Provides meta-data for use in loading programs. 
- data_server/get_program
  - evd_ros_core/GetData
    - Gets currently loaded program and/or partial programs by UUID. 
- data_server/set_prorgam
  - evd_ros_core/SetData
    - Sets a new program into data_server and/or partial programs by change manifest. 
- data_server/get_history
  - evd_ros_core/GetHistory
    - Gets current serialized history buffer and/or specific entires from server
- data_server/get_uuids
  - evd_ros_core/GetData
    - Gets current list of uuids (either all or filtered by type)     

### Issue Server
Provide pending jobs and issues in the EvD system.

Publishers:
- issue_server/updated_issues
  - Bool
    - Issues have been recently updated (signal for frontend)
- issue_server/updated_pending_jobs
  - Bool
    - Pending jobs have been updated (signal for frontend)

Subscribers:
- issue_server/issue_submit
  - evd_ros_core/Issue
    - Nodes may submit a new issue at any time. This will trigger updated_issues to publish.  

Services:
- issue_server/get_issues
  - evd_ros_core/GetIssues
    - Gets all issues and/or filtered list of issues
- issue_server/clear_issue
  - evd_ros_core/ClearIssue
    - Clears an issue from the server
- issue_server/get_pending_jobs
  - evd_ros_core/GetPendingJobs
    - Gets all pending jobs running in the backend (graders/planners) and/or filtered subset of jobs. 
- issue_server/set_pending_jobs
  - evd_ros_core/SetPendingJobs
    - Adds a list of pending jobs into the server
- issue_server/clear_pending_job
  - evd_ros_core/ClearPendingJob
    - Clears a specific job from the server's list  

### Robot Control Server
High-level control of robot instance subsystems.

Interfaces:
- *RobotInterface*
- *ProgramRunner*
- *MachineInterface*
- *DataClientInterface*

Publishers:
- `robot_control_server/at_start`
  - Bool
    - Program runner at start
- `robot_control_server/at_end`
  - Bool
    - Program runner at end
- `robot_control_server/lockout`
  - Bool
    - Program runner is currently executing
- `robot_control_server/status`
  - evd_ros_core/ProgramRunnerStatus
    - Provides current operation of the program runner
- `robot_control_server/tokens`
  - String
    - Serialized JSON with all active tokens (robot, things, machines) in program.
- `robot_control_server/errors`
  - String
    - Publishes errors encountered during execution of program and controls in human readable form    

Subscribers:
- `robot_control_server/play`
  - Empty
    - Commands program runner to play
- `robot_control_server/stop`
  - Empty
    - Commands program runner to stop
- `robot_control_server/pause`
  - Empty
    - Commands program runner to pause
- `robot_control_server/reset`
  - Empty
    - Commands program runner to reset (to start)

Services:
- `robot_control_server/set_root_node`
  - evd_ros_core/SetRootNode
    - Change the root executable node in the program (send an empty string as default to program).
- `robot_control_server/get_root_node1`
  - evd_ros_core/GetRootNode
    - Get the current root node uuid (or empty string if default program behavior)  

### Trace Processor
Trace processor generates traces from trajectories and joint states from waypoints.

Interfaces:
- *DataClientInterface*
- *IssueClientInterface

Publishers:
- `graders/submit_job`
  - Strong (serialized trace)

### Graders
There are several graders that follow a standard interface. Each grader takes
in a trace and through a set of heuristics attempts to determine a 0 to 1 rating at each point in the trace. Note - zero is "bad" and one is "good" though
it is up to the grader itself to determine what the semantics of that is.

Subscribers:
- graders/submit_job
  - String (serialized trace)

Publishers:
- DataClientInterface
  - Adds grades to traces
- IssueClientInterface
  - Updates pending jobs and errors on failed grade

#### Collision Grader
Performs a collision lookup given the robots joint configuration in the trace.
One means no collision imminent, zero means collision. Impending collisions are in between. One grade per collision object.

#### Occupancy Grader
Performs a quick heuristic check to see if human occupancy zones are entered. Zero if robot enters zone, one if not near. If impending entry then in bewteen.

#### Speed Grader
Grades the speed both for safety (going too fast) and or performance (going too slow).

#### Pinchpoint Grader
Generates a grade for each pinchpoint. One means no pinch point possible. Zero means robot is colliding with itself. In between is degree of potenial pinchpoint.


## Future Work
There are several parts that should be addressed in the future but are not on the
critical path toward EvD.

First, EvD does not handle program verification. Implementing this could improve
/ finish the program quality frame feedback. Of course this also requires changes
to EvDscript.

Second, EvDscript needs to implement full control flow. That is branching and
conditional looping. There should also be an exception mechanism and sensor
mechanisms.

Third, EvD should handle grasp verification. This can improve the authenticity of
grasping simulation over the naive assumptions currently baked into EvD.
