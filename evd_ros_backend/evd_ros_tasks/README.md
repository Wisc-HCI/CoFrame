# evd_ros_tasks
EvD is an education system developed for collaborative manufacturing robots (e.g., Universal Robots, Franka Emika) with the goal of overlaying an expert's view of a task onto the operator's.
We aim to improve understanding of the task such that operators can make informed changes to their program.

This package provides tasks implemented for experiments and demos when using the EvD system.

To see the system overview check out the repository [README](../README.md).

## Tasks
The following tasks were implemented, click the link to get further documentation.

- [3D Printer Machine Tending](./tasks/3d_printer_machine_tending/README.md) ~ Interacts with a fake CNC machine and performs palletization.

## Task Format
All tasks must contain an `applications` directory. This is where serialized evd_script is stored.
Additionally, all `collision_meshes` and unique `models` are required to be stored in within the task
directory. Lastly, the application should have a URDF that specifies the links between the robot and
all fixtures / static objects within the world.

The `applications` directory should always contain a `meta.json` file. When initialized,
it should look like this:

```
{
    "description": "<Brief note about the task>",
    "name": "<Task Name>",
    "options": []
}
```

The options field will be filled in by the data server when applications are saved. Each option is an
application plus configuration that can be run by EvD.

Lastly, make sure to update the launch file `main.launch` with the task option and include a
custom launch file in the `launch\tasks` directory. The custom launch file should hook into
existing EvD ore infrastructure w/ custom configuration. If nodes external to EvD need to run,
then this is where you should launch them.

## Notes
This package ends up serving as a place to store one-off task specific code. As
such there is a fair amount of deprecated and half-baked code here.
