# evd_ros_tasks / 3d_printer_machine_tending
The following documentation pertains to the 3D printer machine tending task.

An often performed task for cobots is to tend to CNC devices. The robot can
insert / updating tooling, insert stock monitor process, and/or extract product
from the machine. In this specific task, the cobot is responsible for monitoring
a single 3D printer. The task is simple, as the 3D printer merely needs to print
a single item then the robot must grasp it and place it into a box.

# Notes
Currently tasks are hand constructed at this point. This is acceptable as we
don't expect operators to craft entirely new use cases for their cobot but to
extend existing tasks that have been scaffolded by engineers. Modify the task
with caution, as at this time there are no validity checks.
