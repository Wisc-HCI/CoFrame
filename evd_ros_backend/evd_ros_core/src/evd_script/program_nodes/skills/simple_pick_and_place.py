'''
Pick-and-place is a very common activity for collaborative robots.

This is a simplified (non-conditional/error-handling) implementation that
commands an end-effector to move to a target object, grasp it, move to final
location, and then release it.
'''

from ...data_nodes.skill_argument import SkillArgument
from ..skill import Skill
from ..primitives import MoveTrajectory, Gripper


def SimplePickAndPlace(name=None, uuid=None, parent=None, editable=True, deleteable=False, description=''):

    pick_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='pick_trajectory')
    place_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='place_trajectory')
    thing_arg = SkillArgument(parameter_key='thing_uuid',name='thing_uuid')
    arguments = [pick_traj_arg,place_traj_arg,thing_arg]

    primitives = [
        MoveTrajectory(
            trajectory_uuid=pick_traj_arg.temporary_value, 
            editable=editable, 
            deleteable=False),
        Gripper(
            thing_uuid=thing_arg.temporary_value,
            semantic=Gripper.SEMANTIC_GRASPING,
            position=100,
            editable=editable,
            deleteable=False),
        MoveTrajectory(
            trajectory_uuid=place_traj_arg.temporary_value, 
            editable=editable, 
            deleteable=False),
        Gripper(
            thing_uuid=thing_arg.temporary_value,
            semantic=Gripper.SEMANTIC_RELEASING,
            position=0,
            editable=editable,
            deleteable=False)
    ]

    return Skill(
        name= name if name != None else 'Simple Pick And Place',
        uuid=uuid,
        parent=parent,
        editable=editable,
        deleteable=deleteable,
        description=description,
        arguments=arguments,
        primitives=primitives)