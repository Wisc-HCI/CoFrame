'''
Pick-and-place is a very common activity for collaborative robots.

This is a simplified (non-conditional/error-handling) implementation that
commands an end-effector to move to a target object, grasp it, move to final
location, and then release it.
'''

from ...data_nodes.skill_argument import SkillArgument
from ..skill import Skill
from ..primitives import MoveTrajectory, Gripper


class SimplePickAndPlace(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Simple Pick and Place Skill'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'simple-pick-and-place' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, primitives=None, arguments=None, parameters=None, type='', 
                 name=None, uuid=None, parent=None, append_type=True, 
                 editable=False, deleteable=False, description=''):

        pick_traj_arg = None
        place_traj_arg = None
        thing_arg = None

        if arguments != None:
            for a in arguments:
                if a.parameter_key == 'trajectory_uuid':
                    if a.name == 'pick_trajectory':
                        pick_traj_arg = a
                    elif a.name == 'place_trajectory':
                        place_traj_arg = a
                elif a.parameter_key == 'thing_uuid':
                    thing_arg = a

            if pick_traj_arg == None:
                pick_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='pick_trajectory')
                arguments.append(pick_traj_arg)
            if place_traj_arg == None:
                place_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='place_trajectory')
                arguments.append(place_traj_arg)
            if thing_arg == None:
                thing_arg = SkillArgument(parameter_key='thing_uuid',name='thing_uuid')
                arguments.append(thing_arg)

        else:
            pick_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='pick_trajectory')
            place_traj_arg = SkillArgument(parameter_key='trajectory_uuid', name='place_trajectory')
            thing_arg = SkillArgument(parameter_key='thing_uuid',name='thing_uuid')
            arguments = [pick_traj_arg,place_traj_arg,thing_arg]

        if primitives == None:
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
        else: #make sure each has a fresh copy (only applicable since we predefined the primitives)
            primitives[0].trajectory_uuid = pick_traj_arg.temporary_value
            primitives[1].thing_uuid = thing_arg.temporary_value
            primitives[2].trajectory_uuid = place_traj_arg.temporary_value
            primitives[3].thing_uuid = thing_arg.temporary_value

        if name == None:
            name = 'Simple Pick and Place'

        super(SimplePickAndPlace,self).__init__(
            type=SimplePickAndPlace.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            primitives=primitives,
            arguments=arguments,
            parameters=parameters)