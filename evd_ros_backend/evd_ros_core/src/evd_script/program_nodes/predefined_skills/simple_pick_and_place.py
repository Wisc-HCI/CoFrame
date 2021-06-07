'''
Pick-and-place is a very common activity for collaborative robots.

This is a simplified (non-conditional/error-handling) implementation that
commands an end-effector to move to a target object, grasp it, move to final
location, and then release it.
'''

from ...data_nodes.skill_arguement import SkillArguement
from ..skill import Skill
from ..primitives import MoveTrajectory
from ..hierarchical_tasks.open_gripper import OpenGripper
from ..hierarchical_tasks.close_gripper import CloseGripper


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

    def __init__(self, primitives=None, arguements=None, parameters=None, type='', 
                 name='', uuid=None, parent=None, append_type=True, 
                 editable=False, deleteable=False, description=''):

        pick_traj_arg = None
        place_traj_arg = None
        thing_arg = None

        if arguements != None:
            for a in arguements:
                if a.parameter_key == 'trajectory_uuid':
                    if a.name == 'pick_trajectory':
                        pick_traj_arg = a
                    elif a.name == 'place_trajectory':
                        place_traj_arg = a
                elif a.parameter_key == 'thing_uuid':
                    thing_arg = a

            if pick_traj_arg == None:
                pick_traj_arg = SkillArguement(parameter_key='trajectory_uuid', name='pick_trajectory'),
                arguements.append(pick_traj_arg)
            if place_traj_arg == None:
                place_traj_arg = SkillArguement(parameter_key='trajectory_uuid', name='place_trajectory'),
                arguements.append(place_traj_arg)
            if thing_arg == None:
                thing_arg = SkillArguement(parameter_key='thing_uuid',name='thing_uuid')
                arguements.append(thing_arg)

        else:
            pick_traj_arg = SkillArguement(parameter_key='trajectory_uuid', name='pick_trajectory'),
            place_traj_arg = SkillArguement(parameter_key='trajectory_uuid', name='place_trajectory'),
            thing_arg = SkillArguement(parameter_key='thing_uuid',name='thing_uuid')
            arguements = [pick_traj_arg,place_traj_arg,thing_arg]

        if primitives == None:
            primitives = [
                MoveTrajectory(trajectory_uuid=pick_traj_arg.temporary_value, editable=editable, deleteable=False),
                CloseGripper(thing_uuid=thing_arg.temporary_value, editable=editable, deleteable=False),
                MoveTrajectory(trajectory_uuid=place_traj_arg.temporary_value, editable=editable, deleteable=False),
                OpenGripper(thing_uuid=thing_arg.temporary_value, editable=editable, deleteable=False)
            ]
        else: #make sure each has a fresh copy (only applicable since we predefined the primitives)
            primitives[0].trajectory_uuid = pick_traj_arg.temporary_value
            primitives[1].thing_uuid = thing_arg.temporary_value
            primitives[2].trajectory_uuid = place_traj_arg.temporary_value
            primitives[3].thing_uuid = thing_arg.temporary_value

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
            arguements=arguements,
            parameters=parameters)