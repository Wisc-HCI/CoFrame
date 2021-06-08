'''
Machine Blocking Process simplifies the generalzied machining process for a robot. The
machining could be a pre-written CNC routine that gets invoked by this primitive, then
waits until that routine finishes, and will perform any "cleanup" at end of the process.
'''

from ...data_nodes.skill_arguement import SkillArguement
from ..skill import Skill
from ..primitives.machine_start import MachineStart
from ..primitives.machine_stop import MachineStop
from ..primitives.machine_wait import MachineWait


class MachineBlockingProcess(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine Blocking Process Skill'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-blocking-process' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, primitives=None, arguements=None, parameters=None, type='', 
                 name='', uuid=None, parent=None, append_type=True, 
                 editable=False, deleteable=False, description=''):

        machine_uuid_arg = None
        if arguements != None:
            for a in arguements:
                if a.parameter_key == 'machine_uuid':
                    machine_uuid_arg = a
                    break

            if machine_uuid_arg == None:
                machine_uuid_arg = SkillArguement(parameter_key='machine_uuid', name='machine_uuid')
                arguements.append(machine_uuid_arg)
        else:
            machine_uuid_arg = SkillArguement(parameter_key='machine_uuid', name='machine_uuid')
            arguements = [machine_uuid_arg]
            
        if primitives == None:
            primitives=[
                MachineStart(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False),
                MachineWait(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False),
                MachineStop(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False)
            ]
        else: # make sure each has a fresh copy (only applicable since we predefined the primitives)
            for p in primitives:
                p.machine_uuid = machine_uuid_arg.temporary_value

        super(MachineBlockingProcess,self).__init__(
            type=MachineBlockingProcess.type_string() + type if append_type else type,
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
