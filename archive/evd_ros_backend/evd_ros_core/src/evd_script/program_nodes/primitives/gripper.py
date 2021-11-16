'''
Gripper exposes the robot's grasping behavior in a generalized fashion.
Position, speed, and effort paramters ought to be supplied. Internal kinematics
of the gripper is handled at the implementation level.

#TODO implement real-time behavior
'''

from ..primitive import Primitive
from ...data_nodes import Thing
from ...data_nodes.regions import SphereRegion
from ...data_nodes.geometry import Pose, Position, Orientation
from ...type_defs import NUMBER_TYPE, ENUM_TYPE


class Gripper(Primitive):

    '''
    Constants
    '''

    SEMANTIC_AMBIGUOUS = 'ambiguous'
    SEMANTIC_GRASPING = 'grasping'
    SEMANTIC_RELEASING = 'releasing'

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Gripper'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'gripper' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['parameters'].append({
            'type': NUMBER_TYPE,
            'key': 'position',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': NUMBER_TYPE,
            'key': 'effort',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': NUMBER_TYPE,
            'key': 'speed',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': ENUM_TYPE,
            'key': 'semantic',
            'is_uuid': False,
            'is_list': False,
            'enum_values': [
                cls.SEMANTIC_AMBIGUOUS,
                cls.SEMANTIC_GRASPING,
                cls.SEMANTIC_RELEASING
            ]
        })
        template['parameters'].append({
            'type': Thing.full_type_string(),
            'key': 'thing_uuid',
            'is_uuid': True,
            'is_list': False
        })
        return template

    def __init__(self, position=0, effort=0, speed=0, thing_uuid=None, semantic=None,
                 parameters=None, type='', name='', uuid=None, parent=None, 
                 append_type=True, editable=True, deleteable=True, description=''):

        if parameters == None:
            parameters = {
                'thing_uuid': thing_uuid,
                'position': position,
                'effort': effort,
                'speed': speed,
                'semantic': semantic if semantic != None else self.SEMANTIC_GRASPING
            }

        super(Gripper,self).__init__(
            type=Gripper.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def position(self):
        return self._parameters['position']

    @position.setter
    def position(self, value):
        if self._parameters['position'] != value:
            self._parameters['position'] = value
            self.updated_attribute('parameters.position','set')

    @property
    def effort(self):
        return self._parameters['effort']

    @effort.setter
    def effort(self, value):
        if self._parameters['effort'] != value:
            self._parameters['effort'] = value
            self.updated_attribute('parameters.effort','set')

    @property
    def speed(self):
        return self._parameters['speed']

    @speed.setter
    def speed(self, value):
        if self._parameters['speed'] != value:
            self._parameters['speed'] = value
            self.updated_attribute('parameters.speed','set')

    @property
    def thing_uuid(self):
        return self._parameters['thing_uuid']

    @thing_uuid.setter
    def thing_uuid(self, value):
        if self._parameters['thing_uuid'] != value:
            self._parameters['thing_uuid'] = value

            if self._parameters['thing_uuid'] == None:
                # without a reference to a thing, behavior is unknowable
                self.semantic = self.SEMANTIC_AMBIGUOUS
            elif self.semantic == self.SEMANTIC_AMBIGUOUS:
                # assume attempting to grasp by default
                self.semantic = self.SEMANTIC_GRASPING

            self.updated_attribute('parameters.thing_uuid','set')

    @property
    def semantic(self):
        return self._parameters['semantic']

    @semantic.setter
    def semantic(self, value):
        if self._parameters['semantic'] != value:

            if self.thing_uuid == None:
                if value != self.SEMANTIC_AMBIGUOUS:
                    raise Exception('If no thing is defined then semantic can only be ambiguous')
            else:
                if value != self.SEMANTIC_GRASPING and value != self.SEMANTIC_RELEASING:
                    raise Exception('If thing defined then it must either be grasping or releasing')

            self._parameters['semantic'] = value
            self.updated_attribute('parameters.semantic','set')

    def set(self, dct):
        position = dct.get('position', None)
        if position != None:
            self.position = position

        effort = dct.get('effort', None)
        if effort != None:
            self.effort = effort

        speed = dct.get('speed', None)
        if speed != None:
            self.speed = speed

        if 'thing_uuid' in dct.keys():
            self.thing_uuid = dct['thing_uuid']

        if 'semantic' in dct.keys():
            self.semantic = dct['semantic']

        super(Gripper,self).set(dct)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        hooks.tokens['robot']['state']['gripper']['position'] = self.position

        if self.thing_uuid != None:
            within = self._check_if_within_grasp_region(hooks)
            if not within:
                raise Exception('thing {} not within gripper region'.format(self.thing_uuid))

            # set grasping semantic
            if self.semantic == self.SEMANTIC_GRASPING:
                hooks.tokens['robot']['state']['gripper']['grasped_thing'] = self.thing_uuid
            elif self.semantic == self.SEMANTIC_RELEASING:
                hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
        else:
            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
            hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = True

        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.state[self.uuid] = 'pending'
            hooks.robot_interface.grip_async(self.position, self.speed, self.effort)

        else:
            resp = hooks.robot_interface.is_acked('gripper')
            if resp != None:
                if resp:
                    next = self.parent

                    if self.thing_uuid != None:
                        within = self._check_if_within_grasp_region(hooks)
                        if not within:
                            raise Exception('thing {} not within gripper region'.format(self.thing_uuid))

                        # set grasping semantic
                        if self.semantic == self.SEMANTIC_GRASPING:
                            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = self.thing_uuid
                        elif self.semantic == self.SEMANTIC_RELEASING:
                            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
                    else:
                        hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
                        hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = True

                else:
                    raise Exception('Robot NACKed')

        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['gripper']['position'] = status.gripper_position

        if next == self.parent:
            del hooks.state[self.uuid]
        return next

    def _check_if_within_grasp_region(self, hooks):
        # check if thing is actually near the gripper
        gripperRegion = SphereRegion(
            center_position=Position(0,0.1,0), 
            link='ee_link',
            uncertainty_radius=0.1, 
            free_orientation=True)

        _, thing_pose = Pose.compute_relative(
                Pose.from_simple_dct(hooks.tokens[self.thing_uuid]['state']),
                Pose.from_simple_dct(hooks.tokens['robot']['state']))

        return gripperRegion.check_if_pose_within_uncertainty(thing_pose)
