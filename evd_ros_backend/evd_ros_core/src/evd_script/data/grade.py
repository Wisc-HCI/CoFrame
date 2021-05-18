#TODO generalize the grade structure
from ..node import Node


class Grade(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'grade' + '.' if trailing_delim else ''

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, value, grade_type, type='', name='', uuid=None, parent=None, append_type=True):
        self._value = None
        self._grade_type = None

        super(Grade,self).__init__(
            type=Grade.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.value = value
        self.grade_type = grade_type

    def to_dct(self):
        msg = super(Grade,self).to_dct()
        msg.update({
            'grade_type': self.grade_type,
            'value': self.value
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            value=dct['value'],
            grade_type=dct['grade_type'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        if self._value != value:
            self._value = value
            self.updated_attribute('value','set')

    @property
    def grade_type(self):
        return self._grade_type

    @grade_type.setter
    def grade_type(self, value):
        if self._grade_type != value:
            self._grade_type = value
            self.updated_attribute('grade_type','set')

    def set(self, dct):

        if 'value' in dct.keys():
            self.value = dct['value']

        if 'grade_type' in dct.keys():
            self.grade_type = dct['grade_type']

        super(Grade,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Grade,self).deep_update()

        self.updated_attribute('value','update')
        self.updated_attribute('grade_type','update')

    def shallow_update(self):
        super(Grade,self).shallow_update()

        self.updated_attribute('value','update')
        self.updated_attribute('grade_type','update')
