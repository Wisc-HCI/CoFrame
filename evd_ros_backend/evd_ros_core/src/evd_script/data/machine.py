from ..node import Node
from ..environment.collision_mesh import CollisionMesh

#TODO provide a thing input region
#TODO provide a thing output region
#TODO handle thing conversion (thing consumers and thing generators)

class Machine(Node):

    '''
    Data structure methods
    '''

    def __init__(self, collision_mesh=None, type='', name='', uuid=None, parent=None, append_type=True):
        self._collision_mesh = None

        super(Machine,self).__init__(
            type='machine.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.collision_mesh = collision_mesh

    def to_dct(self):
        pass

    @classmethod
    def from_dct(cls, dct):
        pass

    '''
    Data accessor/modifier methods
    '''

    @property
    def collision_mesh(self):
        return self._collision_mesh

    @collision_mesh.setter
    def collision_mesh(self, value):
        if self._collision_mesh != value:
            if self._collision_mesh != None:
                self._collision_mesh.remove_from_cache()

            self._collision_mesh = value

            if self._collision_mesh != None:
                self._collision_mesh.add_to_cache()
            self.updated_attribute('collision_mesh','set')


    def set(self, dct):

        if 'collision_mesh' in dct.keys():
            self.collision_mesh = CollisionMesh.from_dct(dct['collision_mesh'])

        super(Machine,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):

        if self.collision_mesh != None:
            self.collision_mesh.remove_from_cache()

        super(Machine,self).remove_from_cache()

    def add_to_cache(self):

        if self.collision_mesh != None:
            self.collision_mesh.add_to_cache()

        super(Machine,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Machine,self).deep_update()

        self.updated_attribute('collision_mesh','update')

    def shallow_update(self):
        super(Machine,self).shallow_update()

        self.updated_attribute('collision_mesh','update')
