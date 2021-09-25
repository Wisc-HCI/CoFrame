import time
import pybullet

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


MAX_DISTANCE_MEASURED = 100

 
class PyBulletModel(object):
    
    def __init__(self, urdf_path, config, gui=True):
        self.timeStep = config['timestep']

        # pybullet setup
        physicsClient = pybullet.connect(pybullet.GUI if gui else pybullet.DIRECT)
        pybullet.setAdditionalSearchPath(urdf_path)
        pybullet.setRealTimeSimulation(False)
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(self.timeStep)

        # Load robot
        self.robotId = pybullet.loadURDF(config['urdf']['robot'], [0,0,0], useFixedBase=True)
        self.robot_jointIds = {}
        self.robot_linkIds = {}
        for j in range(pybullet.getNumJoints(self.robotId)):
            info = pybullet.getJointInfo(self.robotId, j)
            
            jointName = info[1].decode("utf-8") 
            linkName = info[12].decode("utf-8")
            self.robot_jointIds[jointName] = j 
            self.robot_linkIds[linkName] = j   

        # Load environment
        self.envId = pybullet.loadURDF(config['urdf']['environment'], [0,0,0], useFixedBase=True)
        self.env_jointIds = {}
        self.env_linkIds = {}
        for j in range(pybullet.getNumJoints(self.envId)):
            info = pybullet.getJointInfo(self.envId, j)

            jointName = info[1].decode("utf-8")
            linkName = info[12].decode("utf-8")
            self.env_jointIds[jointName] = j
            self.env_linkIds[linkName] = j

        # Collisions, Occupancy
        self._collisions = {}
        self._occupancy = {}

    def set_joints(self, joints, names):
        for name, id in self.robot_jointIds.items():
            index = -1
            for i, n in enumerate(names):
                if n == name:
                    index = i
                    break

            if index != -1:
                pybullet.resetJointState(
                    self.robotId,
                    jointIndex=id,
                    targetValue=joints[index],
                    targetVelocity=0)

    def step(self, joints, names, physics=False):

        # Set each joints in pybullet
        for name, id in self.robot_jointIds.items():
            index = -1
            for i, n in enumerate(names):
                if n == name:
                    index = i
                    break

            if index != -1:
                if physics:
                    pybullet.setJointMotorControl2(
                        self.robotId,
                        jointIndex=id,
                        controlMode=pybullet.POSITION_CONTROL, 
                        targetPosition=joints[index],
                        maxVelocity=1000,
                        force=5 * 240.)
                else:
                    pybullet.resetJointState(
                        self.robotId,
                        jointIndex=id,
                        targetValue=joints[index],
                        targetVelocity=0)

        # Run simulation
        pybullet.stepSimulation()
  
        # (jointPositions, jointVelocities, names), (frames, names)
        return self.readJointState(names), self.readFrames()

    def readJointState(self, names=None):
        if names == None:
            names = self.robot_jointIds.keys() # gets all

        jointPositions = [0] * len(names)
        jointVelocities = [0] * len(names)
        for name, id in self.robot_jointIds.items():
            index = -1
            for i, n in enumerate(names):
                if n == name:
                    index = i
                    break

            if index != -1:
                pos, vel, _, _ = pybullet.getJointState(
                    self.robotId,
                    jointIndex=id)

                jointPositions[index] = pos
                jointVelocities[index] = vel

        return (jointPositions, jointVelocities, names)

    def readFrames(self):

        frames = []
        names = []
        for name, id in self.robot_linkIds.items():
            info = pybullet.getLinkState(
                self.robotId,
                linkIndex=id)

            pos = info[2]
            rot = info[3]

            names.append(name)
            frames.append((pos, rot))

        return (frames, names)

    @property
    def joint_names(self):
        return self.robot_jointIds.keys()

    @property
    def frame_names(self):
        return self.robot_linkIds.keys()

    @classmethod
    def get_ee_pose(cls, frames, ee_frame='tool0'):
        pose = Pose()

        for i in range(0,len(frames)):
            if frames[i][1] == ee_frame:
                (pos, rot) = frames[i][0]

                pose.position.x = pos.x
                pose.position.y = pos.y
                pose.position.z = pos.z

                pose.orientation.x = rot.x
                pose.orientation.y = rot.y
                pose.orientation.z = rot.z
                pose.orientation.w = rot.w

                break

        return pose

    def cleanup(self):
        pybullet.disconnect()

    def registerCollisionMeshes(self, collisionMeshes):
        self._collisions = {}
        for c in collisionMeshes:
            if c.link in self.robot_linkIds.keys():
                bodyId = self.robotId
                linkId = self.robot_linkIds[c.link]
            elif c.link in self.env_linkIds.keys():
                bodyId = self.envId
                linkId = self.env_linkIds[c.link]
            else:
                break # Skipping this mesh

            # We need to know both the body to search under and the link for distance computation
            self._collisions[c.uuid] = (bodyId, linkId)

    def collisionCheck(self):
        data = {}

        for uuid, (c_body, c_link) in self._collisions.items():
            data[uuid] = {n: [] for n in self.robot_linkIds.keys()}

            # This is a stupid / inefficient way to do this but I just want to get some data out
            points = pybullet.getClosestPoints(c_body, self.robotId, MAX_DISTANCE_MEASURED)

            for frameName, r_link in self.robot_linkIds.items():
                for point in points:
                    # if bodies and links match
                    if point[1] == c_body and point[2] == self.robotId and point[3] == c_link and point[4] == r_link: 
                        data[uuid][frameName].append({
                            'postion_a': point[5],
                            'position_b': point[6],
                            'distance': point[8]
                        })

        return data
        
    @property
    def collision_uuids(self):
        return self._collisions.keys()

    def registerOccupancyZones(self, occupancyZones):
        self._occupancy = {}
        for o in occupancyZones:
            cubeId = pybullet.createCollisionShape(shapeType=pybullet.GEOM_BOX, halfExtents=[o.scale_x/2,o.scale_z/2,1])
            pybullet.createMultiBody(baseMass=0, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=cubeId, basePosition=[o.position_x,o.position_z,0], useMaximalCoordinates=True)

            # We need to know the body to search under and -1 means the base collision objet (since we created it that way)
            self._occupancy[o.uuid] = (cubeId, -1)

    def occupancyCheck(self):
        data = {}

        for uuid, (o_body, o_link) in self._occupancy.items():
            data[uuid] = {n: [] for n in self.robot_linkIds.keys()}

            # This again is very stupid (less so than before but still)
            points = pybullet.getClosestPoints(o_body, self.robotId, MAX_DISTANCE_MEASURED)

            for frameName, r_link in self.robot_linkIds.items():
                for point in points:
                    # if bodies and links match
                    if point[1] == o_body and point[2] == self.robotId and point[3] == o_link and point[4] == r_link:
                        data[uuid][frameName].append({
                            'position_a': point[5],
                            'position_b': point[6],
                            'distance': point[8]
                        })

        return data

    @property
    def occupancy_uuids(self):
        return self._occupancy.keys()

    def selfCollisionCheck(self):
        data = {}

        # and here for the monumentally stupid loop. But really don't care about the duplicates here right now
        for a_frameName, a_r_link in self.robot_linkIds.items():
            data[a_frameName] = {}

            # Ow
            points = pybullet.getClosestPoints(self.robotId, self.robotId, MAX_DISTANCE_MEASURED)

            for b_frameName, b_r_link in self.robot_linkIds.items():
                data[a_frameName][b_frameName] = []

                for point in points:
                    # if bodies and links match
                    if point[1] == self.robotId and point[2] == self.robotId and point[3] == a_r_link and point[4] == b_r_link:
                        data[a_frameName][b_frameName].append({
                            'position_a': point[5],
                            'position_b': point[6],
                            'distance': point[8]
                        })

        return data