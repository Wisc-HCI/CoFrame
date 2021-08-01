
import time
import pybullet

from sensor_msgs.msg import JointState


class PyBulletModel(object):
    
    def __init__(self, urdf_path, config, gui=True):
        self.timeStep = config['timestep']

        #pybullet setup
        physicsClient = pybullet.connect(pybullet.GUI if gui else pybullet.DIRECT)
        pybullet.setAdditionalSearchPath(urdf_path)
        pybullet.setRealTimeSimulation(False)
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(self.timeStep)
        pybullet.setRealTimeSimulation(False)

        #flags = pybullet.URDF_USE_SELF_COLLISION|pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS 
        flags = pybullet.URDF_MERGE_FIXED_LINKS

        self.robotId = pybullet.loadURDF(config['urdf'], [0,0,0], useFixedBase=True, flags=flags)
        self.jointIds = {}
        for j in range(pybullet.getNumJoints(self.robotId)):
            info = pybullet.getJointInfo(self.robotId, j)
            jointName = info[1].decode("utf-8") 
            self.jointIds[jointName] = j     

        print('\n\n\n\n',self.jointIds,'\n\n\n\n')   

    def step(self, joints, names):

        # Set each joints in pybullet
        for name, id in self.jointIds.items():
            index = -1
            for i, n in enumerate(names):
                if n == name:
                    index = i
                    break

            if index != -1:
                pybullet.setJointMotorControl2(
                    self.robotId,
                    jointIndex=id,
                    controlMode=pybullet.POSITION_CONTROL, 
                    targetPosition=joints[index],
                    maxVelocity=1000,
                    force=5 * 240.)

        # Run simulation
        pybullet.stepSimulation()

        # Read current joint state
        msg = JointState()
        return msg

    def cleanup(self):
        pybullet.disconnect()