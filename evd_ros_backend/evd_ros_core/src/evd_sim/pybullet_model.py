
import time
import pybullet

#/home/curt/catkin_ws/src/Expert_View_Dashboard/evd_ros_backend/evd_ros_tasks/config/pybullet/planner_ur3e_robotiq85_plus_workcell.urdf


class PyBulletModel(object):
    
    def __init__(self, urdf_path, config, realTime=False, gui=False):
        self._realtime = realTime

        print('\n\n\n',urdf_path,'\n\n\n')

         #pybullet setup
        physicsClient = pybullet.connect(pybullet.GUI if gui else pybullet.DIRECT) # or pybullet.DIRECT for non-gui
        #pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP,1)
        #pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        pybullet.setAdditionalSearchPath(urdf_path)
        pybullet.setGravity(0, 0, -9.8)

        self.timeStep = config['timestep']
        pybullet.setTimeStep(self.timeStep)

        print('\n\n\n',config['urdf'],'\n\n\n')

        self.robotId = pybullet.loadURDF(config['urdf'], [0,0,0], useFixedBase=True)
        self.jointIds = {}
        for j in range(pybullet.getNumJoints(self.robotId)):
            info = pybullet.getJointInfo(self.robotId, j)
            jointName = info[1].decode("utf-8") 
            self.jointIds[jointName] = j

        pybullet.setRealTimeSimulation(self._realtime)

    def step(self):

        lastTime = time.time()
         # Set each joint in pybullet
        for name, id in self.jointIds.items():

            index = -1
            for i, n in enumerate(self.config_data["joint_ordering"]):
                #print(name, 'simulated_' + self.config_data["joint_ordering"][i])
                if name == 'simulated_' + n:
                    index = i

            if index != -1:
                val = joints[i]
                pybullet.setJointMotorControl2(
                    self.robotId,
                    id,
                    pybullet.POSITION_CONTROL, 
                    targetPosition=val)

        pybullet.stepSimulation()
        if self._realtime:
            time.sleep(self.timeStep)