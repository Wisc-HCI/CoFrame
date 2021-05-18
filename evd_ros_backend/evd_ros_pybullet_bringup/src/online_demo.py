#! /usr/bin/env python3

import os
import time
import pybullet
import pybullet_data

import pathlib
currentDir = pathlib.Path(__file__).parent.absolute()

physicsClient = pybullet.connect(pybullet.GUI) # or pybullet.DIRECT for non-gui
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
pybullet.setGravity(0,0,-9.8)

planeId = pybullet.loadURDF("plane.urdf")

time.sleep(0.5)
path = 'fmauch_universal_robot/ur_e_description/urdf'
print(path)

# So the key realization is that pybullet loads a URDF naively. As such I need to be at the right


robotId = pybullet.loadURDF(path, [0,0,0], useFixedBase=True)
'''
jointIds = []
paramIds = []
time.sleep(0.5)

for j in range(p.getNumJoints(robotId)):
    info = pybullet.getJointInfo(robotId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
'''

pybullet.setRealTimeSimulation(1)

try:
    while(True):
        lastTime = time.time()

        '''
        # Set each joint
        for j in range(len(jointIds)):
            pybullet.setJointMotorControl2(robotId,j,pybullet.POSITION_CONTROL, 0)
        '''
except:
    pass

pybullet.disconnect()