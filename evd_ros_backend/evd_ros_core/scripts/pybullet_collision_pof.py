#!/usr/bin/env python3

import time
import pybullet


def drawAABB(aabb):
    aabbMin = aabb[0]
    aabbMax = aabb[1]

    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMin[2]]
    pybullet.addUserDebugLine(f, t, [1, 0, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    pybullet.addUserDebugLine(f, t, [0, 1, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMin[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [0, 0, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    pybullet.addUserDebugLine(f, t, [1, 1, 1])


gui = True
timeStep = 0.0165

physicsClient = pybullet.connect(pybullet.GUI if gui else pybullet.DIRECT)
pybullet.setRealTimeSimulation(False)
pybullet.setGravity(0, 0, 0)
pybullet.setTimeStep(timeStep)
pybullet.setRealTimeSimulation(1)

'''
Okay so I need to get creative in order to read distance between objects instead of true collision

It seems that I should first create some simple test objects. My goal is to prove out
scaling. 
'''

print('\n\n\n')


sphereID_1 = pybullet.createCollisionShape(shapeType=pybullet.GEOM_SPHERE, radius=0.5)
cubeID_1 = pybullet.createCollisionShape(shapeType=pybullet.GEOM_BOX, halfExtents=[0.5,0.5,0.5],)

print('SphereID', sphereID_1, 'CubeID', cubeID_1)

sphere = pybullet.createMultiBody(baseMass=0, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=sphereID_1, basePosition=[0.9,0.3,0], useMaximalCoordinates=True)
cube = pybullet.createMultiBody(baseMass=0, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=cubeID_1, basePosition=[0,0,1.25], useMaximalCoordinates=True)

print('sphere',sphere,'cube',cube)




print('------')

#retVal = pybullet.getCollisionShapeData(sphereID_1,-1)
#print(retVal)
#(objId, linkId, geoType, dims, fileName, position, orientation) = retVal[0]

# print('sphere')
# print('objId',objId,'linkId',linkId)
# print('gepType',geoType)
# print('dims',dims)
# print('filename',fileName)
# print('position',position,'orientation',orientation)
# print('color',color)

aabb = pybullet.getAABB(sphereID_1)
print('aabb',aabb)
drawAABB(aabb)

print('\n\n')

#retVal = pybullet.getCollisionShapeData()
#print(retVal)
# (objId, linkId, geoType, dims, fileName, position, orientation) = retVal[0]

# print('cube')
# print('objId',objId,'linkId',linkId)
# print('gepType',geoType)
# print('dims',dims)
# print('filename',fileName)
# print('position',position,'orientation',orientation)
# print('color',color)

aabb = pybullet.getAABB(cubeID_1)
print('aabb',aabb)
drawAABB(aabb)

print('\n\n')

points = pybullet.getClosestPoints(sphere, cube, 10000)
print(points)
for p in points:
    (_, aID, bID, linkAID, linkBID, posA, posB, _, contactDistance, _, _, _, _, _) = p
    print('Body A', aID, 'Body B', bID)
    print('Link A', linkAID, 'Link B', linkBID)
    print('Pos A', posA, 'Pos B', posB)
    print('contactDistance', contactDistance)

    pybullet.addUserDebugLine(posA, posB, [0.5, 0, 1])

while True:
    time.sleep(1./240.)

pybullet.disconnect()

'''
Okay so I can use aabb to get the bounding box

I still need to confirm that I can get collision detection without it interferring with movement
'''