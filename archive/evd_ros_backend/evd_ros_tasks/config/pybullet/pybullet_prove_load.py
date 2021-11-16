#!/usr/bin/env python3
import os
import time
import pybullet as pb

urdf_path = os.getcwd()
print('URDF PATH', urdf_path)

physicsClient = pb.connect(pb.GUI)
#pb.setAdditionalSearchPath(urdf_path)
pb.setRealTimeSimulation(False)
pb.setGravity(0, 0, 0)
pb.setTimeStep(1/60)
pb.setRealTimeSimulation(1)

# Core
print('\n\n')
envId = pb.loadURDF('environment.urdf', [0,0,0], useFixedBase=True)
print('\n\n')
robotId = pb.loadURDF('robot.urdf', [0,0,0], useFixedBase=True)

# Collision URDFS
pd = pb
print('\n\n')
cm_3d_printerId = pb.loadURDF('collision_mesh_urdfs/3d_printer.stl.urdf', [0,3,0], useFixedBase=True)
print('\n\n')
cm_assembly_jigId = pb.loadURDF('collision_mesh_urdfs/assembly_jig.stl.urdf', [0,4,0], useFixedBase=True)
print('\n\n')
cm_blade_feederId = pd.loadURDF('collision_mesh_urdfs/blade_feeder.stl.urdf', [1,3,0], useFixedBase=True)
print('\n\n')
cm_boxId = pd.loadURDF('collision_mesh_urdfs/box.stl.urdf',[1,4,0],useFixedBase=True)
print('\n\n')
cm_conveyorId = pd.loadURDF('collision_mesh_urdfs/conveyor.stl.urdf',[-2,3,0],useFixedBase=True)
print('\n\n')
cm_knife_feederId = pd.loadURDF('collision_mesh_urdfs/knife_feeder.stl.urdf',[2,4,0], useFixedBase=True)
print('\n\n')
cm_pedestalId = pd.loadURDF('collision_mesh_urdfs/pedestal.stl.urdf',[3,3,0],useFixedBase=True)
print('\n\n')
cm_tableId = pd.loadURDF('collision_mesh_urdfs/table.stl.urdf',[3,-4,0],useFixedBase=True)
print('\n\n')

while True:
    time.sleep(1)

pb.disconnect()