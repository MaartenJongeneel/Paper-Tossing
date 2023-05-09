import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

hysicsClient = p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.resetDebugVisualizerCamera(cameraDistance=11, cameraYaw=35, cameraPitch=-25, cameraTargetPosition=[-11,2,3])

# Environment and object definition
plane1HalfLength = 10
plane1HalfWidth = 10
plane1HalfHeight = 0
sh_colPlane1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[plane1HalfLength, plane1HalfWidth, plane1HalfHeight])
plane1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colPlane1,
                         basePosition=[-11,0,0],baseOrientation=p.getQuaternionFromEuler([0,0,0]))
p.changeVisualShape(plane1, -1, rgbaColor=[66/255, 73/255, 73/255, 1])

plane2HalfLength = 0
plane2HalfWidth = 5
plane2HalfHeight = 2.5
sh_colPlane2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[plane2HalfLength, plane2HalfWidth, plane2HalfHeight])
plane2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colPlane2,
                         basePosition=[-11,0,2.5],baseOrientation=p.getQuaternionFromEuler([0,0,0]))
p.changeVisualShape(plane2, -1, rgbaColor=[64/255, 224/255, 208/255, 1])

plane3HalfLength = 2.5
plane3HalfWidth = 5
plane3HalfHeight = 0
sh_colPlane3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[plane3HalfLength, plane3HalfWidth, plane3HalfHeight])
plane3 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colPlane3,
                         basePosition=[-12.605,0,6.915],baseOrientation=p.getQuaternionFromEuler([0,0.8732,0]))
p.changeVisualShape(plane3, -1, rgbaColor=[64/255, 224/255, 208/255, 1])


startPos = [-12.15849365, -2.1830127, 9.90849365]
startOrientation = [0.4330, 0.4330, -0.2500, 0.7500]
cube = p.loadURDF("cube.urdf",startPos, startOrientation)
p.changeVisualShape(cube, -1, rgbaColor=[255/255, 255/255, 0/255, 1])

# Adjust parameters
p.changeDynamics(cube, -1, restitution=1)
p.changeDynamics(cube, -1, lateralFriction=100000)
p.changeDynamics(cube, -1, spinningFriction=1000)
p.changeDynamics(cube, -1, mass=1)

p.changeDynamics(plane1, -1, restitution=1)
p.changeDynamics(plane1, -1, lateralFriction=100000)
p.changeDynamics(plane1, -1, spinningFriction=1000)

p.changeDynamics(plane2, -1, restitution=0)
p.changeDynamics(plane2, -1, lateralFriction=100000)
p.changeDynamics(plane2, -1, spinningFriction=1000)

p.changeDynamics(plane3, -1, restitution=0)
p.changeDynamics(plane3, -1, lateralFriction=100000)
p.changeDynamics(plane3, -1, spinningFriction=1000)

# Simulation
cubePos = np.zeros((900,3))
cubeOrn = np.zeros((900,4))
for i in range (900):
    p.stepSimulation()
    cubePos[i], cubeOrn[i] = p.getBasePositionAndOrientation(cube)
    time.sleep(1./240.)

p.disconnect()