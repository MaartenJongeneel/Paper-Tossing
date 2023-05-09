import pybullet as p
import csv
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

#Set the gravity
p.setGravity(0,0,-1) #-9.81)

#Import states from csv file
with open('simstates/test_states.csv','r') as file:
    csvreader = csv.reader(file)
    for row in csvreader:
        print(f'\t{row[12]} {row[13]} {row[14]}')

#Set the box start position,orientation and restitution
startPos = [0,0,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf/box004.urdf",startPos, startOrientation)
p.changeDynamics(boxId,-1,restitution=0.5)

#Define the camera settings (for visualiztion only)
cam = p.getDebugVisualizerCamera()
p.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch = cam[9], cameraTargetPosition = [0,0,0],cameraDistance =1.2)


# Define the contact plane dimensions, collisionshape, position and orientation.
planeL = 1
planeW = 1
planeH = 0
sh_colPlane1 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[planeL,planeW,planeH])
plane1 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex=sh_colPlane1,basePosition=[0,0,0],baseOrientation=p.getQuaternionFromEuler([0,0,0]))
p.changeDynamics(plane1,-1,restitution=1)
p.changeVisualShape(plane1, -1, rgbaColor=[150/255, 150/255, 150/255, 1])

p.setPhysicsEngineParameter(useSplitImpulse = 1) #this should prevent compensations due to pennetrations 

#Simulate
for i in range (10000):    
    p.stepSimulation()    
    time.sleep(1./240.)

    #Write box position and orientation to file
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
p.disconnect()
