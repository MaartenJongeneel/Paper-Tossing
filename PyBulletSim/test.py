import pybullet as p
import csv
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

#Set the gravity
p.setGravity(0,0,-9.81) #-9.81)

#Import states from csv file
with open('simstates/test_states.csv','r') as box_states:
    boxstates = csv.reader(box_states)

for row in boxstates:
    print(f'\t{row[12]} {row[13]} {row[14]}')

#Set the box restitution and friction
eN = 0.5
mu = 0.3

# Set the box initial position, orientation, lin velocity, and angular velocity
startPos = [0,0,0.1] #[x,y,z]
startOrientation = [0,0,0,1] # [x,y,z,w] quaternion
linearVelocity = [0,0,0.5]
angularVelocity = [0,0,2]

# Set contact plane base position and orientation
planePos = [0,0,0]
planeOrientation =  [0,0,0,1]

# Load the box and assign initial pose, velocity, and coefficients
boxId = p.loadURDF("urdf/box004.urdf",startPos, startOrientation)
p.resetBaseVelocity(boxId,linearVelocity,angularVelocity)
p.changeDynamics(boxId,-1,restitution=eN,lateralFriction=mu)

#Define the camera settings (for visualiztion only)
cam = p.getDebugVisualizerCamera()
p.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch = cam[9], cameraTargetPosition = [0,0,0],cameraDistance =1.2)


# Define the contact plane dimensions, collisionshape, position and orientation.
planeL, planeW, planeH = 1,1,0
sh_colPlane1 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[planeL,planeW,planeH])
plane1 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex=sh_colPlane1,basePosition=planePos,baseOrientation=planeOrientation)
p.changeDynamics(plane1,-1,restitution=1)
p.changeVisualShape(plane1, -1, rgbaColor=[150/255, 150/255, 150/255, 1])

p.setPhysicsEngineParameter(useSplitImpulse = 1) #this should prevent compensations due to penetrations 

#Simulate
for i in range (10000):    
    p.stepSimulation()    
    time.sleep(1./240.)

    #Write box position and orientation to file
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
p.disconnect()
