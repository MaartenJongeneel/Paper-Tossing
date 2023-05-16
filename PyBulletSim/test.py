import pybullet as p
import csv
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

#Define the object and parameter method (Trajectory (Traj) or Velocity (Vel))
object = "Box004"
param = "Traj"

#Set the gravity
p.setGravity(0,0,-9.81) 

#Define the camera settings (for visualiztion only)
cam = p.getDebugVisualizerCamera()
p.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch = -15, cameraTargetPosition = [0,0,0],cameraDistance =1.6)

#Import states from csv file. Each row has the structure [startPos[x,y,z] startOrientation[x,y,z,w] linearVelocity[x,y,z] angularVelocity[x,y,z] planePos[x,y,z] planeOrientation[x,y,z,w]] 
path = 'simstates/' + object + '_' + param + '/test_states.csv'
with open(path,'r') as box_states:
    boxstates = csv.reader(box_states, quoting=csv.QUOTE_NONNUMERIC)

    for row in boxstates:

        #Set the box restitution and friction
        eN = 0.5
        mu = 0.3

        # Set the box initial position, orientation, lin velocity, and angular velocity
        startPos = [row[0],row[1],row[2]] #[x,y,z]
        startOrientation = [row[3],row[4],row[5],row[6]] # [x,y,z,w] quaternion
        linearVelocity = [row[7],row[8],row[9]]
        angularVelocity = [row[10],row[11],row[12]]

        # Set contact plane base position and orientation
        planePos = [row[13],row[14],row[15]]
        planeOrientation =  [row[16],row[17],row[18],row[19]]

        # Load the box and assign initial pose, velocity, and coefficients
        urdf = "urdf/" + object + ".urdf"
        boxId = p.loadURDF(urdf,startPos, startOrientation)
        p.resetBaseVelocity(boxId,linearVelocity,angularVelocity)
        p.changeDynamics(boxId,-1,restitution=eN,lateralFriction=mu)


        # Define the contact plane dimensions, collisionshape, position and orientation.
        planeL, planeW, planeH = 20,20,0
        sh_colPlane1 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[planeL,planeW,planeH])
        plane1 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex=sh_colPlane1,basePosition=planePos,baseOrientation=planeOrientation)
        p.changeDynamics(plane1,-1,restitution=1,lateralFriction=1)
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
