#https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf
# press w for skleton view
#tested python 3.7.3

import pybullet as p
import pybullet_data
import math
import numpy as np
import time
import numpy as np
import csv
from fkin import *

class pyRobot:
    def __init__(self):
        print("Using pyBullet Version: ", p.getAPIVersion())
        self.enable_write = False
        self.PathS2 = False

        self.grip_l = 0.0

    def connect(self):        
        #create pybullet env
        self.physicsClient = p.connect(p.GUI)        
        self.resetSim()

    def disconnect(self):
        p.disconnect()

    def resetSim(self):
        p.resetSimulation()

    #enable simulation in seprate thread
    def enableRealTimeSim(self):
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.physicsClient)

    #enable gravity
    def enableGravity(self, gX=0, gY=0, gZ=0):
        p.setGravity(gX, gY, gZ)

    #disable gravity
    def disableGravity(self, gX=0, gY=0, gZ=0):
        p.setGravity(gX, gY, gZ)

    #load urdf in the virtual environment
    def loadPlane(self):
        #load addons
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeID = p.loadURDF("plane.urdf")

    def load_iiwa(self):
        self.iiwa = p.loadURDF("/cad_data/lbr_iiwa7_r800/urdf/lbr_iiwa7_r800.urdf", [0, 0, 0], globalScaling=1, useFixedBase=1)
    
    #red sphere
    def loadSphere1(self):
        #sphere1 initial location
        self.s1 = [0.5,0,1.2]
        
        self.sphere = p.loadURDF("/sphere/urdf/sphere.urdf", self.s1, globalScaling=0.1, useFixedBase=1)
        self.sphere_slider()

    #blue sphere   
    def loadSphere3(self):
        #sphere3 initial location
        self.s3 = [-0.5,0,1.2]
        self.s3x_i, self.s3y_i, self.s3z_i = self.s3[0], self.s3[1], self.s3[2]
        self.s3x, self.s3y, self.s3z = self.s3[0], self.s3[1], self.s3[2]
        
        self.sphere3 = p.loadURDF("/sphere/urdf/sphere3.urdf", self.s3, globalScaling=0.1, useFixedBase=1)

    #display axis
    def showArmAxis(self, bodyID=None, index=None):
        x=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0.1,0,0], lineColorRGB = [1, 0, 0],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID, parentLinkIndex=index )
        y=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0,0.1,0], lineColorRGB = [0, 1, 0],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID, parentLinkIndex=index )
        z=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0,0,0.1], lineColorRGB = [0, 0, 1],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID, parentLinkIndex=index )

    def showBodyAxis(self, bodyID=None):
        x=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0.1,0,0], lineColorRGB = [1, 0, 0],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID )
        y=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0,0.1,0], lineColorRGB = [0, 1, 0],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID )
        z=p.addUserDebugLine(lineFromXYZ = [0,0,0], lineToXYZ = [0,0,0.1], lineColorRGB = [0, 0, 1],
                           lineWidth = 0.1, lifeTime = 0, parentObjectUniqueId = bodyID )

   #load robot     
    def loadRobot(self):
        self.load_iiwa()

        self.showArmAxis(self.iiwa, 6)
        self.showArmAxis(self.iiwa, 6)

    #load all urdf   
    def loadUrdf(self):
        self.loadPlane()
        self.loadRobot()

        self.loadSphere1()
        self.loadSphere3()
        self.showBodyAxis(self.sphere)
        self.showBodyAxis(self.sphere3)


    ################################################
    #send joint command to virtual robot
    def joint_command(self, lArm=[0, 0, 0, 0, 0, 0, 0]):
        if self.enable_write==True:
             self.write_data(data=[lArm])
             
        #for moving all joint in array
        p.setJointMotorControlArray(self.iiwa, [0,1,2,3,4,5,6] , p.POSITION_CONTROL, lArm)

        self.ltheta = lArm
    
    #calucate inverse kinematics of the arm
    def ikin(self, lpose=[0,0,0,0,0,0]):
        
        self.ltheta=p.calculateInverseKinematics(bodyUniqueId = self.iiwa,
                                                 endEffectorLinkIndex=6,
                                                 targetPosition=lpose[:3],
                                                 targetOrientation=p.getQuaternionFromEuler(lpose[3:6]),
                                                 solver=p.IK_DLS)
    #get pose
    def pose(self):
        print(forward_kinematics(self.ltheta))

    #virtual robot home pose
    def home(self):
         self.joint_command()

    #get state of all joint
    def getState(self):
        print(p.getJointStates(self.iiwa,[0,1,2,3,4,5,6]))

    #step simulation
    def enableStepSim(self):
        while True:
            p.getCameraImage(320,200)
            p.stepSimulation()
    
     ################################################
    #slider for sphere1
    def sphere_slider(self):
        self.slider_sx= p.addUserDebugParameter("MoveX", -3.14, 3.14,0.0)
        self.slider_sy= p.addUserDebugParameter("MoveY", -3.14, 3.14,0.0)
        self.slider_sz= p.addUserDebugParameter("MoveZ", -3.14, 3.14,0.0)

        self.slider_sroll= p.addUserDebugParameter("Roll", -3.14, 3.14,0.0)
        self.slider_spitch= p.addUserDebugParameter("Pitch", -3.14, 3.14,0.0)
        self.slider_syaw= p.addUserDebugParameter("Yaw", -3.14, 3.14,0.0)

    #update pose from slider of sphere1   
    def moveSphere(self): 
         self.sx= p.readUserDebugParameter(self.slider_sx)
         self.sy= p.readUserDebugParameter(self.slider_sy)
         self.sz= p.readUserDebugParameter(self.slider_sz)

         self.sroll= p.readUserDebugParameter(self.slider_sroll)
         self.spitch= p.readUserDebugParameter(self.slider_spitch)
         self.syaw= p.readUserDebugParameter(self.slider_syaw)
         
         p.resetBasePositionAndOrientation(self.sphere, [self.sx,self.sy,self.sz], p.getQuaternionFromEuler([self.sroll,self.spitch,self.syaw]))

    #move robot arm tracking sphere1
    def trackSphere1(self):
        running = True
        
        while running:
            events = p.getKeyboardEvents()
            if p.B3G_BACKSPACE in events:
                running = False
            
            self.moveSphere()
            self.ikin(lpose=[self.sx, self.sy, self.sz, 0, 0, 0])
            self.joint_command(lArm=self.ltheta)
    
    ################################################
    #move robot arm tracking sphere3
    def move(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.s3x, self.s3y, self.s3z = self.s3x+(x/1000), self.s3y+(y/1000), self.s3z+(z/1000)
        p.resetBasePositionAndOrientation(self.sphere3, [self.s3x, self.s3y, self.s3z], [0,0,0,1])
        self.trackSphere3(self.s3x, self.s3y, self.s3z)

    def move_abs(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.s3x, self.s3y, self.s3z = (x/1000), (y/1000), (z/1000)
        p.resetBasePositionAndOrientation(self.sphere3, [self.s3x, self.s3y, self.s3z], [0,0,0,1])
        self.trackSphere3(self.s3x, self.s3y, self.s3z)
        
    def trackSphere3(self, x=0, y=0, z=0):
        self.ikin(lpose=[x, y, z, 0, 0, 0])
        self.joint_command(lArm=self.ltheta)

    ################################################
    #to store joints pos in csv 
    def storeData(self, data=True):
        self.file_path = 'output4.csv'
        self.enable_write = data
    
    def write_data(self, data):        
        with open(self.file_path, mode='a', newline='') as file:
              writer = csv.writer(file)
              writer.writerows(data)









