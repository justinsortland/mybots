import pyrosim.pyrosim as pyrosim
import pybullet as p
import constants as c

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.offset
        self.rad = c.rad
    
    def Set_Value(self, robot, desiredAngle):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robot, jointName=self.jointName, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, maxForce=50)
        # pyrosim.Set_Motor_For_Joint(bodyIndex=self.robot.robotId, jointName="Torso_FrontLeg", controlMode=p.POSITION_CONTROL, targetPosition=frontLegtargetAngles[i], maxForce=50)