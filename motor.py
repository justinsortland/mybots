import numpy
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName, amplitude, frequency, offset, rad):
        self.jointName = jointName
        self.amplitude = amplitude
        self.frequency = frequency
        self.offset = offset
        self.rad = rad
        self.Prepare_To_Act()


    def Prepare_To_Act(self):
      
        # backLegtargetAngles[i] = c.backLegAmplitude * numpy.sin(c.backLegFrequency * c.rad[i] + c.backLegPhaseOffset)
        # frontLegtargetAngles[i] = c.frontLegAmplitude * numpy.sin(c.frontLegFrequency * c.rad[i] + c.frontLegPhaseOffset)

        if self.jointName == "Torso_BackLeg":
            self.motorValues = self.amplitude * numpy.sin((self.frequency/2) * self.rad + self.offset)
        else:
            self.motorValues = self.amplitude * numpy.sin(self.frequency * self.rad + self.offset)

        # for linkName in pyrosim.linkNamesToIndices:
        #     self.motorValues[linkName] = MOTOR(linkName)
    
    def Set_Value(self, robot, t):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robot, jointName=self.jointName, controlMode=p.POSITION_CONTROL, targetPosition=self.motorValues[t], maxForce=50)
        # pyrosim.Set_Motor_For_Joint(bodyIndex=self.robot.robotId, jointName="Torso_FrontLeg", controlMode=p.POSITION_CONTROL, targetPosition=frontLegtargetAngles[i], maxForce=50)

    def Save_Values(self):
        numpy.save("data/motorValues.npy", self.motorValues)
        # numpy.save("data/frontLegtargetAngles.npy", frontLegtargetAngles)      
