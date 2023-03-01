import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
import constants as c

from sensor import SENSOR
from motor import MOTOR

from pyrosim.neuralNetwork import NEURAL_NETWORK

class ROBOT:
    def __init__(self, solutionID):
        self.motors = {}
        self.robot = p.loadURDF("body" + str(solutionID) + ".urdf")
        pyrosim.Prepare_To_Simulate(self.robot)
        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        os.system("rm body" + str(solutionID) + ".urdf")
        
        os.system("rm brain" + str(solutionID) + ".nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in self.nn.Generate_List_Of_Sensor_Neuron_Links():
            self.sensors[linkName] = SENSOR(linkName)

        # breakpoint()

    def Sense(self, t):
        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, robot, desiredAngle):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                desiredAngle *= c.motorJointRange
                self.motors[jointName].Set_Value(robot, desiredAngle)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self, id):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]

        # xCoordinates = []

        # for i in range(p.getNumJoints(self.robot)):
        #     stateOfLink = p.getLinkState(self.robot,i)
        #     positionOfLink = stateOfLink[0]

        #     xCoordinateOfLink = positionOfLink[0]

        #     xCoordinates.append(xCoordinateOfLink)

        # xCenterOfMass = sum(xCoordinates) / len(xCoordinates)

        f = open("tmp" + str(id) + ".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        # f.write(str(xCenterOfMass))
        f.close()
        os.system("mv tmp" + str(id) + ".txt " + "fitness" + str(id) + ".txt")
