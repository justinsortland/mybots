import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
import constants as c
import numpy

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

    def Get_Fitness(self):
        stateOfLinkZero = p.getBasePositionAndOrientation(self.robot)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        return xCoordinateOfLinkZero

    def Write_Fitness(self, fitness1, fitness2, id):
        dist = fitness1*fitness1 + fitness2*fitness2
        dist = numpy.sqrt(dist)
        # Mention in README how write_fitness like this
        # Restraint limbs can only be connected to 2 other ones

        f = open("tmp" + str(id) + ".txt", "w")
        f.write(str(dist))
        f.close()
        os.system("mv tmp" + str(id) + ".txt " + "fitness" + str(id) + ".txt")


