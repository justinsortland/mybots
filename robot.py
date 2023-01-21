import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c

from sensor import SENSOR
from motor import MOTOR

from pyrosim.neuralNetwork import NEURAL_NETWORK


class ROBOT:
    def __init__(self):
        self.motors = {}
        self.robot = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robot)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, c.amplitude, c.frequency, c.offset, c.rad)

    def Act(self, robot, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                print(neuronName)
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                print(jointName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                print(desiredAngle)

        # for motor in self.motors:
        #     self.motors[motor].Set_Value(robot, t)

    def Think(self):
        self.nn.Update()
        self.nn.Print()