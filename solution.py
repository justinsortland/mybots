import pyrosim.pyrosim as pyrosim
import random
import numpy
import os
import time
import constants as c
import pybullet as p

class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        # self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)*3 - 1

        self.name_id = 0
        self.num_cubes = random.randint(3, 10)

        self.linksWithSensors = []
        for i in range(self.num_cubes):
            self.linksWithSensors.append(random.randint(0,1))

        self.num_sensors = 0
        for link in self.linksWithSensors:
            if self.linksWithSensors[link] == 1:
                self.num_sensors += 1

        self.num_motors = self.num_cubes - 1 

        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1


    def Start_Simulation(self, mode):
        self.Create_World()
        # self.Create_Body()
        # self.Create_Brain()
        self.Generate_Snake()
        self.Generate_Snake_Brain()
        # id = p.loadURDF("body.urdf")

        os.system("python3 simulate.py " + mode + " " + str(self.myID) + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(c.sleepTime)

        f = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(f.readlines()[0])
        f.close()

        os.system("rm fitness" + str(self.myID) + ".txt")

    def Mutate(self):
        # randomRow = random.randint(0, len(self.weights) - 1)
        # randomColumn = random.randint(0, len(self.weights[0]) - 1)
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)
        self.weights[randomRow][randomColumn] = random.random()*3 - 1

    def Set_ID(self, id):
        self.myID = id
        
    def Create_World(self):

        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2, 2, 0.5] , size=[1, 1, 1], colorString='    <color rgba="0 1.0 1.0 1.0"/>', colorName='Grey')
        pyrosim.End()

    # def Create_Body(self):
    #     while not os.path.exists("body.urdf"):
    #         time.sleep(0.01)

    #     pyrosim.Start_URDF("body.urdf")

    #     pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1] , size=[1, 1, 1])
    #     pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0] , size=[0.35, 1, 0.35])
    #     pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0] , size=[0.35, 1, 0.35])
    #     # pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0] , size=[1, 0.2, 0.2])
    #     # pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0] , size=[1, 0.2, 0.2])

    #     pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -0.5] , size=[0.35, 0.35, 1])
    #     pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -0.5] , size=[0.35, 0.35, 1])
    #     # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])
    #     # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])


    #     pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, -0.5, 1], jointAxis = "1 0 0")
    #     pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 0.5, 1], jointAxis = "1 0 0")
    #     # pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5, 0, 1], jointAxis = "0 1 0")
    #     # pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5, 0, 1], jointAxis = "0 1 0")

    #     pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0, 1, 0], jointAxis = "1 0 0")
    #     pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0, -1, 0], jointAxis = "1 0 0")
    #     # pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1, 0, 0], jointAxis = "0 1 0")
    #     # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1, 0, 0], jointAxis = "0 1 0")

    #     pyrosim.End()

    # def Create_Brain(self):
    #     # while not os.path.exists("brain" + str(self.myID) + ".nndf"):
    #     #     time.sleep(0.01)

    #     pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

    #     pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    #     pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    #     pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
    #     # pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
    #     # pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLeg")

    #     pyrosim.Send_Sensor_Neuron(name=3, linkName="FrontLowerLeg")
    #     pyrosim.Send_Sensor_Neuron(name=4, linkName="BackLowerLeg")
    #     # pyrosim.Send_Sensor_Neuron(name=7, linkName="LeftLowerLeg")
    #     # pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLowerLeg")

    #     pyrosim.Send_Motor_Neuron(name=5, jointName="Torso_BackLeg")
    #     pyrosim.Send_Motor_Neuron(name=6, jointName="Torso_FrontLeg")
    #     # pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_LeftLeg")
    #     # pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_RightLeg")

    #     pyrosim.Send_Motor_Neuron(name=7, jointName="FrontLeg_FrontLowerLeg")
    #     pyrosim.Send_Motor_Neuron(name=8, jointName="BackLeg_BackLowerLeg")
    #     # pyrosim.Send_Motor_Neuron(name=15, jointName="LeftLeg_LeftLowerLeg")
    #     # pyrosim.Send_Motor_Neuron(name=16, jointName="RightLeg_RightLowerLeg")

    #     # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=3, weight=-1.0)
    #     # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=-1.0)

    #     # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=-1.0)
    #     # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=4, weight=-1.0)\
        
    #     for currentRow in range(c.numSensorNeurons):
    #         for currentColumn in range(c.numMotorNeurons):
    #             pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])

    #     pyrosim.End()

    # def fdgsdfgdsfg(self):
    #     self.num_cubes = random.randint(3, 11) # randomly generate number of cubes
    #     num_sensors = random.randint(0, self.num_cubes) # randomly generate number of sensor neurons
    #     num_motors = random.randint(0, self.num_cubes) # randomly generate number of motor neurons

    #     # Add cubes
    #     for i in range(num_cubes):
    #         pyrosim.Send_Cube(name=i, x=0, y=0, z=i * 0.1 + 0.1, length=0.1, width=0.1, height=0.1)

    #     # Add sensor neurons
    #     for i in range(num_sensors):
    #         pyrosim.Send_Sensor_Neuron(neuronID=i, linkName=i)

    #     # Add motor neurons
    #     for i in range(num_motors):
    #         pyrosim.Send_Motor_Neuron(neuronID=i, jointName=i)

    def Generate_Snake(self):

        pyrosim.Start_URDF("body.urdf")

        cubePosX = 0
        cubePosY = 0
        cubePosZ = 1
        cubePos = [cubePosX, cubePosY, cubePosZ]

        jointPosX = 0.5
        jointPosY = 0
        jointPosZ = 0.5
        jointPos = [jointPosX, jointPosY, jointPosZ]

        cubeSizeX = 1
        cubeSizeY = 1
        cubeSizeZ = 1
        cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

        for i in range(self.num_cubes):
            cubeName = "Segment" + str(i) 
            if self.linksWithSensors[i] == 1:
                pyrosim.Send_Cube(name=cubeName, pos=cubePos, size=cubeSize, colorString="0 255.0 0 1.0", colorName='Green')
            else:
                pyrosim.Send_Cube(name=cubeName, pos=cubePos, size=cubeSize, colorString="0 0 255.0 1.0", colorName='Blue')

            if i != (self.num_cubes-1):
                parentName = cubeName
                childName = "Segment" + str(i+1)
                jointName = parentName + "_" + childName

                axis = str(random.uniform(-1, 1)) + " " + str(random.uniform(-1, 1)) + " " + str(random.uniform(-1, 1))
                pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=jointPos, jointAxis=axis)

                cubeSizeX = random.uniform(0.8, 1.2)
                cubeSizeY = random.uniform(0.8, 1.2)
                cubeSizeZ = random.uniform(0.8, 1.2)
                cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

                cubePosX = cubeSizeX/2
                jointPosX = cubeSizeX
                jointPos = [jointPosX, jointPosY, jointPosZ]

                cubePos = [cubePosX, cubePosY, cubePosZ]

        pyrosim.End()

    def Generate_Snake_Brain(self):

        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        for i in range(self.num_sensors):
            sensorNeuronName = self.name_id
            sensorNeuronLinkName = "Segment" + str(self.name_id)
            pyrosim.Send_Sensor_Neuron(name=sensorNeuronName, linkName=sensorNeuronLinkName)
            self.name_id += 1

        for i in range(self.num_motors):
            motorNeuronName = self.name_id
            motorNeuronJointName = "Segment" + str(i) + "_" + "Segment" + str(i+1)
            pyrosim.Send_Motor_Neuron(name=motorNeuronName, jointName=motorNeuronJointName)
            self.name_id += 1

        for currentRow in range(self.num_sensors):
            for currentColumn in range(self.num_motors):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+self.num_sensors, weight=self.weights[currentRow][currentColumn])

        pyrosim.End()