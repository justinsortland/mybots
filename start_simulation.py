import pyrosim.pyrosim as pyrosim
import random
import numpy
import os
import time
import constants as c

class SOLUTION:
    def __init__(self, myID):
        self.myID = myID

        self.name_id = 0
        self.segment_id = 0
        
        self.numCubesArray = []
        for i in range(3):
            self.numCubesArray.append(random.randint(3,6))

        self.linksWithSensors = []
        for i in range(sum(self.numCubesArray)):
            self.linksWithSensors.append(random.randint(0,1))

        self.num_sensors = 0
        for link in self.linksWithSensors:
            if self.linksWithSensors[link] == 1:
                self.num_sensors += 1

        self.num_motors = sum(self.numCubesArray) - 1 

        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1

    def Start_Simulation(self, mode):
        self.Generate_World()
        self.Generate_Creature()
        self.Generate_Creature_Brain()
        os.system("python3 simulate.py " + mode + " " + str(self.myID) + " 2&>1 &")

    def Generate_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2, 2, 0.5] , size=[1, 1, 1], colorString='    <color rgba="0 1.0 1.0 1.0"/>', colorName='Grey')
        pyrosim.End()
        
    def Generate_Creature(self):
        pyrosim.Start_URDF("body.urdf")
        
        cubePosX = random.uniform(-1,1)
        cubePosY = random.uniform(-1,1)
        cubePosZ = random.uniform(3,4)
        cubePosition = [cubePosX, cubePosY, cubePosZ]

        cubeSizeX = random.uniform(0.5,1.5)
        cubeSizeY = random.uniform(0.5,1.5)
        cubeSizeZ = random.uniform(0.5,1.5)
        cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

        faces = [[cubePosition[0], cubePosition[1] + (cubeSize[1]/2), cubePosition[2]],
                 [cubePosition[0], cubePosition[1] - (cubeSize[1]/2), cubePosition[2]],
                 [cubePosition[0] - (cubeSize[0]/2), cubePosition[1], cubePosition[2]],
                 [cubePosition[0] + (cubeSize[0]/2), cubePosition[1], cubePosition[2]],
                 [cubePosition[0], cubePosition[1], cubePosition[2] + (cubeSize[2]/2)],
                 [cubePosition[0], cubePosition[1], cubePosition[2] - (cubeSize[2]/2)]]
        
        directions = []
        for k in range(3):
            direction = random.choice(faces)
            directions.append(direction)
            faces.remove(direction)
        
        cubeDictionary = {}
        
        xMin = cubePosition[0] - (cubeSize[0]/2)
        xMax = cubePosition[0] + (cubeSize[0]/2)

        yMin = cubePosition[1] - (cubeSize[1]/2)
        yMax = cubePosition[1] + (cubeSize[1]/2)

        zMin = cubePosition[2] - (cubeSize[2]/2)
        zMax = cubePosition[2] + (cubeSize[2]/2)
            
        space = [[xMin,xMax],
                 [yMin,yMax],
                 [zMin,zMax]]
        
        for index,direction in enumerate(directions):

            jointPosition = direction

            for j in range(self.numCubesArray[index]):
                cubeName = "Segment" + str(self.segment_id)

                cubeInfo = {'name':cubeName, 
                            'faces':faces, 
                            'cubepos':cubePosition, 
                            'cubesize':cubeSize, 
                            'jointpos':jointPosition, 
                            'sensor':self.linksWithSensors[self.segment_id],
                            'space':space,
                            'parent':None, 
                            'child':None,
                            'direction':direction}
                
                cubeDictionary[cubeName] = cubeInfo
                            
                if j != 0 and index != 0 and 'Segment' + str(self.segment_id) in cubeDictionary and 'Segment' + str(self.segment_id-1) in cubeDictionary and cubeDictionary['Segment' + str(self.segment_id-1)]['direction'] == cubeDictionary['Segment' + str(self.segment_id)]['direction']:
                    cubeInfo['parent'] = "Segment" + str(self.segment_id-1)

                if j != (self.numCubesArray[index]-1) and 'Segment' + str(self.segment_id) in cubeDictionary and 'Segment' + str(self.segment_id+1) in cubeDictionary and cubeDictionary['Segment' + str(self.segment_id+1)]['direction'] == cubeDictionary['Segment' + str(self.segment_id)]['direction']:
                    cubeInfo['child'] = "Segment" + str(self.segment_id+1)

                if cubeInfo['sensor'] == 1:
                    pyrosim.Send_Cube(name=cubeName, pos=cubePosition, size=cubeSize, colorString="0 255.0 0 1.0", colorName='Green')
                else:
                    pyrosim.Send_Cube(name=cubeName, pos=cubePosition, size=cubeSize, colorString="0 0 255.0 1.0", colorName='Blue')
                
                if j != (self.numCubesArray[index]-1):
                    parentName = cubeName
                    childName = "Segment" + str(self.segment_id+1)
                    jointName = parentName + "_" + childName
                    axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))
                    pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=jointPosition, jointAxis=axis)

                    prevCubeSize = cubeSize
                    
                    cubeSizeX = random.uniform(0.5,1.5)
                    cubeSizeY = random.uniform(0.5,1.5)
                    cubeSizeZ = random.uniform(0.5,1.5)
                    cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

                    chosenDirection = random.choice(cubeSize)
                    chosenDirectionDimension = cubeSize.index(chosenDirection)

                    if chosenDirectionDimension == 0:
                        for a,cube in enumerate(cubeDictionary):
                            key = 'Segment' + str(self.segment_id)
                            if key in cubeDictionary:
                                cubeSpace = cubeDictionary[key]['space']
                                if cubePosition[chosenDirectionDimension] > cubeSpace[0][0] and cubePosition[chosenDirectionDimension] < cubeSpace[0][1] and cubePosition[1] > cubeSpace[1][0] and cubePosition[1] < cubeSpace[1][1] and cubePosition[2] > cubeSpace[2][0] and cubePosition[2] < cubeSpace[2][1]:
                                    prevCubeSizeInDimension = prevCubeSize[chosenDirectionDimension]
                                    cubePosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                    # if index != self.numCubesArray[index]-1:
                                    jointPosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                # else:
                                #     chosenDirection = random.choice(cubeSize)
                                #     chosenDirectionDimension = cubeSize.index(chosenDirection)

                    elif chosenDirectionDimension == 1:
                        for b,cube in enumerate(cubeDictionary):
                            key = 'Segment' + str(self.segment_id)
                            if key in cubeDictionary:
                                cubeSpace = cubeDictionary[key]['space']
                                if cubePosition[chosenDirectionDimension] > cubeSpace[1][0] and cubePosition[chosenDirectionDimension] < cubeSpace[1][1] and cubePosition[0] > cubeSpace[0][0] and cubePosition[0] < cubeSpace[0][1] and cubePosition[2] > cubeSpace[2][0] and cubePosition[2] < cubeSpace[2][1]:
                                    prevCubeSizeInDimension = prevCubeSize[chosenDirectionDimension]
                                    cubePosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                    # if index != self.numCubesArray[index]-1:
                                    jointPosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                # else:
                                #     chosenDirection = random.choice(cubeSize)
                                #     chosenDirectionDimension = cubeSize.index(chosenDirection)

                    else:
                        for c,cube in enumerate(cubeDictionary):
                            key = 'Segment' + str(self.segment_id)
                            if key in cubeDictionary:
                                cubeSpace = cubeDictionary[key]['space']
                                if cubePosition[chosenDirectionDimension] > cubeSpace[2][0] and cubePosition[chosenDirectionDimension] < cubeSpace[2][1] and cubePosition[0] > cubeSpace[0][0] and cubePosition[0] < cubeSpace[0][1] and cubePosition[1] > cubeSpace[1][0] and cubePosition[1] < cubeSpace[1][1]:
                                    prevCubeSizeInDimension = prevCubeSize[chosenDirectionDimension]
                                    cubePosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                    # if index != self.numCubesArray[index]-1:
                                    jointPosition[chosenDirectionDimension] = (chosenDirection + prevCubeSizeInDimension)/4
                                # else:
                                #     chosenDirection = random.choice(cubeSize)
                                #     chosenDirectionDimension = cubeSize.index(chosenDirection)

                    xMin = cubePosition[0] - (cubeSize[0]/2)
                    xMax = cubePosition[0] + (cubeSize[0]/2)

                    yMin = cubePosition[1] - (cubeSize[1]/2)
                    yMax = cubePosition[1] + (cubeSize[1]/2)

                    zMin = cubePosition[2] - (cubeSize[2]/2)
                    zMax = cubePosition[2] + (cubeSize[2]/2)
                
                    space = [[xMin,xMax],
                             [yMin,yMax],
                             [zMin,zMax]]
                    
                    faces = [[cubePosition[0], cubePosition[1] + (cubeSize[1]/2), cubePosition[2]],
                            [cubePosition[0], cubePosition[1] - (cubeSize[1]/2), cubePosition[2]],
                            [cubePosition[0] - (cubeSize[0]/2), cubePosition[1], cubePosition[2]],
                            [cubePosition[0] + (cubeSize[0]/2), cubePosition[1], cubePosition[2]],
                            [cubePosition[0], cubePosition[1], cubePosition[2] + (cubeSize[2]/2)],
                            [cubePosition[0], cubePosition[1], cubePosition[2] - (cubeSize[2]/2)]]
                    
                self.segment_id += 1

        pyrosim.End()

    def Generate_Creature_Brain(self):
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

solution = SOLUTION(0)
solution.Start_Simulation("GUI")