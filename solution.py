import pyrosim.pyrosim as pyrosim
import random
import numpy
import os
import time
import constants as c

class SOLUTION:
    def __init__(self, myID):
        self.myID = myID

        random.seed(8)

        self.segment_id = 0
        
        self.num_torso_cubes = random.randint(4,5)

        self.numLeg1Segments = random.randint(2,3)
        self.numLeg2Segments = random.randint(2,3)

        self.total_cubes = self.num_torso_cubes + self.numLeg1Segments + self.numLeg2Segments

        self.torsoLinksWithSensors = []

        self.leg1LinksWithSensors = []
        self.leg2LinksWithSensors = []

        for i in range(self.num_torso_cubes):
            self.torsoLinksWithSensors.append(random.randint(0,1))

        for i in range(self.numLeg1Segments):
            self.leg1LinksWithSensors.append(random.randint(0,1))

        for i in range(self.numLeg2Segments):
            self.leg2LinksWithSensors.append(random.randint(0,1))

        self.num_sensors = 0
        for link1 in self.torsoLinksWithSensors:
            if self.torsoLinksWithSensors[link1] == 1:
                self.num_sensors += 1

        for link2 in self.leg1LinksWithSensors:
            if self.leg1LinksWithSensors[link2] == 1:
                self.num_sensors += 1

        for link3 in self.leg2LinksWithSensors:
            if self.leg2LinksWithSensors[link3] == 1:
                self.num_sensors += 1

        if self.num_sensors == 0:
            self.num_sensors = 1

        self.linksWithLegs = [1,1]
        for i in range(self.num_torso_cubes-2):
            self.linksWithLegs.append(0)

        while self.linksWithLegs[0] == 1 or self.linksWithLegs[-1] == 1:
            random.shuffle(self.linksWithLegs)

        for i,link in enumerate(self.linksWithLegs):
            if i != (len(self.linksWithLegs)-1):
                if self.linksWithLegs[i] == 1 and self.linksWithLegs[i+1] == 1:
                    random.shuffle(self.linksWithLegs)

        if self.total_cubes < 2:
            self.total_cubes = 2
                    
        self.num_motors = self.total_cubes - 1 

        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1

        self.leg_id = 0

        self.linkNamesWithSensors = []
        self.jointNamesWithMotors = []

        self.cubeSpaceDict = {}

        self.cubeInfoDict = {}
        self.jointInfoDict = {}

    def Start_Simulation(self, mode):
        self.Generate_World()
        self.leg_id = 0
        self.Generate_Torso()
        self.Generate_Brain()
        os.system("python3 simulate.py " + mode + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(c.sleepTime)
        f = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(f.readlines()[0])
        f.close()
        os.system("rm fitness" + str(self.myID) + ".txt")

    def Mutate(self):
        mutations = ['Mutate Weights', 'Mutate Sensors']
        mutation = random.choice(mutations)
        if mutation == 'Mutate Weights':
            self.Mutate_Weights()
        elif mutation == 'Mutate Sensors':
            self.Mutate_Sensors()

    def Mutate_Weights(self):
        if self.num_sensors == 1 or self.num_sensors == 0:
            randomRow = 0
        else:
            randomRow = random.randint(0, self.num_sensors - 1)

        if self.num_motors == 1 or self.num_motors == 0:
            randomColumn = 0
            self.num_motors = 2
        else:
            randomColumn = random.randint(0, self.num_motors - 1)

        self.weights[randomRow][randomColumn] = random.random()*3 - 1        

    def Mutate_Sensors(self):
        for i in range(self.num_torso_cubes):
            self.torsoLinksWithSensors[i] = random.randint(0,1)

        for i in range(self.numLeg1Segments):
            self.leg1LinksWithSensors[i] = random.randint(0,1)

        for i in range(self.numLeg2Segments):
            self.leg2LinksWithSensors[i] = random.randint(0,1)

    def Mutate_Body(self):
        mutations = ['Add Limb', 'Remove Limb']
        mutation = random.choice(mutations)
        if mutation == 'Add Limb':
            directions = []
        else:
            pass

    def Set_ID(self, id):
        self.myID = id
    
    def Generate_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[6, 6, 0.5] , size=[1, 1, 1], colorString='    <color rgba="0 1.0 1.0 1.0"/>', colorName='Grey')
        pyrosim.End()

    def Generate_Torso(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
        
        # Initial cube information
        torsoCubePosition = [0,0,0.5]
        torsoJointPosition =[0.5,0,0.5] 
        torsoCubeSize = [1,1,1]
        torsoCubeSizeArray = []

        # For loop for generating torso cubes
        for i in range(self.num_torso_cubes):
            torsoCubeName = "Torso" + str(i)
            if self.torsoLinksWithSensors[i] == 1:
                self.linkNamesWithSensors.append(torsoCubeName)
                pyrosim.Send_Cube(name=torsoCubeName, pos=torsoCubePosition, size=torsoCubeSize, colorString="0 255.0 0 1.0", colorName='Green')
            else:
                pyrosim.Send_Cube(name=torsoCubeName, pos=torsoCubePosition, size=torsoCubeSize, colorString="0 0 255.0 1.0", colorName='Blue')
            
            cubeInfo = {'cubeName':torsoCubeName,
                        'cubePosition':torsoCubePosition,
                        'cubeSize':torsoCubeSize,
                        'hasSensor':True if self.torsoLinksWithSensors[i] == 1 else False,
                        'jointBefore': None,
                        'jointAfter': None,
                       }
                        
            self.cubeInfoDict[torsoCubeName] = cubeInfo

            if i != (self.num_torso_cubes-1):
                torsoCubeSize = [random.uniform(0.8,1.2),random.uniform(0.8,1.2),random.uniform(0.8,1.2)]
                torsoCubeSizeArray.append(torsoCubeSize)

                torsoCubePosition[0] = (torsoCubeSizeArray[i][0])/2
                torsoCubePosition[2] = torsoCubeSizeArray[i][2]/4

            # Use helper function to generating leg cubes and then respective leg joints
            if self.linksWithLegs[i] == 1:
                self.Generate_Leg(torsoCubePosition, torsoCubeSize, torsoCubeName, self.leg_id)
                         
        # For loop for generating joints
        for j in range(self.num_torso_cubes):
            if j != (self.num_torso_cubes-1):
                parentName = "Torso" + str(j)
                childName = "Torso" + str(j+1)
                jointName = parentName + "_" + childName
                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))                
                pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=torsoJointPosition, jointAxis=axis)
                self.jointNamesWithMotors.append(jointName)

                jointInfo = {'jointName':jointName,
                             'jointPosition':torsoJointPosition,
                             'hasParent':parentName,
                             'hasChild':childName,
                            }
                
                self.jointInfoDict[jointName] = jointInfo
                
                torsoJointPosition[0] = torsoCubeSizeArray[j][0]

        # for joint in self.jointInfoDict:
        #     print(self.jointInfoDict[joint]['hasChild'])
            # if self.jointInfoDict[joint]['hasChild'] == torsoCubeName:
            #     self.cubeInfoDict['jointBefore'] = self.jointInfoDict['jointName']
            # if self.jointInfoDict[joint]['hasParent'] == torsoCubeName:
            #     self.cubeInfoDict['jointAfter'] = self.jointInfoDict['jointName']

        pyrosim.End()

    def Generate_Leg(self, torsoCubePosition, torsoCubeSize, torsoCubeName, legID):
        legLinksWithSensors = []
        if legID == 0:
            numLegSegments = self.numLeg1Segments
            legLinksWithSensors = self.leg1LinksWithSensors
        else:
            numLegSegments = self.numLeg2Segments
            legLinksWithSensors = self.leg2LinksWithSensors
        faces = [[torsoCubePosition[0], torsoCubePosition[1] + (torsoCubeSize[1]/2), torsoCubePosition[2]],
                 [torsoCubePosition[0], torsoCubePosition[1], torsoCubePosition[2] + (torsoCubeSize[2]/2)]]
        chosenDirection = random.choice(faces)
        k = faces.index(chosenDirection)
        index = 1 if k == 0 else 2

        legCubePosition = [torsoCubePosition[0], (torsoCubeSize[1])/2, torsoCubePosition[2]] if index == 1 else [torsoCubePosition[0], torsoCubePosition[1], (torsoCubeSize[2])/2]
        legJointPosition = [torsoCubePosition[0], torsoCubeSize[1], torsoCubePosition[2]] if index == 1 else [torsoCubePosition[0], torsoCubePosition[1], torsoCubeSize[2]]
        legCubeSize = [random.uniform(0.6,1.2),random.uniform(0.6,1.2),random.uniform(0.6,1.2)]
        legCubeSizeArray = []

        # For loop for generating cubes
        for i in range(numLegSegments):
            legCubeName = "Leg" + str(self.leg_id) + "Part" + str(i)
            if legLinksWithSensors[i] == 1:
                self.linkNamesWithSensors.append(legCubeName)
                pyrosim.Send_Cube(name=legCubeName, pos=legCubePosition, size=legCubeSize, colorString="0 255.0 0 1.0", colorName='Green')
            else:
                pyrosim.Send_Cube(name=legCubeName, pos=legCubePosition, size=legCubeSize, colorString="0 0 255.0 1.0", colorName='Blue')

            cubeInfo = {'cubeName':legCubeName,
                        'cubePosition':legCubePosition,
                        'cubeSize':legCubeSize,
                        'hasSensor':True if legLinksWithSensors[i] == 1 else False
                        }
            
            self.cubeInfoDict[legCubeName] = cubeInfo

            if i != (numLegSegments-1):
                legCubeSize = [random.uniform(0.6,1.2),random.uniform(0.6,1.2),random.uniform(0.6,1.2)]
                legCubeSizeArray.append(legCubeSize)

                legCubePosition[index] = (legCubeSizeArray[i][index])/2

        # For loop for generating joints
        for j in range(numLegSegments):
            if j != (numLegSegments-1):
                parentName = "Leg" + str(self.leg_id) + "Part" + str(j)
                childName = "Leg" + str(self.leg_id) + "Part" + str(j+1)
                jointName = parentName + "_" + childName
                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))                
                pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=legJointPosition, jointAxis=axis)
                self.jointNamesWithMotors.append(jointName)

                jointInfo = {'jointName':jointName,
                             'jointPosition':legJointPosition,
                             'parentOf':parentName,
                             'childOf':childName
                            }
                
                self.jointInfoDict[jointName] = jointInfo                

                legJointPosition[index] = legCubeSizeArray[j][index]

        # Add joint between Torso Link and 1st Leg Link at the end below here
        parentName = torsoCubeName
        childName = "Leg" + str(self.leg_id) + "Part0"
        jointName = parentName + "_" + childName
    
        legJointPosition = faces[k]
    
        axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) 
        pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=legJointPosition, jointAxis=axis)
        self.leg_id += 1
        self.jointNamesWithMotors.append(jointName)

    def Generate_Brain(self):
        name_id = 0
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        for sensorLinkName in self.linkNamesWithSensors:
            pyrosim.Send_Sensor_Neuron(name=name_id, linkName=sensorLinkName)            
            name_id += 1
        
        # breakpoint()
        
        for motorJointName in self.jointNamesWithMotors:
            pyrosim.Send_Motor_Neuron(name=name_id, jointName=motorJointName)
            name_id += 1

        for currentRow in range(self.num_sensors):
            for currentColumn in range(self.num_motors):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+self.num_sensors, weight=self.weights[currentRow][currentColumn])

        pyrosim.End()
