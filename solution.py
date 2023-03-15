import pyrosim.pyrosim as pyrosim
import random
import numpy
import os
import time
import constants as c

class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        self.segment_id = 0
        self.leg_id = 0
        
        self.num_torso_cubes = random.randint(2,3)
        self.numLeg1Segments = random.randint(1,2)
        self.numLeg2Segments = random.randint(1,2)
        self.total_cubes = self.num_torso_cubes + self.numLeg1Segments + self.numLeg2Segments

        self.linksWithLegs = [1,1]
        for i in range(self.num_torso_cubes):
            self.linksWithLegs.append(0)
        while self.linksWithLegs[0] == 1 or self.linksWithLegs[-1] == 1:
            random.shuffle(self.linksWithLegs)
        for i,link in enumerate(self.linksWithLegs):
            if i != (len(self.linksWithLegs)-1) and i != 0:
                if self.linksWithLegs[i] == 1 and self.linksWithLegs[i+1] == 1:
                    random.shuffle(self.linksWithLegs)

        if self.total_cubes < 2:
            self.total_cubes = 2
                
        self.num_motors = self.total_cubes - 1 

        self.jointNamesWithMotors = []

        self.cubeInfoDict = {}
        self.jointInfoDict = {}

        self.num_sensors = 0

    def Start_Simulation(self, mode, flag):
        self.Generate_World()
        self.leg_id = 0
        if flag == 1:
            self.Generate_Morphology_Body()
        self.Generate_Body()
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
        mutations = ['Mutate Weights', 'Mutate Body', 'Mutate Brain', 'Mutate Body and Brain', 'Mutate Weights and Body']
        mutation = random.choice(mutations)
        if mutation == 'Mutate Weights':
            self.Mutate_Weights()
        elif mutation == 'Mutate Body':
            self.Mutate_Body()
        elif mutation == 'Mutate Brain':
            self.Mutate_Brain
        elif mutation == 'Mutate Weights and Body':
            self.Mutate_Weights()
            self.Mutate_Body()
        elif mutation == 'Mutate Body and Brain':
            self.Mutate_Body()
            self.Mutate_Brain()

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

    def Mutate_Brain(self):
        print("Tried to mutate sensors")

        self.num_sensors = 0
        for cube in self.cubeInfoDict:
            self.cubeInfoDict[cube]['hasSensor'] = random.choice([True,False])
            if self.cubeInfoDict[cube]['hasSensor'] == True:
                self.num_sensors += 1

        if self.num_sensors == 0:
            self.num_sensors = 1
            randomCube = random.choice(list(self.cubeInfoDict.keys()))
            self.cubeInfoDict[randomCube]['hasSensor'] = True
        
        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1

    def Mutate_Body(self):
        mutations = ['Add Link', 'Remove Link']
        mutation = random.choice(mutations)
        directions = []
        for cube in self.cubeInfoDict:
            
            # Make sure that adding or remove link with no joints or links after it
            if self.cubeInfoDict[cube]['jointAfter'] == None and self.cubeInfoDict[cube]['cubeAfter'] == None:
                directions.append(cube)

        chosenCube = random.choice(directions)

        if chosenCube[0:4] == 'Leg0' and self.numLeg1Segments == 3:
            mutation = 'Remove Link'

        if chosenCube[0:4] == 'Leg1' and self.numLeg2Segments == 3:
            mutation = 'Remove Link'

        if self.cubeInfoDict[chosenCube]['hasLeg'] == False and self.total_cubes > 7:
            mutation = 'Remove Link'

        # Make sure that not removing root link
        if chosenCube == 'Torso0':
            mutation = 'Add Link'

        # Make sure at least two links exist, so at least one joint exists in joint dictionary
        if len(self.cubeInfoDict) == 2 and 'Torso0' in self.cubeInfoDict and 'Torso1' in self.cubeInfoDict:
            mutation = 'Add Link'

        # Make sure not removing link that has leg branching out of it
        if self.cubeInfoDict[chosenCube]['hasLeg'] == True:
            mutation = 'Add Link'
        
        if mutation == 'Add Link':

            # Increment counters for number of links
            self.num_torso_cubes += 1
            self.total_cubes += 1
            self.num_motors += 1
            print("Tried to add link")

            # Check if chosen cube is a torso cube or a leg cube
            if chosenCube[0:5] == "Torso":
                # Initialize info about cube that will be sent to Send_Cube() 
                chosenCubeIndex = int(chosenCube[5])
                newCubeIndex = str(chosenCubeIndex+1)
                torsoCubeName = "Torso" + newCubeIndex
                print(f"Tried to add {torsoCubeName}")

                torsoCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]
                torsoCubePosition = [torsoCubeSize[0]/2.6,0,0]
                # Add information about cube into cube dictionary 
                cubeInfo = {'cubeName':torsoCubeName,
                            'cubePosition':torsoCubePosition[:],
                            'cubeSize':torsoCubeSize,
                            'hasSensor':random.choice([True,False]),
                            'jointBefore':None,
                            'jointAfter':None,
                            'cubeBefore':chosenCube,
                            'cubeAfter':None,
                            'direction':"+x",
                            'hasLeg':random.choice([True,False]) 
                        }                      
                self.cubeInfoDict[torsoCubeName] = cubeInfo    

                # for cube in self.cubeInfoDict:
                #     if self.cubeInfoDict[cube]['hasLeg'] == True:
                #         self.cubeInfoDict[torsoCubeName]['hasLeg'] = random.choice([True,False])

                # Check if link has sensor or not and update number of sensors accordingly
                if self.cubeInfoDict[torsoCubeName]['hasSensor'] == True:
                    self.num_sensors += 1

                # Add joint that connects chosen cube to new cube
                parentName = chosenCube
                childName = torsoCubeName
                jointName = parentName + "_" + childName
                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))    
                torsoJointPosition = [(self.cubeInfoDict[chosenCube]['cubeSize'][0]/2.6),self.cubeInfoDict[chosenCube]['cubePosition'][1],self.cubeInfoDict[chosenCube]['cubePosition'][2]]              
                self.jointNamesWithMotors.append(jointName)

                # Add information about joint to joint dictionary
                jointInfo = {'jointName':jointName,
                             'jointPosition':torsoJointPosition[:],
                             'hasParent':parentName,
                             'hasChild':childName,
                             'axis':axis,
                             'direction':"+x"
                            }     
                self.jointInfoDict[jointName] = jointInfo

                # Assign joint as parent of added cube and child of cube we chose to add to
                self.cubeInfoDict[torsoCubeName]['jointBefore'] = self.jointInfoDict[jointName]['jointName']
                self.cubeInfoDict[chosenCube]['jointAfter'] = self.jointInfoDict[jointName]['jointName']

                # Assign added cube as cube after chosen cube
                self.cubeInfoDict[chosenCube]['cubeAfter'] = torsoCubeName
            else:
                print(f"Tried to add leg link ({chosenCube})")
                # Get direction of cube
                cubeDirection = self.cubeInfoDict[chosenCube]['direction']

                # Find out from which leg the link was chosen
                chosenLegIndex = int(chosenCube[3])

                # Find out which link from said leg was chosen
                chosenLegLink = chosenCube[8]

                # Initialize info about cube that will be sent to Send_Cube()
                newLinkIndex = int(chosenLegLink) + 1
                legCubeName = "Leg" + str(chosenLegIndex) + "Part" + str(newLinkIndex)
                legCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]

                # Set the position of the leg link based on the direction of the chosen block
                if cubeDirection == "+y":
                    legCubePosition = [0,legCubeSize[1]/2.6,0]
                else:
                    legCubePosition = [0,0,legCubeSize[2]/2.6]

                # Add information about cube into cube dictionary
                cubeInfo = {'cubeName':legCubeName,
                            'cubePosition':legCubePosition,
                            'cubeSize':legCubeSize,
                            'hasSensor':random.choice([True,False]),
                            'jointBefore':None,
                            'jointAfter':None,
                            'cubeBefore':chosenCube,
                            'cubeAfter':None,
                            'direction':cubeDirection,
                            'hasLeg':False
                           }                      
                self.cubeInfoDict[legCubeName] = cubeInfo
 
                # Check if link has sensor or not
                if self.cubeInfoDict[legCubeName]['hasSensor'] == True:
                    self.num_sensors += 1
                    if chosenLegIndex == 0:
                        self.numLeg1Segments += 1
                    else:
                        self.numLeg2Segments += 1  

                # Add joint that connects chosen cube to new cube
                parentName = chosenCube
                childName = legCubeName
                jointName = parentName + "_" + childName
                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))   

                if cubeDirection == "+y":
                    torsoJointPosition = [self.cubeInfoDict[chosenCube]['cubePosition'][0], (self.cubeInfoDict[chosenCube]['cubeSize'][1]/2.6), self.cubeInfoDict[chosenCube]['cubePosition'][2]]        
                else:
                    torsoJointPosition = [self.cubeInfoDict[chosenCube]['cubePosition'][0], self.cubeInfoDict[chosenCube]['cubePosition'][1], (self.cubeInfoDict[chosenCube]['cubeSize'][2]/2.6)]        

                self.jointNamesWithMotors.append(jointName)

                # Add information about joint to joint dictionary
                jointInfo = {'jointName':jointName,
                             'jointPosition':torsoJointPosition,
                             'hasParent':parentName,
                             'hasChild':childName,
                             'axis':axis,
                             'direction':cubeDirection
                            }     
                self.jointInfoDict[jointName] = jointInfo  

                # Assign joint as parent to added cube and child to cube we chose to add to
                self.cubeInfoDict[legCubeName]['jointBefore'] = self.jointInfoDict[jointName]['jointName']
                self.cubeInfoDict[chosenCube]['jointAfter'] = self.jointInfoDict[jointName]['jointName']

                # Assign added cube as cube after chosen cube
                self.cubeInfoDict[chosenCube]['cubeAfter'] = legCubeName
        else:
            print(f"Tried to remove {chosenCube}")
            # Decrement counter for total number of links and number of motors (which depends on total number of links )
            self.total_cubes -= 1
            self.num_motors -= 1

            # Access the joint that is before the cube that you want to remove
            for joint in self.jointInfoDict:
                if self.jointInfoDict[joint]['hasChild'] == chosenCube:
                    jointBeforeChosenCube = self.jointInfoDict[joint]['jointName']

            # Access the cube that is before chosen cube and set jointAfter to None
            for cube in self.cubeInfoDict:
                if jointBeforeChosenCube in self.jointInfoDict:
                    if cube == self.jointInfoDict[jointBeforeChosenCube]['hasParent']:
                        cubeBefore = cube
            self.cubeInfoDict[cubeBefore]['jointAfter'] = None
            self.cubeInfoDict[cubeBefore]['cubeAfter'] = None

            # Check if torso cube has a sensor and decrement number of sensors if so
            if self.cubeInfoDict[chosenCube]['hasSensor'] == True:
                self.num_sensors -= 1
            
            # Remove link from arrays and decrement link counters
            if chosenCube[0:5] == "Torso":
                # Decrememt number of torso links
                self.num_torso_cubes -= 1
            else:
                # Check if removing leg 0 link or leg 1 link
                if chosenCube[3] == "0":
                    # Decrement number of leg 0 links
                    self.numLeg1Segments -= 1 
                else:
                    # Decrement number of leg 1 links
                    self.numLeg2Segments -= 1

            # Remove cube and preceding joint from dictionaries
            self.jointNamesWithMotors.remove(jointBeforeChosenCube)
            del self.jointInfoDict[jointBeforeChosenCube]
            del self.cubeInfoDict[chosenCube]
        
        # Update weights matrix to reflect the change in number of sensors and motors after adding or removing link
        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1  

    def Set_ID(self, id):
        self.myID = id
    
    def Generate_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[6, 6, 0.5] , size=[1, 1, 1], colorString='    <color rgba="0 1.0 1.0 1.0"/>', colorName='Grey')
        pyrosim.End()

    def Generate_Morphology_Body(self):      

        # Initial cube information

        torsoCubePosition = [0,0,1]
        # torsoJointPosition = [0.5,0,2] 

        # TRY MAKING ROOT LINK ALSO RANDOM SIZED WHEN YOU WAKE UP IN THE MORNING! LESS CUBES THE BETTERR
        torsoCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]
        torsoJointPosition = [torsoCubeSize[0]/2.6,0,1]
        torsoCubeSizeArray = []

        # For loop for generating torso cubes
        for i in range(self.num_torso_cubes):
            torsoCubeName = "Torso" + str(i)

            # raise ValueError(torsoCubePosition)

            # Add information about cube into cube dictionary
            cubeInfo = {'cubeName':torsoCubeName,
                        'cubePosition':torsoCubePosition,
                        'cubeSize':torsoCubeSize,
                        'hasSensor':random.choice([True,False]),
                        'jointBefore':None,
                        'jointAfter':None,
                        'cubeBefore':None,
                        'cubeAfter':None,
                        'direction':"+x",
                        'hasLeg': random.choice([True,False])
                       }                      
            self.cubeInfoDict[torsoCubeName] = cubeInfo

            # Use helper function to generating leg cubes and then respective leg joints
            if self.cubeInfoDict[torsoCubeName]['hasLeg'] == True:
                self.Generate_Morphology_Leg(self.cubeInfoDict[torsoCubeName]['cubePosition'], self.cubeInfoDict[torsoCubeName]['cubeSize'], self.cubeInfoDict[torsoCubeName]['cubeName'], self.leg_id)
                        
            # Initialize info for cube and joint that will be appended in next iteration of for loop
            if i != (self.num_torso_cubes-1):
                torsoCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]
                torsoCubeSizeArray.append(torsoCubeSize)
                torsoCubePosition = [torsoCubeSize[0]/2.6,0,0]

        # For loop for generating joints
        for j in range(self.num_torso_cubes):
            if j != (self.num_torso_cubes-1):

                # Send joint
                parentName = "Torso" + str(j)
                childName = "Torso" + str(j+1)
                jointName = parentName + "_" + childName

                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))                
                self.jointNamesWithMotors.append(jointName)

                # Add information about joint into joint dictionary
                jointInfo = {'jointName':jointName,
                             'jointPosition':torsoJointPosition,
                             'hasParent':parentName,
                             'hasChild':childName,
                             'axis':axis,
                             'direction':"+x"
                            }     
                self.jointInfoDict[jointName] = jointInfo 

                self.cubeInfoDict[parentName]['jointAfter'] = jointName
                self.cubeInfoDict[childName]['jointBefore'] = jointName

                self.cubeInfoDict[parentName]['cubeAfter'] = childName
                self.cubeInfoDict[childName]['cubeBefore'] = parentName

                torsoJointPosition = [torsoCubeSizeArray[j][0],0,0]

                # Set torso joint position for next iteration based on previous cube size
                # for cube in self.cubeInfoDict:
                #     if cube == childName:
                        
                #         # AFTER EXAM FIND POSITION OF JOINT BEFORE CUBE = CHILDNAME AND FIX TORSOJOINTPOSITION WITH RESPECT TO THAT

                #         torsoJointPosition[0] = self.cubeInfoDict[cube]['cubeSize'][0]
                #         torsoJointPosition[2] = 0

                
         
    def Generate_Morphology_Leg(self, torsoCubePosition, torsoCubeSize, torsoCubeName, legID):

        # Determine which leg adding link to
        if legID == 0:
            numLegSegments = self.numLeg1Segments
            # legLinksWithSensors = self.leg1LinksWithSensors
        else:
            numLegSegments = self.numLeg2Segments
            # legLinksWithSensors = self.leg2LinksWithSensors

        # Randomly pick a face of the torsco cube to branch out from
        directions = [[torsoCubePosition[0], torsoCubePosition[1] + (torsoCubeSize[1]/2.6), torsoCubePosition[2]],
                      [torsoCubePosition[0], torsoCubePosition[1], torsoCubePosition[2] + (torsoCubeSize[2]/2.6)]]
        chosenDirection = random.choice(directions)
        k = directions.index(chosenDirection)

        # Determine index of position that will be updated based on face chosen
        index = 1 if k == 0 else 2

        legCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]
        legCubeSizeArray = []

         # Initialize information about first cube of leg
        legCubePosition = [0, (legCubeSize[1])/2.6, 0] if index == 1 else [0, 0, (legCubeSize[2])/2.6]

        # Determine cube direction based on chosen direction
        if index == 1:
            cubeDirection = "+y"
        else:
            cubeDirection = "+z"            

        # For loop for generating cubes
        for i in range(numLegSegments):
            legCubeName = "Leg" + str(self.leg_id) + "Part" + str(i)

            # Add information about cube into cube dictionary
            cubeInfo = {'cubeName':legCubeName,
                        'cubePosition':legCubePosition,
                        'cubeSize':legCubeSize,
                        'hasSensor':random.choice([True,False]),
                        'jointBefore':None,
                        'jointAfter':None,
                        'cubeBefore':None,
                        'cubeAfter':None,
                        'direction':cubeDirection,
                        'hasLeg':False
                        }
            self.cubeInfoDict[legCubeName] = cubeInfo

            # Prepare information for next cube to be added, as long as not the last iteration of loop
            if i != (numLegSegments-1):
                legCubeSize = [random.uniform(0.25,1),random.uniform(0.25,1),random.uniform(0.25,1)]
                legCubeSizeArray.append(legCubeSize)
                if index == 2:
                    legCubePosition = [0,0,(legCubeSize[index]/2.6)]
                else:
                    legCubePosition = [0,(legCubeSize[index]/2.6),0]

        legJointPosition = [0,torsoCubeSize[1],0] if index == 1 else [0,0,torsoCubeSize[2]]

        # For loop for generating joints
        for j in range(numLegSegments):
            if j != (numLegSegments-1):
                parentName = "Leg" + str(self.leg_id) + "Part" + str(j)
                childName = "Leg" + str(self.leg_id) + "Part" + str(j+1)
                jointName = parentName + "_" + childName
                axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))                
                self.jointNamesWithMotors.append(jointName)
                jointInfo = {'jointName':jointName,
                             'jointPosition':legJointPosition,
                             'hasParent':parentName,
                             'hasChild':childName,
                             'axis':axis,
                             'direction':cubeDirection
                            }
                self.jointInfoDict[jointName] = jointInfo   

                self.cubeInfoDict[parentName]['jointAfter'] = jointName
                self.cubeInfoDict[childName]['jointBefore'] = jointName

                self.cubeInfoDict[parentName]['cubeAfter'] = childName
                self.cubeInfoDict[childName]['cubeBefore'] = parentName

                # Set leg joint position for next iteration based on previous cube size
                if index == 2:
                    legJointPosition = [0,0,(legCubeSizeArray[j][index])]
                else:
                    legJointPosition = [0,(legCubeSizeArray[j][index]),0]

                # for cube in self.cubeInfoDict:
                #     if cube == parentName and self.cubeInfoDict[cube]['direction'] == cubeDirection:
                #         legJointPosition[index] = self.cubeInfoDict[cube]['cubeSize'][index] 
                #         legJointPosition[0] = 0

        # Add joint between Torso Link and 1st Leg Link below here
        parentName = torsoCubeName
        childName = "Leg" + str(self.leg_id) + "Part0"
        jointName = parentName + "_" + childName
        legJointPosition = [torsoCubePosition[0],(torsoCubeSize[1]/2.6),torsoCubePosition[2]] if index == 1 else [torsoCubePosition[0],torsoCubePosition[1],(torsoCubeSize[2]/2.6)]
        axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) 
        self.leg_id += 1
        self.jointNamesWithMotors.append(jointName)

        # Add information about joint into joint dictionary
        jointInfo = {'jointName':jointName,
                     'jointPosition':legJointPosition,
                     'hasParent':parentName,
                     'hasChild':childName,
                     'axis':axis,
                     'direction':cubeDirection
                    }
        self.jointInfoDict[jointName] = jointInfo 
        self.cubeInfoDict[childName]['jointBefore'] = jointName             

    def Generate_Body(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        for joint in self.jointInfoDict: 
            if joint[0:5] == 'Torso' and joint[7:12] == 'Torso':
                parentName = joint[0:6] 
                childName = joint[7:13]
                self.cubeInfoDict[parentName]['cubeAfter'] = childName
                self.cubeInfoDict[parentName]['jointAfter'] = joint

                self.cubeInfoDict[childName]['cubeBefore'] = parentName
                self.cubeInfoDict[childName]['jointBefore'] = joint

            elif joint[0:5] == 'Torso' and joint[7:10] == 'Leg':
                parentName = joint[0:6]
                childName = joint[7:16]
                self.cubeInfoDict[parentName]['cubeAfter'] = childName
                self.cubeInfoDict[childName]['cubeBefore'] = parentName

                self.cubeInfoDict[childName]['cubeBefore'] = parentName
                self.cubeInfoDict[childName]['jointBefore'] = joint

        # print(self.cubeInfoDict)
        # print("---------")
        # print(self.jointInfoDict)

        # Iterate over cube dictionary and send cubes based on dictionary information
        for cube in self.cubeInfoDict:
            cubeName = self.cubeInfoDict[cube]['cubeName']
            cubePosition = self.cubeInfoDict[cube]['cubePosition']
            cubeSize = self.cubeInfoDict[cube]['cubeSize']
            colorRGB = "0 255.0 0 1.0" if self.cubeInfoDict[cube]['hasSensor'] == True else "0 0 255.0 1.0"
            colorName = 'Green' if self.cubeInfoDict[cube]['hasSensor'] == True else 'Blue'
            pyrosim.Send_Cube(name=cubeName, pos=cubePosition, size=cubeSize, colorString=colorRGB, colorName=colorName)

        # Iterate over joint dictionary and send joints based on dictionary information
        for joint in self.jointInfoDict:
            jointName = self.jointInfoDict[joint]['jointName']
            jointParent = self.jointInfoDict[joint]['hasParent']
            jointChild = self.jointInfoDict[joint]['hasChild']
            jointPosition = self.jointInfoDict[joint]['jointPosition']
            axis = self.jointInfoDict[joint]['axis']
            pyrosim.Send_Joint(name=jointName, parent=jointParent, child=jointChild, type="revolute", position=jointPosition, jointAxis=axis)

        for cube in self.cubeInfoDict:
            if self.cubeInfoDict[cube]['hasSensor'] == True:
                self.num_sensors += 1

        if self.num_sensors == 0:
            self.num_sensors = 1

        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1

        pyrosim.End()

    def Generate_Brain(self):
        name_id = 0
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        for cube in self.cubeInfoDict:
            if self.cubeInfoDict[cube]['hasSensor'] == True:
                pyrosim.Send_Sensor_Neuron(name=name_id, linkName=cube)            
                name_id += 1        

        for motorJointName in self.jointNamesWithMotors:
            pyrosim.Send_Motor_Neuron(name=name_id, jointName=motorJointName)
            name_id += 1

        for currentRow in range(self.num_sensors):
            for currentColumn in range(self.num_motors):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+self.num_sensors, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()