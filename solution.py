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
        self.num_cubes = random.randint(3, 10)

        # self.linksWithSensors = []
        # for i in range(self.num_cubes):
        #     self.linksWithSensors.append(random.randint(0,1))

        # self.num_sensors = 0
        # for link in self.linksWithSensors:
        #     if self.linksWithSensors[link] == 1:
        #         self.num_sensors += 1

        # self.num_motors = self.num_cubes - 1 

        self.tree = self.Generate_Tree()

        self.num_sensors = sum(1 for segment in self.tree if segment['has_sensor'])
        self.num_motors = len(self.tree) - 1 

        self.weights = numpy.random.rand(self.num_sensors, self.num_motors)*3 - 1


    def Start_Simulation(self, mode):
        self.Create_World()
        self.Generate_Creature()
        self.Generate_Creature_Brain()

        os.system("python3 simulate.py " + mode + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(c.sleepTime)

        f = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(f.readlines()[0])
        f.close()

        os.system("rm fitness" + str(self.myID) + ".txt")

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)
        self.weights[randomRow][randomColumn] = random.random()*3 - 1

    def Set_ID(self, id):
        self.myID = id
        
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2, 2, 0.5] , size=[1, 1, 1], colorString='    <color rgba="0 1.0 1.0 1.0"/>', colorName='Grey')
        pyrosim.End()

    # def Generate_Snake(self):
    #     pyrosim.Start_URDF("body.urdf")

    #     cubePosX = 0
    #     cubePosY = 0
    #     cubePosZ = 1
    #     cubePos = [cubePosX, cubePosY, cubePosZ]

    #     jointPosX = 0.5
    #     jointPosY = 0
    #     jointPosZ = 0.5
    #     jointPos = [jointPosX, jointPosY, jointPosZ]

    #     cubeSizeX = 1
    #     cubeSizeY = 1
    #     cubeSizeZ = 1
    #     cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

    #     for i in range(self.num_cubes):
    #         cubeName = "Segment" + str(i) 
    #         if self.linksWithSensors[i] == 1:
    #             pyrosim.Send_Cube(name=cubeName, pos=cubePos, size=cubeSize, colorString="0 255.0 0 1.0", colorName='Green')
    #         else:
    #             pyrosim.Send_Cube(name=cubeName, pos=cubePos, size=cubeSize, colorString="0 0 255.0 1.0", colorName='Blue')

    #         if i != (self.num_cubes-1):
    #             parentName = cubeName
    #             childName = "Segment" + str(i+1)
    #             jointName = parentName + "_" + childName

    #             axis = str(random.uniform(-1, 1)) + " " + str(random.uniform(-1, 1)) + " " + str(random.uniform(-1, 1))
    #             pyrosim.Send_Joint(name=jointName, parent=parentName, child=childName, type="revolute", position=jointPos, jointAxis=axis)

    #             cubeSizeX = random.uniform(0.8, 1.2)
    #             cubeSizeY = random.uniform(0.8, 1.2)
    #             cubeSizeZ = random.uniform(0.8, 1.2)
    #             cubeSize = [cubeSizeX, cubeSizeY, cubeSizeZ]

    #             cubePosX = cubeSizeX/2
    #             jointPosX = cubeSizeX
    #             jointPos = [jointPosX, jointPosY, jointPosZ]

    #             cubePos = [cubePosX, cubePosY, cubePosZ]

    #     pyrosim.End()

    # def Generate_Snake_Brain(self):

    #     pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        # for i in range(self.num_sensors):
        #     sensorNeuronName = self.name_id
        #     sensorNeuronLinkName = "Segment" + str(self.name_id)
        #     pyrosim.Send_Sensor_Neuron(name=sensorNeuronName, linkName=sensorNeuronLinkName)
        #     self.name_id += 1

        # for i in range(self.num_motors):
        #     motorNeuronName = self.name_id
        #     motorNeuronJointName = "Segment" + str(i) + "_" + "Segment" + str(i+1)
        #     pyrosim.Send_Motor_Neuron(name=motorNeuronName, jointName=motorNeuronJointName)
        #     self.name_id += 1

        # for currentRow in range(self.num_sensors):
        #     for currentColumn in range(self.num_motors):
        #         pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+self.num_sensors, weight=self.weights[currentRow][currentColumn])

    #     pyrosim.End()

    def Generate_Tree(self):
        root = {'pos': [0,0,0], 'children': []}
        node_queue = [root]

        while node_queue:
            node = node_queue.pop(0)
            for i in range(random.randint(0,2)):
                child_pos = [random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)]
                node['children'].append({'pos':child_pos, 'has_sensor':random.randint(0,1), 'children': []})
                node_queue.append(node['children'][-1])

        pruned_tree = []
        for i,segment in enumerate(root['children']):
            if segment['has_sensor']:
                pruned_tree.append(segment)
                pruned_tree += segment['children']
        pruned_tree = [{'pos': root['pos'], 'has_sensor': False, 'children': pruned_tree}]

        return pruned_tree
    
    def Generate_Creature(self):
        pyrosim.Start_URDF("body.urdf")

        self.current_joint_id = 0
        self.current_link_id = 0

        self.Generate_Segment(self.tree[0], parent_pos=[0,0,0])

        pyrosim.End()

    def Generate_Segment(self, segment, parent_pos):
        link_name = f'segment_{self.current_link_id}'
        link_pos = [parent_pos[i] + segment['pos'][i] for i in range(3)]
        link_size = [random.uniform(0.8,1.2) for i in range(3)]
        link_color = 'Green' if segment['has_sensor'] else 'Blue'
        print(self.tree)
        pyrosim.Send_Cube(name=link_name, pos=link_pos, size=link_size, colorString='0 255.0 0 1.0', colorName=link_color)

        if parent_pos is not None:
            joint_name = f'joint_{self.current_joint_id}'
            joint_pos = [parent_pos[i] + segment['pos'][i] for i in range(3)]
            joint_axis = str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1)) + " " + str(random.uniform(-1,1))
            pyrosim.Send_Joint(name=joint_name, parent=f'segment_{self.current_link_id - 1}', child=f'segment_{self.current_link_id}', type="revolute", position=joint_pos, jointAxis=joint_axis)

    def Generate_Creature_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        sensor_neuron_names = []
        motor_neuron_names = []

        for i in range(self.num_sensors):
            sensor_neuron_name = self.name_id
            sensor_neuron_link_name = "Segment" + str(self.name_id)
            pyrosim.Send_Sensor_Neuron(name=sensor_neuron_name, linkName=sensor_neuron_link_name)
            sensor_neuron_names.append(sensor_neuron_name)
            self.name_id += 1

        for i in range(self.num_motors):
            motor_neuron_name = self.name_id
            motor_neuron_joint_name = "Segment" + str(i) + "_" + "Segment" + str(i+1)
            pyrosim.Send_Motor_Neuron(name=motor_neuron_name, jointName=motor_neuron_joint_name)
            motor_neuron_names.append(motor_neuron_name)
            self.name_id += 1

        for row, sensor_neuron_name in enumerate(sensor_neuron_names):
            for col, motor_neuron_name in enumerate(motor_neuron_names):
                pyrosim.Send_Synapse(sourceNeuronName=sensor_neuron_name, targetNeuronName=motor_neuron_name, weight=self.weights[row][col])

        pyrosim.End()
