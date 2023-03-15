from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim

import time
import numpy

import constants as c

class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        self.step = 0
        try:
            if self.directOrGUI == 'DIRECT':
                self.physicsClient = p.connect(p.DIRECT)
            elif self.directOrGUI == 'GUI':
                self.physicsClient = p.connect(p.GUI)
            else:
                raise ValueError('Incorect Mode! Please choose "DIRECT" or "GUI"')
            
        except Exception as error:
            print(error)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(c.gravityX, c.gravityY, c.gravityZ)

        self.world = WORLD(self.physicsClient)
        self.robot = ROBOT(solutionID)

    def Run(self):

        #c.rad = numpy.linspace(0, 2*numpy.pi, 1000)
        #backLegtargetAngles = numpy.zeros(c.steps)
        #frontLegtargetAngles = numpy.zeros(c.steps)
 
        for i in range(c.steps):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(self.robot.robot, i)            

            if self.directOrGUI == "GUI":
                time.sleep(c.sleepTime)

            # time.sleep(1/500)
            # print(i)
    
    def Get_Fitness(self):
        return self.robot.Get_Fitness()

    def Write_Fitness(self, fitness1, fitness2, id):
        self.robot.Write_Fitness(fitness1, fitness2, id)
    
    def __del__(self):
        p.disconnect()