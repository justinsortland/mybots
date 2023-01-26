from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim

import time
import numpy

import constants as c

class SIMULATION:
    def __init__(self, directOrGUI):
        self.world = WORLD()
        self.robot = ROBOT()

        self.directOrGUI = directOrGUI
        if self.directOrGUI == "DIRECT":
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)

    def Run(self):

        #c.rad = numpy.linspace(0, 2*numpy.pi, 1000)
        #backLegtargetAngles = numpy.zeros(c.steps)
        #frontLegtargetAngles = numpy.zeros(c.steps)
 
        for i in range(c.steps):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(self.robot.robot, i)            

            # time.sleep(1/500)
            # print(i)
    
    def Get_Fitness(self):
        self.robot.Get_Fitness()
    
    def __del__(self):
        p.disconnect()