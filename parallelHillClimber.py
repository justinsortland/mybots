import constants as c
import copy
import os
import numpy
import matplotlib.pyplot as plt

from solution import SOLUTION

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm body*.urdf")
        os.system("rm fitness*.txt")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

        self.fitnessArray = numpy.zeros((c.numberOfGenerations+1,c.populationSize))
        self.maxArray = numpy.zeros(c.numberOfGenerations+1)
        self.xvalues = numpy.zeros(c.numberOfGenerations+1)

        self.bestLastParent = None

    def Evolve(self):
        self.Evaluate(self.parents,1)
        for currentGeneration in range(c.numberOfGenerations):
            if currentGeneration == 0:
                for parent in self.parents.keys():
                    self.parents[parent].Start_Simulation("GUI",0)
                    break
            self.Evolve_For_One_Generation(currentGeneration,0)
    
    def Evolve_For_One_Generation(self, currentGeneration, flag):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children,flag)
        self.Print(currentGeneration)
        self.Select(currentGeneration)
        # self.Show_Child()

    def Spawn(self):
        self.children = {}
        for i in self.parents.keys():
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

        # self.child = copy.deepcopy(self.parent)  
        # self.child.Set_ID(self.nextAvailableID)
        # self.nextAvailableID += 1

    def Mutate(self):
        for i in self.children.keys():
            self.children[i].Mutate()
            self.children[i].leg_id = 0
            # self.children[i].Generate_Brain()

    def Print(self, currentGeneration):
        print("")
        for i in self.parents.keys():
            print("Parent: " + str(self.parents[i].fitness) + " Child: " + str(self.children[i].fitness))
            self.fitnessArray[currentGeneration,i] = self.parents[i].fitness
        print("")

    def Select(self, currentGeneration):
        for i in self.parents.keys():
            if self.parents[i].fitness < self.children[i].fitness:
                self.parents[i] = self.children[i]

            if currentGeneration == (c.numberOfGenerations-1):
                self.fitnessArray[currentGeneration+1,i] = self.parents[i].fitness

    def Show_Best(self):
        best_fitness = -1000.0
        best_parent = None
        for parent in self.parents.keys():
            if self.parents[parent].fitness > best_fitness:
                best_fitness = self.parents[parent].fitness
                best_parent = self.parents[parent]
        
        # self.bestLastParent = best_parent
        # with open("BestRobot.pickle", self.bestLastParent.fitness, "\n") as b:
        #     pickle.dump(self.bestLastParent, b, pickle.HIGHEST_PROTOCOL)

        best_parent.Start_Simulation("GUI",0)
        # self.parent.Evaluate("GUI")

    def Show_Child(self):
        for i in self.parents.keys():
            self.children[i].Start_Simulation("GUI",0)
            break

    def Evaluate(self, solutions, flag):
        for i in solutions.keys():
            solutions[i].Start_Simulation("DIRECT", flag)
            
        for i in solutions.keys():
            solutions[i].Wait_For_Simulation_To_End()

    def Get_Fitness_Array(self):
        for i in range(c.numberOfGenerations+1):
            maxFitness = max(self.fitnessArray[i])
            self.maxArray[i] = maxFitness
            self.xvalues[i] = i
        return self.maxArray
    
    def Get_Number_Of_Generations_Array(self):
        return self.xvalues