import constants as c
import copy

from solution import SOLUTION

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()

    def Evolve(self, mode):
        self.parent.Evaluate(mode)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(mode)
    
    def Evolve_For_One_Generation(self, mode):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate(mode)
        self.Print()
        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Print(self):
        fitnessPrint = f"Parent: {self.parent.fitness}   Child: {self.child.fitness}"
        print(fitnessPrint)

    def Show_Best(self):
        pass
