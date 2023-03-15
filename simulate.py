import sys

from simulation import SIMULATION

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]

simulation = SIMULATION(directOrGUI, solutionID)
fitness1 = simulation.Get_Fitness()
simulation.Run()
fitness2 = simulation.Get_Fitness()
simulation.Write_Fitness(fitness1,fitness2,solutionID)