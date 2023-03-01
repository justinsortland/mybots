import os
import random

from hillclimber import HILL_CLIMBER 
from parallelHillClimber import PARALLEL_HILL_CLIMBER
from solution import SOLUTION

# for i in range(5):
#     os.system("python3 generate.py")
#     os.system("python3 simulate.py")

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Plot_Fitness()
phc.Show_Best()

# solution = SOLUTION(0)
# solution.Start_Simulation("GUI")