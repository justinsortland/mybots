import os

from hillclimber import HILL_CLIMBER 

# for i in range(5):
#     os.system("python3 generate.py")
#     os.system("python3 simulate.py")

mode = "DIRECT"
hc = HILL_CLIMBER()
hc.Evolve(mode)
hc.Show_Best()