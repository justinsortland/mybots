import os
import matplotlib.pyplot as plt
import random
from parallelHillClimber import PARALLEL_HILL_CLIMBER

fitnessArrays = []
for i in range(10):
    
    # Set random seed value based on iteration of loop
    if i == 0:
        random.seed(32535634564)
    elif i == 1:
        random.seed(78234873940)
    elif i == 2:
        random.seed(3746574637)
    elif i == 3:
        random.seed(9854967965)
    elif i == 4:
        random.seed(3267724376234)
    elif i == 5:
        random.seed(6588675586)
    elif i == 6:
        random.seed(328742832)
    elif i == 7:
        random.seed(957685645)
    elif i == 8:
        random.seed(2378942837)
    else:
        random.seed(38754878437784)

    phc = PARALLEL_HILL_CLIMBER()
    phc.Evolve()

    os.system("rm body0.urdf")
    os.system("rm brain0.nndf")

    phc.Show_Best()

    fitness_array = phc.Get_Fitness_Array()
    fitnessArrays.append(fitness_array)


plt.plot(fitnessArrays[0],label='PHC #1')
plt.plot(fitnessArrays[1],label='PHC #2')
plt.plot(fitnessArrays[2],label='PHC #3')
plt.plot(fitnessArrays[3],label='PHC #4')
plt.plot(fitnessArrays[4],label='PHC #5')
plt.plot(fitnessArrays[5],label='PHC #6')
plt.plot(fitnessArrays[6],label='PHC #7')
plt.plot(fitnessArrays[7],label='PHC #8')
plt.plot(fitnessArrays[8],label='PHC #9')
plt.plot(fitnessArrays[9],label='PHC #10')


plt.xlabel('Number of Generations')
plt.ylabel('Fitness Value')
plt.legend()
plt.show()