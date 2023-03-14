# Final Project 

In this assignment, I used solution.py to generate the 3D creature from asssignments 7-8, which has the following:
1. Random number of links/cubes
2. Random number of sensor neurons, where cubes with sensor neurons are denoted by a green coloring, whereas cubes without sensors neurons are denoted by a blue coloring
3. Random number of motor neurons 

The code randomly generates a set of links and then picking from random links in that set of links, randomly generates another set of links branching off that link. The resulting creature's morphology typically resembles a random blob of cubes as shown below.

<p align="center">                                      
<img src="Images/3DCreature1.png" width=250 height=250>                     
</p>                                                  

# Teaser GIF
Here is a fun GIF showing how the creature moves.
<p align="center">                                      
<img src="Images/3DCreature1.png" width=250 height=250>                     
</p>  
                                                                              
# Brain and Body Generation
Everything in regards to both body and brain generation was randomized from the number of cubes to the number of sensors and motors for the creature. The creature first starts out at an initial position of [0,0,1] with a randomized size vector as well as the joint position being based on the randomized size vector. Then, the following properties were randomized:
* The number of torso links
* The number of leg links
* The number of legs
* The joint axis of the joints
* The size of each link
* The position of the center of the link (with respect to the joint connecting previous link to current link)
* The position of the next joint (with respect to the previous joint)
* The probability of the link having a sensor (75%)
* The probability of the link having a motor (85%)

Additionally, I have provided detailed diagrams of both brain and body generation below.

<p align="center">
<img src="Images/BodyGeneration.png" width=400 height=300>
</p>

<p align="center">
<img src="Images/BrainGeneration.png" width=400 height=300>
</p>

# Mutation and Evolution

There were three possible mutations that I implemented for my creature (including mutation combinations):

50% probability for brain mutation
* Altering one of the synapse weights
* Re-randomizing sensor placement throughout the creature

50% probability for body mutation
* 25% probability to add or remove a link:
  * 12.5% probability to add a link to an already existing link
  * 12.5% probability to remove an already existing link, making sure it has no other links after it or a leg branching out of it) 

Note that these probabilities are just arbitraily based on which one is most effective in evolution based on my observations.

Similarly to body and brain generation, I have included figures below detailing the processes of both mutation and evolution.

<p align="center">
<img src="Images/Mutation.png" width=400 height=300>
</p>

<p align="center">
<img src="Images/Evolution.png" width=400 height=300>
</p>

# Fitness Curve
Here is my graph containing my ten fitness curves, each one corresponding to a search.py run where I ran each simulation with a population size of 10 for 500 generations, each using a different seed:

<p align="center">
<img src="Images/FitnessCurve.png" width=400 height=300>
</p>

# Methods

# Constants

# YouTube Link
Here's a link to my video, which shows the difference between the random and evolved creature (which was cut for time to keep the video ~10 seconds):
https://youtu.be/0P5wL5jHgcE

# Limitations

# Things I've Tried

# Future Recommendations

# How To Run Simulation
In order to run one random simulation, run the following command:
```
python solution.py
```
or alternatively,
```
python3 solution.py
```

References:
- Northwestern University: CS 396 - AI Life
- Education in Evolutionary Robotics
- Pyrosim
- Ludobots (Karl Sim)
