# mybots

My fitness function is located in the Get_Fitness() function of robot.py.
Here is how it works:

```
    def Get_Fitness(self, id):
        z_coordinate_values = []
        for i in range(0, 1000, 10):
            basePositionAndOrientation = p.getBasePositionAndOrientation(self.robot)
            basePosition = basePositionAndOrientation[0]
            zPosition = basePosition[2]
            z_coordinate_values.append(zPosition)
 
        average_z_coordinate = sum(z_coordinate_values) / len(z_coordinate_values)

        f = open("tmp" + str(id) + ".txt", "w")
        f.write(str(average_z_coordinate))
        f.close()
        os.system("mv tmp" + str(id) + ".txt " + "fitness" + str(id) + ".txt")
 ```
 
 First, I run iterate through the 1000 steps, and access the z-coordinate of the base position at each step.
 
 Next, I append each z-coordinate to an array of z-coordinate values and then find the average of those
 coordinates and use that as my fitness value, which I write to the file.
