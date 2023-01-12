import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("data/backLegSensorValues.npy", allow_pickle=True)
print(backLegSensorValues)

matplotlib.pyplot.plot(backLegSensorValues)
matplotlib.pyplot.show()