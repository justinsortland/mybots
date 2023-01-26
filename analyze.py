import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("data/backLegSensorValues.npy", allow_pickle=True)
#print(backLegSensorValues)

frontLegSensorValues = numpy.load("data/frontLegSensorValues.npy", allow_pickle=True)
#print(frontLegSensorValues)

#matplotlib.pyplot.plot(backLegSensorValues, label='Back Leg Sensor Value', linewidth=4)
#matplotlib.pyplot.plot(frontLegSensorValues, label='Front Leg Sensor Value')



frontLegtargetAngles = numpy.load("data/frontLegtargetAngles.npy", allow_pickle=True)
backLegtargetAngles = numpy.load("data/backLegtargetAngles.npy", allow_pickle=True)

matplotlib.pyplot.plot(frontLegtargetAngles, label='Front leg motor values', linewidth=4)
matplotlib.pyplot.plot(backLegtargetAngles, label='Back leg motor values', linewidth=1)

matplotlib.pyplot.legend()
matplotlib.pyplot.show()