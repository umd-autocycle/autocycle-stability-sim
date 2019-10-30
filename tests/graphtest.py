import numpy as np
from autocyclesim.graphs import plot_params


def f(t):
    return np.exp(-t) * np.cos(2 * np.pi * t)


time = np.arange(0, 3, .01)
timeName = "Time"
data = f(time)
dataName = "roll"
dataaa = 2 * f(time)
data2 = (dataaa, "other")
xaxis = (time, timeName)
yaxis = (data, dataName)
plot_params("Roll Over Time", xaxis, yaxis, data2, dependent="Radians")
