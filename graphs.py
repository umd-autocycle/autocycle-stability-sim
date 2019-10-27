import matplotlib.pyplot as plt
import numpy as np


def plot_params(xtuple, *data):
    for datum in data:
        plt.plot(xtuple[0], datum[0])
    plt.legend([datum[1] for datum in data])
    plt.xlabel(xtuple[1])
    plt.show()


def f(t):
    return np.exp(-t) * np.cos(2*np.pi*t)


time = np.arange(0, 10, .01)
timeName = "time"
data = f(time)
dataName = "roll"
dataaa = 2*f(time)
data2 = (dataaa,"other")
xaxis = (time, timeName)
yaxis = (data, dataName)

plot_params(xaxis, yaxis, data2)