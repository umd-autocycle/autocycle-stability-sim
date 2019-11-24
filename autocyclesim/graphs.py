import matplotlib.pyplot as plt
import matplotlib
import numpy as np


def plot_params(title, xtuple, *data, dependent=""):
    matplotlib.rcParams['lines.linewidth'] = 2.5
    matplotlib.rc('font', family='Arial')

    n = 0
    for datum in data:
        if isinstance(datum[0], list):
            plt.plot(xtuple[0], datum[0])
        elif type(datum[0]) in (float, int):
            if datum[2] == "h":
                plt.axhline(y=datum[0], color = colorlist[n])
                n+=1
            elif datum[2] == "v":
                plt.axvline(x=datum[0], color = colorlist[n])
                n+=1
    n=0
    plt.legend([datum[1] for datum in data], fontsize=14)
    plt.xlabel(xtuple[1], fontsize=14)
    plt.ylabel(dependent, fontsize=14)
    plt.title(title, fontsize=20)

    ax = plt.subplot(111)
    ax.spines["top"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(False)

    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    plt.show()

def generate_figure(title, xtuple, *data, dependent=""):
    fig = plt.figure()
    matplotlib.rcParams['lines.linewidth'] = 2.5
    matplotlib.rc('font', family='Arial')

    colorlist = ["black", "red", "green", "yellow"]
    n = 0
    legendhandle = []
    legendlabel = []
    for datum in data:
        if isinstance(datum[0], list):
            temp, = plt.plot(xtuple[0], datum[0], figure = fig)
            legendhandle.append(temp)
        elif type(datum[0]) in (float, int):
            if datum[2] == "h":

                legendhandle.append(plt.axhline(y=datum[0], color = colorlist[n],ls='--',lw=1, figure = fig))
                n+=1
            elif datum[2] == "v":
                ymin, ymax = plt.gca().get_ylim()

                if(len(datum) > 3):
                    temp, = plt.plot((datum[0],datum[0]), (0,datum[3]), lw = 1, ls = '--')
                    legendhandle.append(temp)
                else:
                    legendhandle.append(plt.axvline(x=datum[0], color = colorlist[n],ls='--',lw=1, figure = fig))

                n+=1
            elif datum[2] == "ph":
                legendhandle.append(plt.axhline(y=datum[0], color = colorlist[n],ls='--',lw=1, figure = fig))
                plt.axhline(y= -datum[0], color = colorlist[n],ls='--',lw=1, figure = fig)
                n+=1
        legendlabel.append(datum[1])

    n=0
    plt.legend(legendhandle,legendlabel, fontsize=14)
    plt.xlabel(xtuple[1], fontsize=14, figure = fig)
    plt.ylabel(dependent, fontsize=14, figure = fig)
    plt.title(title, fontsize=20, figure = fig)

    ax = plt.subplot(111)
    ax.spines["top"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(False)

    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    plt.yticks(fontsize=12, figure = fig)
    plt.xticks(fontsize=12, figure = fig)

    return fig

