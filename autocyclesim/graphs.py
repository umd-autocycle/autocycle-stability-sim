import matplotlib.pyplot as plt
import matplotlib
import numpy as np


def plot_params(title, xtuple, *data, dependent=""):
    matplotlib.rcParams['lines.linewidth'] = 2.5
    matplotlib.rc('font', family='Arial')

    for datum in data:
        plt.plot(xtuple[0], datum[0])
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

    for datum in data:
        plt.plot(xtuple[0], datum[0], figure = fig)
    plt.legend([datum[1] for datum in data], fontsize=14)
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

