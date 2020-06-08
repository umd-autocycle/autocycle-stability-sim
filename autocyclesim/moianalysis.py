from scipy.fftpack import fft
from math import pi
import matplotlib.pyplot as plt
import numpy as np
import re

n = 3
tests = {
    "Back Frame": 3,
    "Back Wheel": 1,
    "Front Frame": 3,
    "Front Wheel": 1,
}

mois = {}


def moi_m(freq):
    T = 1 / freq
    G = 75e9
    d = 0.009525
    l = 1.055
    I_P = pi / 2 * ((d / 2) ** 4)

    return (T / (2 * pi)) ** 2 * (G * I_P / l)


if __name__ == '__main__':

    for component, num_tests in tests.items():
        mois[component] = []

        for position in range(1, num_tests + 1):
            mois[component].append([])

            for test_num in range(1, 4):
                with open('MOI-Data/%s/%s %d/%s %d-%d.txt' % (
                        component, component, position, component, position, test_num), 'r') as f:
                    x = f.readlines()

                x = [[int(z) for z in y.strip().split()] for y in x[:-1]]
                x = [[y[k] - y[k + 3] for y in x] for k in range(len(x[0]) // 2)]
                x = [[d - np.mean(y) for d in y] for y in x]

                N = len(x[0])
                T = .25
                fs = 1 / T

                y = fft(x[0])
                y = 2.0 / N * np.abs(y[1:N // 2])
                y[0:10] = 0
                xf = np.linspace(0.0, N // 2 - 1, N // 2)[1:]
                xf = fs / N * xf

                fig, ax = plt.subplots()
                ax.plot(xf, y)
                ax.set_title('%s %d-%d' % (component, position, test_num))
                ax.set_xlabel('Frequency (Hz)')
                ax.set_ylabel('Magnitude')

                mois[component][position - 1].append(xf[np.argmax(y)])

            mois[component][position - 1] = np.mean(mois[component][position - 1])

    print(mois)
