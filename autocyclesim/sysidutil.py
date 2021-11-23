import numpy as np


def matrices2theta(M, C1, K0, K2):
    return np.array(
        [M[0, 0], M[0, 1], M[1, 1], K0[0, 0], K0[0, 1], K0[1, 1], K2[0, 1], K2[1, 1], C1[0, 1], C1[1, 0], C1[1, 1]])


def theta2matrices(theta):
    M = np.zeros((2, 2))
    K0 = np.zeros((2, 2))
    K2 = np.zeros((2, 2))
    C1 = np.zeros((2, 2))

    M[0, 0] = theta[0]
    M[0, 1] = theta[1]
    M[1, 0] = theta[1]
    M[1, 1] = theta[2]

    K0[0, 0] = theta[3]
    K0[0, 1] = theta[4]
    K0[1, 0] = theta[4]
    K0[1, 1] = theta[5]

    K2[0, 1] = theta[6]
    K2[1, 1] = theta[7]

    C1[0, 1] = theta[8]
    C1[1, 0] = theta[9]
    C1[1, 1] = theta[10]

    return M, C1, K0, K2
