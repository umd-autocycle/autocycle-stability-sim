import numpy as np
from math import sqrt
from scipy.linalg import qr, pinv


class HomeBrewKalmanFilter:
    def __init__(self, dim_x=4, dim_z=4, dim_u=2):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros((dim_x, 1), dtype='float32')
        self.P_2 = np.identity(dim_x, dtype='float32')

        self.F = np.zeros((dim_x, dim_x), dtype='float32')
        self.B = np.zeros((dim_x, dim_u), dtype='float32')
        self.H = np.zeros((dim_z, dim_x), dtype='float32')

        self.Q_2 = np.zeros((dim_x, dim_x), dtype='float32')
        self.R_2 = np.zeros((dim_z, dim_z), dtype='float32')

        self.M = np.zeros((dim_x + dim_z, dim_x + dim_z))

    def predict(self, u):
        self.x = self.F @ self.x + self.B @ u

        _, P_2 = qr(np.hstack([self.F @ self.P_2, self.Q_2]).T)
        self.P_2 = P_2[:self.dim_x, :self.dim_x].T

    def update(self, y):
        residual = y - self.H @ self.x

        self.M[0:self.dim_z, 0:self.dim_z] = self.R_2.T
        self.M[self.dim_z:, 0:self.dim_z] = (self.H @ self.P_2).T
        self.M[self.dim_z:, self.dim_z:] = self.P_2.T

        _, S = qr(self.M)
        K = S[0:self.dim_z, self.dim_z:].T
        N = S[0:self.dim_z, 0:self.dim_z].T

        self.x += K @ pinv(N) @ residual

        self.P_2 = S[self.dim_z:, self.dim_z:].T


def choleskyFromW(w, dtype):
    dim = w.shape[0]
    L = np.zeros((dim, dim), dtype=dtype)

    for i in range(dim):
        for j in range(i + 1):
            sum = 0.0

            for k in range(j):
                sum += L[i, k] * L[j, k]

            if i == j:
                L[i, j] = np.sqrt(w[i, 0] * w[i, 0] - sum)
            else:
                L[i, j] = (1.0 / L[j][j] * (w[i, 0] * w[j, 0] - sum))

    return L
