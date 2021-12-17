import scipy.linalg

from bikemodel import MeijaardModel

from math import pi, radians, cos, sin
import pandas as pd
import numpy as np
from sysid import populate_experiments
from sysidutil import d4simulate
import matplotlib.pyplot as plt
from scipy.optimize import minimize, basinhopping, least_squares

from sysidutil import theta2matrices, matrices2theta

data_list = pd.read_csv('SYSID Data/index.tsv', sep='\t')

experiments = populate_experiments(data_list)
model = MeijaardModel(zss=False)

print(model.kalman_transition_matrix(4.65, 0.011, True))
print(model.kalman_controls_matrix(4.65, 0.011, True))


def direct4sid(yk, uk, alpha, n, dim, l):
    Y = np.zeros((alpha * l, n))
    for i in range(n):
        for j in range(alpha):
            Y[(l * j):(l * (j + 1)), i] = yk[:, i + j]
    print(Y)

    U = np.zeros((alpha, n))
    for i in range(n):
        for j in range(alpha):
            U[j, i] = uk[i + j]

    print()

    P = np.eye(n) - U.T @ np.linalg.inv(U @ U.T) @ U

    Q, S, V = np.linalg.svd(Y @ P)
    V = V.T
    Qs = Q[:, :dim]
    Ss = S[:dim]
    Vs = V[:, :dim]
    Qn = Q[:, dim:]
    Sn = S[dim:]
    Vn = V[:, dim:]
    # print(Qs)
    # print(Ss)
    # print(Vs)
    # print(Qs @ np.diag(np.sqrt(Ss)))
    Gamma = Qs  # @ np.diag(np.sqrt(Ss))
    print(Gamma.shape)
    C = Gamma[:l, :]
    A, residuals, rank, s = np.linalg.lstsq(Gamma[0:(alpha - 1), :], Gamma[1:alpha, :])
    print(residuals)
    Phi, residuals, rank, s = np.linalg.lstsq(Qn.T, Qn.T @ Y @ np.linalg.pinv(U))
    D = Phi[:l, 0]
    B = np.linalg.pinv(C) @ Phi[l:(2 * l), 0]
    print(A)
    print(B)
    print(C)
    print(D)
    print(C @ A)
    return A, B, C, D


def directbike4sid(ki, kf, q_ref, dq_ref, tk, vk, torque_k):
    # yk = q_ref[:, 1]  # delta
    # yk = dq_ref[:, 0]  # dphi
    yk = np.array([q_ref[:,0], q_ref[:, 1], dq_ref[:, 0], dq_ref[:, 1]])
    uk = torque_k

    alpha = 25
    n = 100
    dim = 4
    A, B, C, D = direct4sid(yk, uk, alpha, n, dim, 4)
    return A, B, C, D


A, B, C, D = directbike4sid(*experiments[0])
ki, kf, q_ref, dq_ref, tk, vk, torque_k = experiments[0]


x, y = d4simulate(A, B, C, D, q_ref, dq_ref, kf - ki, tk, vk, torque_k)


# plt.plot(tk, q_ref[:, 0], label='Phi (rad)')
plt.figure(0)
plt.plot(tk, q_ref[:, 0], label='Phi (rad)')
plt.plot(tk, y[0, :], label='Phi Fit Predicted (rad)')
plt.plot(tk, q_ref[:, 1], label='Del (rad)')
plt.plot(tk, y[1, :], label='Del Fit Predicted (rad)')
plt.legend()
plt.show()

plt.figure(1)
plt.plot(tk, dq_ref[:, 0], label='dPhi (rad)')
plt.plot(tk, y[2, :], label='dPhi Fit Predicted (rad)')
plt.plot(tk, dq_ref[:, 1], label='dDel (rad)')
plt.plot(tk, y[3, :], label='dDel Fit Predicted (rad)')

plt.legend()
plt.show()
