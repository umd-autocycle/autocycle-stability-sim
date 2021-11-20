from bikemodel import MeijaardModel

from math import pi, radians, cos, sin
import pandas as pd
import numpy as np
import sys
import matplotlib.pyplot as plt
from scipy.optimize import minimize

ki = 96
kf = 172

data = pd.read_csv('11-16-2021-STAB - Sheet5.tsv', sep='\t')

# Marshal data
t = data['Time (s)'].to_numpy()
phi = data['Phi (rad)'].to_numpy()
delta = data['Del (rad)'].to_numpy()
dphi = data['dPhi (rad/s)'].to_numpy()
ddel = data['dDel (rad/s)'].to_numpy()
v = data['v (m/s)'].to_numpy()
torque = data['T (Nm)'].to_numpy()
dphi_y = data['dPhi_y (rad/s)'].to_numpy()
ddel_y = data['dDel_y (rad/s)'].to_numpy()

tk = t[ki:kf]
vk = v[ki:kf]
torque_k = torque[ki:kf]
q_ref = np.array([phi[ki:kf], delta[ki:kf]])
dq_ref = np.array([dphi_y[ki:kf], ddel_y[ki:kf]])

model = MeijaardModel(zss=False)


# Simulate
def simulate(M, C1, K0, K2):
    dq = np.zeros((kf - ki, 2))
    q = np.zeros((kf - ki, 2))
    q[0, 0], q[0, 1] = phi[ki], delta[ki]
    m_inv = np.linalg.inv(M)

    for k in range(1, kf - ki):
        dt = tk[k] - tk[k - 1]
        g = np.array([0, torque_k[k - 1]])
        dq[k] = dq[k - 1] + dt * m_inv @ (g - vk[k - 1] * C1 @ dq[k - 1] - (K0 + vk[k - 1] ** 2 * K2) @ q[k - 1])
        q[k] = q[k - 1] + dt * dq[k - 1]
    return q, dq


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


def error(theta):
    M, C1, K0, K2 = theta2matrices(theta)

    if abs(np.linalg.det(M)) < 0.00001:
        return 1000

    q, dq = simulate(M, C1, K0, K2)

    diff = dq - dq_ref.T
    cost = np.average(np.linalg.norm(diff, axis=0))
    return cost + 0.001 * np.linalg.norm(theta - theta0)


q_naive, dq_naive = simulate(model.m, model.c1, model.k0, model.k2)
theta0 = matrices2theta(model.m, model.c1, model.k0, model.k2)
bounds = [(0, None),
          (0, None),
          (0, None),
          (None, None),
          (None, None),
          (None, None),
          (0, None),
          (0, None),
          (None, None),
          (None, None),
          (None, None)]

min_res = minimize(error, theta0, method='Nelder-Mead', bounds=bounds, options={'disp': True, 'maxiter': 10000})
theta = min_res.x
print(min_res.fun)
print(model.m)
print(model.c1)
print(model.k0)
print(model.k2)
m, c1, k0, k2 = theta2matrices(theta)
print()
print(m.__repr__())
print(c1.__repr__())
print(k0.__repr__())
print(k2.__repr__())
q_fit, dq_fit = simulate(m, c1, k0, k2)

# Plot results
plt.figure()
plt.axhline(c='black')
plt.grid()
plt.plot(tk, q_ref[0, :], label='Phi (rad)')
plt.plot(tk, q_ref[1, :], label='Del (rad)')
plt.plot(tk, torque_k, label='Torque (Nm)')
plt.plot(tk, q_naive[:, 0], label='Phi Naively Predicted (rad)')
plt.plot(tk, q_naive[:, 1], label='Del Naively Predicted (rad)')
plt.plot(tk, q_fit[:, 0], label='Phi Fit Predicted (rad)')
plt.plot(tk, q_fit[:, 1], label='Del Fit Predicted (rad)')
plt.xlabel('Time (s)')
plt.title('Phi/Del Comparison')
plt.legend()
plt.show()

plt.figure()
plt.axhline(c='black')
plt.grid()
plt.plot(tk, dq_ref[0, :], label='dPhi (rad/s)')
plt.plot(tk, dq_ref[1, :], label='dDel (rad/s)')
plt.plot(tk, torque_k, label='Torque (Nm)')
plt.plot(tk, dq_naive[:, 0], label='dPhi Naively Predicted (rad/s)')
plt.plot(tk, dq_naive[:, 1], label='dDel Naively Predicted (rad/s)')
plt.plot(tk, dq_fit[:, 0], label='dPhi Fit Predicted (rad/s)')
plt.plot(tk, dq_fit[:, 1], label='dDel Fit Predicted (rad/s)')
plt.xlabel('Time (s)')
plt.title('dPhi/dDel Comparison')
plt.legend()
plt.show()
