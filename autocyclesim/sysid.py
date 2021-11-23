from bikemodel import MeijaardModel

from math import pi, radians, cos, sin
import pandas as pd
import numpy as np
import sys
import matplotlib.pyplot as plt
from scipy.optimize import minimize, basinhopping, least_squares

from sysidutil import theta2matrices, matrices2theta

data_list = pd.read_csv('SYSID Data/index.tsv', sep='\t')


def populate_experiments(dl):
    exps = []

    for index, row in dl.iterrows():
        ki = row['ki']
        kf = row['kf']
        filename = row['Filename']

        data = pd.read_csv('SYSID Data/%s' % filename, sep='\t')

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
        q_ref = np.array([phi[ki:kf], delta[ki:kf]]).T
        dq_ref = np.array([dphi_y[ki:kf], ddel_y[ki:kf]]).T

        experiment = ki, kf, q_ref, dq_ref, tk, vk, torque_k

        exps.append(experiment)

    return exps


experiments = populate_experiments(data_list)
model = MeijaardModel(zss=False)


# Simulate
def simulate(m, c1, k0, k2, q_ref, dq_ref, n, tk, vk, torque_k):
    dq = np.zeros((n, 2))
    q = np.zeros((n, 2))
    q[0, 0], q[0, 1] = q_ref[0, 0], q_ref[0, 1]
    dq[0, 0], dq[0, 1] = dq_ref[0, 0], dq_ref[0, 1]
    m_inv = np.linalg.inv(m)

    for k in range(1, n):
        dt = tk[k] - tk[k - 1]
        g = np.array([0, torque_k[k - 1]])
        dq[k] = dq[k - 1] + dt * m_inv @ (g - vk[k - 1] * c1 @ dq[k - 1] - (k0 + vk[k - 1] ** 2 * k2) @ q[k - 1])
        q[k] = q[k - 1] + dt * dq[k - 1]
    return q, dq


def error(theta):
    m, c1, k0, k2 = theta2matrices(theta)

    if abs(np.linalg.det(m)) < 0.00001:
        return 1000

    cost = 0
    residuals = np.zeros((1,))
    for experiment, i in zip(experiments, range(len(experiments))):
        ki, kf, q_ref, dq_ref, tk, vk, torque_k = experiment

        q, dq = simulate(m, c1, k0, k2, q_ref, dq_ref, kf - ki, tk, vk, torque_k)

        diff = dq - dq_ref
        diff2 = q[:, 1] - q_ref[:, 1]  # Adding in comparison to delta directly

        # cost += (np.sum((diff * 10) ** 2) + np.sum((diff2 * 10) ** 2))
        residuals = np.concatenate((residuals, diff, diff2), axis=None)

    # cost = np.average(costs)

    # return cost + 0.001 * np.linalg.norm(theta - theta0)
    return residuals


mf = np.array([[26.67504339, 1.21856943],
               [1.21856943, 0.59438128]])
c1f = np.array([[0., 4.97370516],
                [-0.98015773, 2.43085255]])
k0f = np.array([[-210.6481775, 1.14387605],
                [1.14387605, 3.2817143]])
k2f = np.array([[0., 21.88145723],
                [0., -0.86196881]])

# theta0 = matrices2theta(mf, c1f, k0f, k2f)
theta0 = matrices2theta(model.m, model.c1, model.k0, model.k2)
bounds = np.array([(0, np.inf),
                   (0, np.inf),
                   (0, np.inf),
                   # (np.NINF, 0),
                   (-300, 0),
                   (np.NINF, np.inf),
                   (np.NINF, np.inf),
                   (0, np.inf),
                   (0, np.inf),
                   (np.NINF, np.inf),
                   (np.NINF, np.inf),
                   (np.NINF, np.inf)])

# min_res = minimize(error, theta0, method='Nelder-Mead',
#                    options={'disp': True, 'maxiter': 10000, 'adaptive': True, 'bounds': bounds})
min_res = least_squares(error, theta0, bounds=bounds.T, tr_options={'regularize': True})
theta_fit = min_res.x
print(min_res.cost)
print(model.m)
print(model.c1)
print(model.k0)
print(model.k2)
M, C1, K0, K2 = theta2matrices(theta_fit)
print()
print(M.__repr__())
print(C1.__repr__())
print(K0.__repr__())
print(K2.__repr__())


def plot_results():
    m = len(experiments)
    f, plts = plt.subplots(2, m)

    for experiment, i in zip(experiments, range(m)):
        ki, kf, q_ref, dq_ref, tk, vk, torque_k = experiment
        # q_naive, dq_naive = simulate(mf, c1f, k0f, k2f, q_ref, dq_ref, kf - ki, tk, vk, torque_k)
        q_naive, dq_naive = simulate(model.m, model.c1, model.k0, model.k2, q_ref, dq_ref, kf - ki, tk, vk, torque_k)
        q_fit, dq_fit = simulate(M, C1, K0, K2, q_ref, dq_ref, kf - ki, tk, vk, torque_k)

        # Plot results
        # plts[2 * i].figure()
        plts[0, i].axhline(c='black')
        plts[0, i].grid()
        plts[0, i].plot(tk, q_ref[:, 0], label='Phi (rad)')
        plts[0, i].plot(tk, q_ref[:, 1], label='Del (rad)')
        plts[0, i].plot(tk, torque_k, label='Torque (Nm)')
        plts[0, i].plot(tk, q_naive[:, 0], label='Phi Naively Predicted (rad)')
        plts[0, i].plot(tk, q_naive[:, 1], label='Del Naively Predicted (rad)')
        plts[0, i].plot(tk, q_fit[:, 0], label='Phi Fit Predicted (rad)')
        plts[0, i].plot(tk, q_fit[:, 1], label='Del Fit Predicted (rad)')
        # plts[0, i].xlabel('Time (s)')
        # plts[0, i].title('Phi/Del Comparison')
        # plts[0, i].legend()
        # plts[2*i].show()

        # plts[2 * i + 1].figure()
        plts[1, i].axhline(c='black')
        plts[1, i].grid()
        plts[1, i].plot(tk, dq_ref[:, 0], label='dPhi (rad/s)')
        plts[1, i].plot(tk, dq_ref[:, 1], label='dDel (rad/s)')
        plts[1, i].plot(tk, torque_k, label='Torque (Nm)')
        plts[1, i].plot(tk, dq_naive[:, 0], label='dPhi Naively Predicted (rad/s)')
        plts[1, i].plot(tk, dq_naive[:, 1], label='dDel Naively Predicted (rad/s)')
        plts[1, i].plot(tk, dq_fit[:, 0], label='dPhi Fit Predicted (rad/s)')
        plts[1, i].plot(tk, dq_fit[:, 1], label='dDel Fit Predicted (rad/s)')
        # plts[1, i].xlabel('Time (s)')
        # plts[1, i].title('dPhi/dDel Comparison')
        # plts[1, i].legend()
        # plts[2*i+1].show()

    # f.legend()
    plts[0, 0].legend()
    plts[1, 0].legend()
    f.show()


plot_results()
