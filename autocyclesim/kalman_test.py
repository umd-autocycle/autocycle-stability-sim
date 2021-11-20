import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter, SquareRootKalmanFilter
from bikemodel import MeijaardModel
from kalman import HomeBrewKalmanFilter, choleskyFromW

model = MeijaardModel(zss=False)

var_roll_accel = 0.57  # Variance in (rad/s^2)^2
var_steer_accel = 0.16  # 0.57#0.16  # Variance in (rad/s^2)^2

var_phi = 0.00002629879368
var_del = 0.0000001
var_dphi = 0.0000002117716535
var_ddel = 0.0000001

# f = KalmanFilter(dim_x=4, dim_z=4, dim_u=2)
# f = SquareRootKalmanFilter(dim_x=4, dim_z=4, dim_u=2)
f = HomeBrewKalmanFilter(dim_x=4, dim_z=4, dim_u=2)

dtype = 'float32'
f.x = np.array([0., 0., 0., 0.], dtype=dtype)  # Initial estimates
f.P_2 *= 0.1
# f.ud_factorize()

f.H = np.identity(4, dtype=dtype)  # Measurement function

# Measurement noise
f.R_2 = np.sqrt(np.array([
    [var_phi, 0., 0., 0.],
    [0., var_del, 0., 0.],
    [0., 0., var_dphi, 0.],
    [0., 0., 0., var_ddel]
], dtype=dtype))

data = pd.read_csv('test.csv')

phil = []
dell = []

t_prev = 21.540
n = 1
for index, row in data.iterrows():
    phi_y = row['Phi_y']
    del_y = row['Del_y']
    dphi_y = row['dPhi_y']
    ddel_y = row['Del_y']
    v = row['Velocity']
    t = row['t (s)']
    torque = row['Torque']
    dt = t - t_prev
    # dt = dt/n
    t_prev = t

    for i in range(n):
        f.F = model.kalman_transition_matrix(v, dt, False).astype(dtype)
        f.B = model.kalman_controls_matrix(v, dt, False).astype(dtype)

        w_orr = np.array([
            [0.5 * var_roll_accel * dt * dt],
            [0.5 * var_steer_accel * dt * dt],
            [var_roll_accel * dt],
            [var_steer_accel * dt]
        ], dtype=dtype)

        # f.w = w_orr
        # print(w_orr)
        zer = np.zeros((4, 3), dtype=dtype)
        f.Q_2 = np.concatenate((w_orr, zer), axis=1)
        # f._Q1_2 = choleskyFromW(w_orr)
        # print(f._Q1_2)
        # print(w_orr @ w_orr.T)
        # f.Q = w_orr @ w_orr.T

        f.predict(np.array([0., torque], dtype=dtype))
        f.update(np.array([phi_y, del_y, dphi_y, ddel_y], dtype=dtype))

        phil.append(f.x[0])
        dell.append(f.x[1])

plt.plot(phil)
plt.plot(dell)
# plt.plot(data['t (s)'], phil)
# plt.plot(data['t (s)'], dell)
# plt.plot(data['t (s)'], data['Phi_y'])
# plt.plot(data['t (s)'], data['Del_y'])
plt.show()
