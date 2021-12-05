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
