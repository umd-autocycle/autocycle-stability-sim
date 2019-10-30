import numpy as np
from math import atan


class BikeModel:
    def linearized_1st_order(self, v):
        pass


class MeijaardModel(BikeModel):
    w = 1.02
    t = 0.08
    alpha = atan(3)
    g = 9.81
    v = 0

    R_rw = 0.3
    m_rw = 2
    A = np.array([0.06, 0.12, 0.06])

    com_rf = 0.3, 0, -0.9
    B = np.array([
        [9.2, 0, 2.4],
        [0, 11, 0],
        [2.4, 0, 2.8],
    ])

    com_ff = np.array([0.9, 0, -0.7])
    m_ff = 4
    C = np.array([
        [0.0546, 0, -0.0162],
        [0, 0.06, 0],
        [-0.0162, 0, 0.0114],
    ])

    r_fw = 0.35
    m = 3
    D = np.array([0.14, 0.28, 0.14])

    def linearized_1st_order(self, v):
        def sd(t, e):
            return [

            ]

        return sd
