from numpy import sign
import time
from math import sqrt
import slycot ##this causes a crash
import control
import numpy as np


class Control:
    def get_control(self, goals):
        def no_control(t, e, v):
            return 0

        return no_control

    def reset_registers(self):
        pass

class FullStateFeedback(Control):
    def __init__(self, eval1, eval2, eval3, eval4):
        self.eval1 = eval1
        self.eval2 = eval2
        self.eval3 = eval3
        self.eval4 = eval4
    def get_control(self, goals):
        def fsf(t, e, v):
            A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [9.4702, -0.5888 - 0.8868 * v * v, -0.104 * v, -0.3277 * v],
                 [12.3999, 31.5587 - 2.0423 * v * v, 3.6177 * v, -3.1388 * v]])
            B = np.array([[0], [0], [-0.1226], [4.265]])
            K = control.place(A, B, [self.eval1, self.eval2, self.eval3, self.eval4])
            e_transpose = np.array([[e[0]], [e[1]], [e[2]], [e[3]]])
            ans = (-1*K * e_transpose)
            return ans[0,0]
        return fsf

class LQR(Control):
    def __init__(self, k_phi,k_delta,k_torque):
        self.Q = np.array([[k_phi, 0, 0, 0],[0, k_delta, 0, 0], [0, 0, 1, 0],[0, 0, 0, 1]])
        self.R = np.array([k_torque])
    def get_control(self, goals):
        def lqr(t, e, v):
            A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [9.4702, -0.5888 - 0.8868 * v * v, -0.104 * v, -0.3277 * v],
                 [12.3999, 31.5587 - 2.0423 * v * v, 3.6177 * v, -3.1388 * v]])
            B = np.array([[0], [0], [-0.1226], [4.265]])
            K, S, E = control.lqr(A,B,self.Q,self.R)
            e_transpose = np.array([[e[0]], [e[1]], [e[2]], [e[3]]])
            ans = (-1*K * e_transpose)
            return ans[0,0]
        return lqr

class PDPhi(Control):
    def __init__(self, k_p, k_d):
        self.k_p = k_p
        self.k_d = k_d

    def get_control(self, goals):
        def pd_phi(t, e, v):
            return self.k_p * e[0] + self.k_d * e[2]

        return pd_phi


class PIDPhi(Control):
    integral = 0
    last_time = 0

    def __init__(self, k_p, k_i, k_d, max_torque):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.max_torque = max_torque

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[0] * (t - self.last_time)
            self.last_time = t
            return min(self.max_torque,
                       max(-self.max_torque, self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral))

        return pid_phi

    def reset_registers(self):
        self.integral = 0
        self.last_time = 0

class PIDDelta(Control):
    integral = 0
    last_time = 0
# v = 5.5 [ 4.,  0.03169785, -0.4409042 ]
# v = 6 [ 7.07133204,  0.06475467, -1.00706603]
# v = 7 [14.62731783,  0.0930806 , -1.98779757]
# v = 9 [33.7214239 ,  0.22002286, -3.86885105]



    def __init__(self, k_p, k_i, k_d, max_torque):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.max_torque = max_torque

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[1] * (t - self.last_time)
            self.last_time = t
            return min(self.max_torque,
                       max(-self.max_torque, self.k_p * e[1] + self.k_d * e[3] + self.k_i * self.integral))

        return pid_phi

    def reset_registers(self):
        self.integral = 0
        self.last_time = 0


class PIDPhiInterpolated(Control):
    integral = 0
    last_time = 0
    first = 4
    final = 15
    interval = 0.5

    def __init__(self, max_torque):
        self.max_torque = max_torque
        self.params = [[33.69352243, 0.263932, 21.15690148],
                       [101.20843419, 4.4401922, 38.1772637],
                       [102.20843419, 5.4401922, 37.55922972],
                       [103.20843419, 5.8221582, 35.94119572],
                       [102.97236622, 5.96805622, 34.32316172],
                       [103.97236622, 6.35002222, 32.70512772],
                       [104.97236622, 6.49592025, 32.08709375],
                       [104.35433225, 6.18809353, 27.85102572],
                       [105.35433225, 7.18809353, 27.23299175],
                       [105.25222667, 7.33399156, 26.61495777],
                       [105.63419267, 7.21653726, 25.9969238],
                       [104.61175785, 7.09906907, 25.37888982],
                       [105.35554416, 7.62693309, 19.40675385],
                       [103.73751016, 8.00889909, 17.78871985],
                       [104.73751016, 8.39086509, 17.17068588],
                       [104.47474016, 8.77283109, 16.5526519],
                       [105.47474016, 8.15479712, 15.93461793],
                       [102.25887179, 8.53676312, 14.31658393],
                       [101.64083781, 8.91872912, 13.69854995],
                       [102.64083781, 9.30069512, 12.08051595],
                       [102.02280384, 8.68266114, 11.46248198],
                       [103.02280384, 9.68266114, 9.84444798],
                       [104.02280384, 10.68266114, 9.226414]]

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[0] * (t - self.last_time)
            self.last_time = t

            index = int((v - self.first) / self.interval)
            interpolation = (v - self.first - index * self.interval) / self.interval

            k_p = self.params[index][0] + interpolation * (self.params[index + 1][0] - self.params[index][0])
            k_i = self.params[index][1] + interpolation * (self.params[index + 1][1] - self.params[index][1])
            k_d = self.params[index][2] + interpolation * (self.params[index + 1][2] - self.params[index][2])

            return min(self.max_torque, max(-self.max_torque, k_p * e[0] + k_d * e[2] + k_i * self.integral))

        return pid_phi

    def reset_registers(self):
        self.integral = 0
        self.last_time = 0


class Lyapunov(Control):

    def __init__(self, E3):
        self.E3 = E3

    def get_control(self, goals):
        def lyapunov_phi(t, e, v):
            a = 10.4702 * e[0]
            b = (-.5888 - .8868 * v * v) * e[1]
            c = - .104 * v * e[2]
            d = - .3277 * v * e[3]
            f = sign(e[2]) * self.E3

            # print(str(t)+'       '+str((a + b + c + d + f)/ .1226))

            return (a + b + c + d + f) / .1226

        return lyapunov_phi


class FuzzyLyapunov(Control):

    def __init__(self, np, z, npd, zd, E1, E3):
        self.np = np
        self.z = z
        self.npd = npd
        self.zd = zd
        self.E1 = E1
        self.E2 = E1
        self.E3 = E3

    def get_control(self, goals):
        def fuzzy_lyapunov_phi(t, e, v):

            u1 = (10.4702 * e[0] + (-.5888 - .8868 * v * v) * e[1] - .104 * v * e[2] - .3277 * v * e[
                3] + self.E1) / .1226
            u2 = (10.4702 * e[0] + (-.5888 - .8868 * v * v) * e[1] - .104 * v * e[2] - .3277 * v * e[
                3] - self.E2) / .1226
            u3 = (10.4702 * e[0] + (-.5888 - .8868 * v * v) * e[1] - .104 * v * e[2] - .3277 * v * e[3] + sign(
                e[2]) * self.E3) / .1226
            u4 = 0

            c4 = 0
            c3 = 0
            c2 = 0
            c1 = 0

            if abs(e[2]) < self.zd:
                if abs(e[0]) < self.z:
                    c4 = min((abs(e[0]) - self.z) / self.z, (abs(e[2]) - self.zd) / self.zd)
                    c3 = 2 * min(abs(e[0]) / self.np, (abs(e[2]) - self.zd) / self.zd)
                    c1 = 2 * min(abs(e[0]) / self.np, abs(e[2]) / self.npd) + min((abs(e[0]) - self.z) / self.z,
                                                                                  abs(e[2]) / self.npd)
                else:
                    c3 = 2 * min(abs(e[0]) / self.np, (abs(e[2]) - self.zd) / self.zd)
                    c1 = min(e[0] / self.np, e[2] / self.npd) + min(0, e[2] / self.npd)
            elif self.zd < abs(e[2]) < self.np:
                if abs(e[0]) < self.z:
                    c1 = 2 * min(abs(e[0]) / self.np, abs(e[2]) / self.npd) + min((abs(e[0]) - self.z) / self.z,
                                                                                  abs(e[2]) / self.npd)
                else:
                    c1 = 2 * min(abs(e[0]) / self.np, abs(e[2]) / self.npd) + 0
            else:
                if abs(e[0]) < self.z:
                    c1 = 2 * abs(e[0]) / self.np + (abs(e[0]) - self.z) / self.z
                else:
                    c1 = 2 * abs(e[0]) / self.np + 0
            '''
            if abs(e[0]) < self.z:
                pz = (self.z-abs(e[0]))/self.z
            else:
                pz = 0
            if abs(e[2]) < self.zd:
                pzd = (self.zd-abs(e[2]))/self.zd
            else:
                pzd = 0
            if abs(e[0]) < self.np:
                pnp = e[0]/self.np
            else:
                pnp = 1
            if abs(e[2]) < self.np:
                pnpd = e[2]/self.np
            else:
                pnpd = 1
            
            c4 = min(pz, pzd)
            c3 = 2*min(pnp, pzd)
            c2 = 0
            c1 = 2*min(pnp, pnpd) + min(pz, pnpd)
            '''

            if e[2] < 0:
                c2 = c1
                c1 = 0

            return (c4 * u4 + c3 * u3 + c2 * u2 + c1 * u1) / (c4 + c3 + c2 + c1)

        return fuzzy_lyapunov_phi
