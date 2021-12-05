from numpy import sign
import time
from math import sqrt
import slycot  ##this causes a crash
import control
import numpy as np

from bikemodel import MeijaardModel


# model = MeijaardModel()
# model.linearized_1st_order(4, [None, None])


class Control:
    def get_control(self, goals):
        def no_control(t, e, v):
            return 0

        return no_control

    def reset_registers(self):
        pass


class FullStateFeedback(Control):
    def __init__(self, bike_model, eval1, eval2, eval3, eval4):
        self.eval1 = eval1
        self.eval2 = eval2
        self.eval3 = eval3
        self.eval4 = eval4
        self.model = bike_model

    def get_control(self, goals):
        def fsf(t, e, v):
            if t == 0:
                start = time.time()
                A = np.concatenate(
                    (np.concatenate(
                        (np.zeros((2, 2)), np.dot(-1 * self.model.m_inv, (self.model.K0 + self.model.K2 * (v ** 2)))),
                        axis=0),
                     np.concatenate((np.eye(2), np.dot(-1 * self.model.m_inv, self.model.C1 * v)), axis=0)), axis=1)
                B = (np.concatenate((np.zeros((2, 2)), self.model.m_inv), axis=0)[:, 1])[..., None]
                self.K = control.place(A, B, [self.eval1, self.eval2, self.eval3, self.eval4])
                end = time.time()

                print(self.K)
                # print(end - start)

            e_transpose = np.array([[e[0]], [e[1]], [e[2]], [e[3]]])
            ans = (-self.K * e_transpose)
            # print(ans[0,0])
            return ans[0, 0]

        return fsf


# class FromData(Control):
#     def __init__(self, data):
#         self.data = data


class FSFFirmware(Control):
    def __init__(self, bike_model, torque_max, l1, l2, l3, l4):
        self.model = bike_model
        self.torque_max = torque_max

        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

        self.M_det = np.linalg.det(bike_model.m)
        self.C1_det = np.linalg.det(bike_model.c1)
        self.K0_det = np.linalg.det(bike_model.k0)
        self.K2_det = np.linalg.det(bike_model.k2)
        self.C1_M_det = np.linalg.det(bike_model.c1 - bike_model.m)
        self.K2_M_det = np.linalg.det(bike_model.k2 - bike_model.m)
        self.K0_M_det = np.linalg.det(bike_model.k0 - bike_model.m)
        self.C1_K2_det = np.linalg.det(bike_model.c1 - bike_model.k2)
        self.C1_K0_det = np.linalg.det(bike_model.c1 - bike_model.k0)
        self.K0_K2_det = np.linalg.det(bike_model.k0 - bike_model.k2)

    def get_control(self, goals):
        def fsf(t, e, v):
            start = time.time()
            LHS = np.array([
                [self.Qk1(self.l1, v), self.Qk2(self.l1, v), self.Qk3(self.l1, v), self.Qk4(self.l1, v)],
                [self.Qk1(self.l2, v), self.Qk2(self.l2, v), self.Qk3(self.l2, v), self.Qk4(self.l2, v)],
                [self.Qk1(self.l3, v), self.Qk2(self.l3, v), self.Qk3(self.l3, v), self.Qk4(self.l3, v)],
                [self.Qk1(self.l4, v), self.Qk2(self.l4, v), self.Qk3(self.l4, v), self.Qk4(self.l4, v)],
            ])
            RHS = np.array([
                [-self.RHSe(self.l1, v)],
                [-self.RHSe(self.l2, v)],
                [-self.RHSe(self.l3, v)],
                [-self.RHSe(self.l4, v)]
            ])

            K = np.dot(np.linalg.inv(LHS), RHS)
            end = time.time()
            # print(end - start)
            # K[3,0] = 0
            print(K)
            # print(np.dot(K.T, np.array(e).T))
            # print(np.array(e).T)
            # print(np.array(e).T + np.random.normal(0, 0.1, 4))
            err = np.array(e).T
            # err[0] += np.random.normal(0, 0.01)
            # err[2] += np.random.normal(0, 0.01)
            # err[3] += np.random.normal(0, 0.01)
            return -(np.dot(K.T, err))[0]

        return fsf

    def Qk1(self, l, v):
        return -(self.model.m[0, 1] * l * l + self.model.c1[0, 1] * l * v + self.model.k2[0, 1] * v * v + self.model.k0[
            0, 1])

    def Qk2(self, l, v):
        return self.model.m[0, 0] * l * l + self.model.c1[0, 0] * l * v + self.model.k2[0, 0] * v * v + self.model.k0[
            0, 0]

    def Qk3(self, l, v):
        return -(self.model.m[0, 1] * l * l + self.model.c1[0, 1] * l * v + self.model.k2[0, 1] * v * v + self.model.k0[
            0, 1]) * l

    def Qk4(self, l, v):
        return (self.model.m[0, 0] * l * l + self.model.c1[0, 0] * l * v + self.model.k2[0, 0] * v * v + self.model.k0[
            0, 0]) * l

    def RHSe(self, l, v):
        return l * l * l * l * self.M_det + l * l * l * v * (self.C1_det + self.M_det - self.C1_M_det) + \
               l * l * v * v * (self.C1_det + self.M_det + self.K2_det - self.K2_M_det) \
               + l * l * (self.K0_det + self.M_det - self.K0_M_det) + l * v * v * v * (
                       self.C1_det + self.K2_det - self.C1_K2_det) \
               + l * v * (self.C1_det + self.K0_det - self.C1_K0_det) + v * v * v * v * self.K2_det \
               + v * v * (self.K0_det + self.K2_det - self.K0_K2_det) + self.K0_det


class LQR(Control):
    def __init__(self, k_phi, k_delta, k_dphi, k_ddelta, k_torque):
        self.Q = np.array([[k_phi, 0, 0, 0], [0, k_delta, 0, 0], [0, 0, k_dphi, 0], [0, 0, 0, k_ddelta]])
        self.R = np.array([k_torque])

    def get_control(self, goals):
        def lqr(t, e, v):
            A = np.concatenate(
                (np.concatenate((np.zeros((2, 2)), np.dot(-1 * model.m_inv, (model.K0 + model.K2 * (v ** 2)))), axis=0),
                 np.concatenate((np.eye(2), np.dot(-1 * model.m_inv, model.C1 * v)), axis=0)), axis=1)
            B = (np.concatenate((np.zeros((2, 2)), model.m_inv), axis=0)[:, 1])[..., None]
            K, S, E = control.lqr(A, B, self.Q, self.R)
            e_transpose = np.array([[e[0]], [e[1]], [e[2]], [e[3]]])
            ans = (-K * e_transpose)
            return ans[0, 0]

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
