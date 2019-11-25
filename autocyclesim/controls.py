from numpy import sign
import time

class Control:
    def get_control(self, goals):
        def no_control(t, e, v):
            return 0

        return no_control


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

    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[0] * t - self.last_time
            self.last_time = t
            return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral

        return pid_phi


class PIDPhiInterpolated(Control):
    integral = 0
    last_time = 0

    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[0] * t - self.last_time
            self.last_time = t
            if v < 3:  # 2
                self.k_p = 79.5
                self.k_i = 0
                self.k_d = 155
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 5 > v >= 3:  # 4
                self.k_p = 230
                self.k_i = 0
                self.k_d = 100
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 7 > v >= 5:  # 5.5
                self.k_p = 295
                self.k_i = 0
                self.k_d = 110
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 9 > v >= 7:  # 8
                self.k_p = 270
                self.k_i = 0
                self.k_d = 85
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 11 > v >= 9:  # 10
                self.k_p = 420
                self.k_i = 0
                self.k_d = 90
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 13 > v >= 11:  # 12
                self.k_p = 485
                self.k_i = 0
                self.k_d = 100
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif v >= 13:  # 14
                self.k_p = 2178
                self.k_i = 0
                self.k_d = 300
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral

        return pid_phi


class Lyapunov(Control):

    def __init__(self, E3):
        self.E3 = E3

    def get_control(self, goals):
        def lyapunov_phi(t, e, v):
            u3 = (10.4702 * e[0] + (-.5888 - .8868 * v * v) * e[1] - .104 * v * e[2] - .3277 * v * e[3] + sign(e[2]) * self.E3) / .1226
            return u3

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

            u1 = (10.4702*e[0]+(-.5888-.8868*v*v)*e[1]-.104*v*e[2]-.3277*v*e[3]+self.E1)/.1226
            u2 = (10.4702*e[0]+(-.5888-.8868*v*v)*e[1]-.104*v*e[2]-.3277*v*e[3]-self.E2)/.1226
            u3 = (10.4702*e[0]+(-.5888-.8868*v*v)*e[1]-.104*v*e[2]-.3277*v*e[3]+sign(e[2])*self.E3)/.1226
            u4 = 0

            c4 = 0
            c3 = 0
            c2 = 0
            c1 = 0

            if abs(e[2]) < self.zd:
                if abs(e[0]) < self.z:
                    c4 = min((abs(e[0])-self.z)/self.z, (abs(e[2])-self.zd)/self.zd)
                    c3 = 2*min(abs(e[0])/self.np, (abs(e[2])-self.zd)/self.zd)
                    c1 = 2 * min(abs(e[0])/self.np, abs(e[2])/self.npd) + min((abs(e[0])-self.z)/self.z, abs(e[2])/self.npd)
                else:
                    c3 = 2*min(abs(e[0])/self.np, (abs(e[2])-self.zd)/self.zd)
                    c1 = min(e[0] / self.np, e[2] / self.npd) + min(0, e[2]/self.npd)
            elif self.zd < abs(e[2]) < self.np:
                if abs(e[0]) < self.z:
                    c1 = 2 * min(abs(e[0]) / self.np, abs(e[2]) / self.npd) + min((abs(e[0])-self.z)/self.z, abs(e[2])/self.npd)
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

            return (c4*u4+c3*u3+c2*u2+c1*u1)/(c4+c3+c2+c1)

        return fuzzy_lyapunov_phi
