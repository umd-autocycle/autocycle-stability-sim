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
                self.k_i = .00001
                self.k_d = 90
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif 13 > v >= 11:  # 12
                self.k_p = 485
                self.k_i = .00001
                self.k_d = 100
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral
            elif v >= 13:  # 14
                self.k_p = 2178
                self.k_i = .00001
                self.k_d = 300
                return self.k_p * e[0] + self.k_d * e[2] + self.k_i * self.integral

        return pid_phi
