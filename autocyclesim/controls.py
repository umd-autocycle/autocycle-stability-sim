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
