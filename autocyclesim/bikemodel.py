import numpy as np
from math import pi, sin, cos, atan


class BikeModel:
    def linearized_1st_order(self, v, f):
        pass


class MeijaardModel(BikeModel):
    w = 1.16
    t = 0.09
    alpha = 1.319
    g = 9.81

    r_rw = 0.35
    m_rw = 3.30
    A = np.array([0.177, 0.354, 0.177])

    # Rear frame parameters
    x_arf = 0.4
    y_arf = 0
    z_arf = -0.605

    m_arf = 28.65
    B_arf = np.array([
        [3.124, 0, -0.877],
        [0, 4.150, 0],
        [-0.877, 0, 3.398, ]
    ])

    # ZSS Parameters
    x_zss = 0
    y_zss = 0
    z_zss = -0.45  # GUESS

    m_zss = 6.352
    B_zss = np.array([
        [0.223, -0.045, -0.001],
        [-0.045, 0.252, 0.002],
        [-0.001, 0.002, 0.473]
    ])

    com_arf = np.array([x_arf, y_arf, z_arf])
    com_zss = np.array([x_zss, y_zss, z_zss])
    com_arf = com_arf.reshape((3, 1))
    com_zss = com_zss.reshape((3, 1))

    # Combine ZSS and Rear Frame MOI, COM, and Mass
    m_rf = m_arf + m_zss
    com_rf = (com_arf * m_arf + com_zss * m_zss) / m_rf
    d_zss = com_zss - com_rf
    d_arf = com_arf - com_rf
    dd_zss = np.identity(3) * np.linalg.norm(d_zss) * np.linalg.norm(d_zss) - d_zss @ d_zss.T
    dd_arf = np.identity(3) * np.linalg.norm(d_arf) * np.linalg.norm(d_arf) - d_arf @ d_arf.T
    B = B_arf + B_zss + dd_arf * m_arf + dd_zss * m_zss

    x_rf = com_rf[0, 0]
    y_rf = com_rf[1, 0]
    z_rf = com_rf[2, 0]

    x_ff, y_ff, z_ff = 0.92, 0, -0.835
    m_ff = 3.05
    C = np.array([
        [0.344, 0, 0.0637],
        [0, 0.239, 0],
        [0.0637, 0, 0.0578],
    ])

    r_fw = 0.35
    m_fw = 2.90
    D = np.array([0.177, 0.354, 0.177])

    def __init__(self, zss):
        if not zss:
            self.x_rf, self.y_rf, self.z_rf = 0.40, 0, -.605
            self.m_rf = 28.65
            self.B = np.array([
                [3.124, 0, -0.877],
                [0, 4.150, 0],
                [-0.877, 0, 3.398],
            ])

        m_t = self.m_rw + self.m_rf + self.m_ff + self.m_fw
        x_t = (self.x_rf * self.m_rf + self.x_ff * self.m_ff + self.w * self.m_fw) / m_t
        z_t = (-self.r_rw * self.m_rw + self.z_rf * self.m_rf
               + self.z_ff * self.m_ff - self.r_fw * self.m_fw) / m_t

        t_xx = self.A[0] + self.B[0, 0] + self.C[0, 0] + self.D[0] \
               + self.m_rw * (self.r_rw ** 2) + self.m_rf * (self.z_rf ** 2) \
               + self.m_fw * (self.r_fw ** 2) + self.m_ff * (self.z_ff ** 2)
        t_xz = self.B[0, 2] + self.C[0, 2] - self.m_rf * self.x_rf * self.z_rf \
               - self.m_ff * self.x_ff * self.z_ff + self.m_fw * self.w * self.r_fw
        t_zz = self.A[2] + self.B[2, 2] + self.C[2, 2] + self.D[2] + self.m_rf * (self.x_rf ** 2) \
               + self.m_ff * (self.x_ff ** 2) + self.m_fw * (self.w ** 2)

        m_f = self.m_ff + self.m_fw
        x_f = (self.x_ff * self.m_ff + self.w * self.m_fw) / m_f
        z_f = (self.z_ff * self.m_ff - self.r_fw * self.m_fw) / m_f

        f_xx = self.C[0, 0] + self.D[0] + self.m_ff * ((self.z_ff - z_f) ** 2) \
               + self.m_fw * ((self.r_fw + z_f) ** 2)
        f_xz = self.C[0, 2] - self.m_ff * (self.x_ff - x_f) * (self.z_ff - z_f) \
               + self.m_fw * (self.w - x_f) * (self.r_fw + z_f)
        f_zz = self.C[2, 2] + self.D[2] + self.m_ff * ((self.x_ff - x_f) ** 2) \
               + self.m_fw * ((self.w - x_f) ** 2)

        lam = pi / 2 - self.alpha
        u = (x_f - self.w - self.t) * cos(lam) - z_f * sin(lam)

        f_ll = m_f * (u ** 2) + f_xx * (sin(lam) ** 2) + 2 * f_xz * sin(lam) * cos(lam) \
               + f_zz * (cos(lam) ** 2)
        f_lx = -m_f * u * z_f + f_xx * sin(lam) + f_xz * cos(lam)
        f_lz = m_f * u * x_f + f_xz * sin(lam) + f_zz * cos(lam)

        f = self.t * cos(lam) / self.w

        s_r = self.A[1] / self.r_rw
        s_f = self.D[1] / self.r_fw
        s_t = s_r + s_f

        s_u = m_f * u + f * m_t * x_t

        self.m = np.array([
            [t_xx, f_lx + f * t_xz],
            [f_lx + f * t_xz, f_ll + 2 * f * f_lz + f ** 2 * t_zz],
        ])

        self.k0 = np.array([
            [self.g * m_t * z_t, -self.g * s_u],
            [-self.g * s_u, -self.g * s_u * sin(lam)],
        ])

        self.k2 = np.array([
            [0, (s_t - m_t * z_t) * cos(lam) / self.w],
            [0, (s_u + s_f * sin(lam)) * cos(lam) / self.w],
        ])

        self.c1 = np.array([
            [0, f * s_t + s_f * cos(lam) + t_xz * cos(lam) / self.w - f * m_t * z_t],
            [-(f * s_t + s_f * cos(lam)), f_lz * cos(lam) / self.w + f * (s_u + t_zz * cos(lam) / self.w)],
        ])
        self.m_inv = np.linalg.inv(self.m)

    def linearized_1st_order(self, v, h):
        """
        e0 = phi
        e1 = delta
        e2 = phi'
        e3 = delta'

        so

        e0' = e2
        e1' = e3
        e2' = phi''
        e3' = delta''
        """

        self.Amatrix = self.dynamics_matrix(v, True)

        self.Bmatrix = self.controls_matrix(v, True)

        def sd(t, e):
            # print(f_lx)
            # print(f)
            # print(t_xz)
            # print(m)
            # print(self.k0)
            # print(self.k2)
            # print(self.c1)
            g = np.array([0.0 if h[0] is None else (h[0])(t, e, v), 0.0 if h[1] is None else (h[1])(t, e, v)])
            q = np.array([e[0], e[1]])
            q_dot = np.array([e[2], e[3]])
            q_ddot = np.dot(self.m_inv, g - np.dot(v * self.c1, q_dot) - np.dot((self.k0 + self.k2 * (v ** 2)), q))

            return [
                q_dot[0],
                q_dot[1],
                q_ddot[0],
                q_ddot[1]
            ]

        return sd

    def dynamics_matrix(self, v, free_running):
        if free_running:
            return np.concatenate(
                (np.concatenate((np.zeros((2, 2)), np.dot(-1 * self.m_inv, (self.k0 + self.k2 * (v ** 2)))), axis=0),
                 np.concatenate((np.eye(2), np.dot(-1 * self.m_inv, self.c1 * v)), axis=0)), axis=1)
        else:
            return np.concatenate(
                (np.concatenate((np.zeros((2, 2)), np.zeros((2, 2))), axis=0),
                 np.concatenate((np.eye(2), np.zeros((2, 2))), axis=0)), axis=1)

    def controls_matrix(self, v, free_running):
        if free_running:
            return np.concatenate((np.zeros((2, 2)), self.m_inv), axis=0)
        else:
            return np.concatenate((np.zeros((2, 2)), np.zeros((2, 2))), axis=0)

    def kalman_transition_matrix(self, v, dt, free_running):
        return np.identity(4) + self.dynamics_matrix(v, free_running) * dt

    def kalman_controls_matrix(self, v, dt, free_running):
        return self.controls_matrix(v, free_running) * dt

    def controllable(self, v):
        A = self.dynamics_matrix(v, True)
        B = self.controls_matrix(v, True)

        n = A.shape[0]

        Ct = np.zeros((n, 2 * n))
        for k in range(0, n-1, 2):
            Ct[:, k:(k+2)] = np.linalg.matrix_power(A, k - 1) @ B

        return np.linalg.matrix_rank(Ct) == n


if __name__ == '__main__':
    model = MeijaardModel(zss=True)
    model.m = np.array([[26.67504339, 1.21856943],
                        [1.21856943, 0.59438128]])
    model.c1 = np.array([[0., 4.97370516],
                         [-0.98015773, 2.43085255]])
    model.k0 = np.array([[-210.6481775, 1.14387605],
                         [1.14387605, 3.2817143]])
    model.k2 = np.array([[0., 21.88145723],
                         [0., -0.86196881]])
    model.m_inv = np.linalg.inv(model.m)

    print(model.controllable(4))

    vv = np.linspace(0, 8, 400)
    eek = []

    import matplotlib.pyplot as plt

    for v in vv:
        model.linearized_1st_order(v, [None, None])
        eig_val, eig_vec = np.linalg.eig(model.Amatrix)
        eig_val = sorted(eig_val, key=np.real)

        eek.append(eig_val)

    eek = np.array(eek)

    plt.plot(vv, eek[:, 3], '-r', label="e1", )
    plt.plot(vv, eek[:, 2], '-r', label="e2")
    plt.plot(vv, eek[:, 1], '-r', label="e3")
    plt.plot(vv, eek[:, 0], '-r', label="e4")
    plt.axline((0, 0), (8, 0))
    plt.xlabel('Bicycle Velocity (m/s)')
    plt.ylabel('Eigenvalue Real Part (1/s)')
    plt.title('Dynamics Matrix Eigenvalues vs. Velocity')
    # plt.legend()
    plt.show()
