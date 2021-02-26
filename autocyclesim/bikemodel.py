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

    x_rf, y_rf, z_rf = 0.40, 0, -.605
    m_rf = 28.65
    B = np.array([
        [3.124, 0, -0.877],
        [0, 4.150, 0],
        [-0.877, 0, 3.398],
    ])

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

    def linearized_1st_order(self, v, h):
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

        m = np.array([
            [t_xx, f_lx + f * t_xz],
            [f_lx + f * t_xz, f_ll + 2 * f * f_lz + f ** 2 * t_zz],
        ])

        k0 = np.array([
            [self.g * m_t * z_t, -self.g * s_u],
            [-self.g * s_u, -self.g * s_u * sin(lam)],
        ])

        k2 = np.array([
            [0, (s_t - m_t * z_t) * cos(lam) / self.w],
            [0, (s_u + s_f * sin(lam)) * cos(lam) / self.w],
        ])

        c1 = np.array([
            [0, f * s_t + s_f * cos(lam) + t_xz * cos(lam) / self.w - f * m_t * z_t],
            [-(f * s_t + s_f * cos(lam)), f_lz * cos(lam) / self.w + f * (s_u + t_zz * cos(lam) / self.w)],
        ])

        '''
        e0 = phi
        e1 = delta
        e2 = phi'
        e3 = delta'
        
        so
        
        e0' = e2
        e1' = e3
        e2' = phi''
        e3' = delta''
        '''
        m_inv = np.linalg.inv(m)

        def sd(t, e):
            # print(f_lx)
            # print(f)
            # print(t_xz)
            # print(m)
            # print(k0)
            # print(k2)
            # print(c1)
            g = np.array([0.0 if h[0] is None else (h[0])(t, e, v), 0.0 if h[1] is None else (h[1])(t, e, v)])
            q = np.array([e[0], e[1]])
            q_dot = np.array([e[2], e[3]])
            q_ddot = np.dot(m_inv, g - np.dot(v * c1, q_dot) - np.dot((k0 + k2 * (v ** 2)), q))

            return [
                q_dot[0],
                q_dot[1],
                q_ddot[0],
                q_ddot[1]
            ]

        return sd
