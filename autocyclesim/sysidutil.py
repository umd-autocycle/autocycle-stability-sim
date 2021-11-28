import numpy as np
from math import sin, cos, pi


def matrices2theta(M, C1, K0, K2):
    return np.array(
        [M[0, 0], M[0, 1], M[1, 1], K0[0, 0], K0[0, 1], K2[0, 1], K2[1, 1], C1[0, 1], C1[1, 0], C1[1, 1]])


# noinspection PyPep8Naming
def theta2matrices(theta):
    w = 1.16
    t = 0.09
    alpha = 1.319
    g = 9.81

    r_rw = 0.35
    m_rw = 3.30

    r_fw = 0.35
    m_fw = 2.90

    m_rf = 35
    m_ff = 3.05

    Ayy = 0.354
    Dyy = 0.320

    # x_ff, z_ff = 0.92, -0.835

    Bxx, Bzz, Bxz, Cxx, Czz, Cxz, x_rf, z_rf, x_ff, z_ff = theta
    Axx = Azz = Ayy / 2
    Dxx = Dzz = Dyy / 2

    # Total mass and COM
    m_t = m_rw + m_rf + m_ff + m_fw
    x_t = (x_rf * m_rf + x_ff * m_ff + w * m_fw) / m_t
    z_t = (-r_rw * m_rw + z_rf * m_rf + z_ff * m_ff - r_fw * m_fw) / m_t

    # Mass moments and products of inertia
    Txx = Axx + Bxx + Cxx + Dxx + m_rw * r_rw ** 2 + m_rf * z_rf ** 2 + m_ff * z_ff ** 2 + m_fw * r_fw ** 2
    Txz = Bxz + Cxz - m_rf * x_rf * z_rf - m_ff * x_ff * z_ff + m_fw * w * r_fw
    Tzz = Azz + Bzz + Czz + Dzz + m_rf * x_rf ** 2 + m_ff * x_ff ** 2 + m_fw * w ** 2

    # Front assembly mass and COM
    m_f = m_ff + m_fw
    x_f = (x_ff * m_ff + w * m_fw) / m_f
    z_f = (z_ff * m_ff - r_fw * m_fw) / m_f

    # Front assembly mass moments and products of inertia
    Fxx = Cxx + Dxx + m_ff * (z_ff - z_f) ** 2 + m_fw * (r_fw - z_f) ** 2
    Fxz = Cxz - m_ff * (x_ff - x_f) * (z_ff - z_f) + m_fw * (w - x_f) * (r_fw + z_f)
    Fzz = Czz + Dzz + m_ff * (x_ff - x_f) ** 2 + m_fw * (w - x_f) ** 2

    lam = pi / 2 - alpha

    # Perpendicular distance from front assembly COM to steering axis
    u = (x_f - w - t) * cos(lam) - z_f * sin(lam)

    Fll = m_f * u ** 2 + Fxx * sin(lam) ** 2 + 2 * Fxz * sin(lam) * cos(lam) + Fzz * cos(lam) ** 2
    Flx = -m_f * u * z_f + Fxx * sin(lam) + Fxz * cos(lam)
    Flz = m_f * u * x_f + Fxz * sin(lam) + Fzz * cos(lam)

    f = t * cos(lam) / w

    Sr = Ayy / r_rw
    Sf = Dyy / r_fw
    St = Sr + Sf

    Su = m_f * u + f * m_t * x_t

    M = np.array([[Txx, Flx + f * Txz],
                  [Flx + f * Txz, Fll + 2 * f * Flz + (f ** 2) * Tzz]])
    K0 = np.array([[g * m_t * z_t, -g * Su],
                   [-g * Su, -g * Su * sin(lam)]])
    K2 = np.array([[0, (St - m_t * z_t) * cos(lam) / w],
                   [0, (Su + Sf * sin(lam)) * cos(lam) / w]])
    C1 = np.array([[0, f * St + Sf * cos(lam) + Txz * cos(lam) / w - f * m_t * z_t],
                   [-(f * St + Sf * cos(lam)), Flz * cos(lam) / w + f * (Su + Tzz * cos(lam) / w)]])

    return M, C1, K0, K2
