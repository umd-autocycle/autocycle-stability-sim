from math import pi, radians, cos, sin
import pandas as pd
import numpy as np

mois = {}


def moi_m(period, rigidity, d, length):
    polar_area_moment = pi / 2 * ((d / 2) ** 4)

    return (period / (2 * pi)) ** 2 * (rigidity * polar_area_moment / length)


if __name__ == '__main__':
    frame = pd.read_csv('MOI-Data/NewMoIData.tsv', sep='\t')
    print(frame)

    frame.transform({
        'Angle from Axis to Horizontal': lambda x: radians(float(x)),
        'Angle from Axis to Forward': lambda x: radians(float(x)),
        'Rod Length': lambda x: float(x),
        'Rod Rigidity': lambda x: float(x),
        'Rod Diameter': lambda x: float(x),
        'Period': lambda x: float(x)
    })

    for i, subset in frame.groupby('Component'):
        coeff_rows = []
        measured_moi = []

        for j, row in subset.iterrows():
            theta = row['Angle from Axis to Horizontal']
            psi = row['Angle from Axis to Forward']
            axis = np.array([cos(theta) * cos(psi), sin(psi), sin(theta) * cos(psi)])

            p = row['Period']
            r = row['Rod Rigidity']
            diam = row['Rod Diameter']
            long = row['Rod Length']

            coeff_rows.append([axis[0] ** 2, axis[1] ** 2, axis[2] ** 2, 2 * axis[0] * axis[2]])
            measured_moi.append(moi_m(p, r, diam, long))

        big_matrix = np.array(coeff_rows)
        mois = np.array(measured_moi)

        i_xx, i_yy, i_zz, i_xz = np.linalg.solve(big_matrix, mois)
