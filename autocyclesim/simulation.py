from scipy.integrate import solve_ivp
from math import pi
import numpy as np


def simulate(bike_model, initial_conditions, length, velocity):
    t_eval = np.arange(0, length, 0.01)
    results = solve_ivp(bike_model.linearized_1st_order(velocity, np.array([0.0, 0.0])), [0, length],
                        initial_conditions, t_eval=t_eval)

    ret = {
        't': results.t,
        'delta': results.y[1],
        'ddelta': results.y[3],
        'phi': results.y[0],
        'dphi': results.y[2],
    }

    # ret['delta'] = [min(pi, max(x, -pi)) for x in ret['delta']]
    # ret['phi'] = [min(pi/2, max(x, -pi/2)) for x in ret['phi']]

    return ret
