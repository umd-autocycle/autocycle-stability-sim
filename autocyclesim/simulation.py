from scipy.integrate import solve_ivp
from math import pi, radians, degrees
import numpy as np
from math import degrees, radians


# initial conditions and return in degrees
def simulate(bike_model, initial_conditions, length, velocity, control_method, perturbation):
    initial_conditions = [radians(x) for x in initial_conditions]
    t_eval = np.arange(0, length, 0.01)
    f = np.array([perturbation, None if control_method is None else control_method.get_control(None)])
    results = solve_ivp(bike_model.linearized_1st_order(velocity, f), [0, length], initial_conditions, t_eval=t_eval)

    ret = {
        't': results.t,
        'phi': [degrees(x) for x in results.y[0]],
        'delta': [degrees(x) for x in results.y[1]],
        'dphi': [degrees(x) for x in results.y[2]],
        'ddelta': [degrees(x) for x in results.y[3]],
    }

    # ret['delta'] = [min(pi, max(x, -pi)) for x in ret['delta']]
    # ret['phi'] = [min(pi/2, max(x, -pi/2)) for x in ret['phi']]

    return ret
