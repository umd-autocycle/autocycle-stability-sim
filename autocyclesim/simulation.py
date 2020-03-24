from scipy.integrate import solve_ivp
from math import pi, radians, degrees
import numpy as np
from math import degrees, radians
from controls import Control


# initial conditions and return in degrees
def simulate(bike_model, initial_conditions, length, velocity, control_method, perturbation):
    if control_method is None:
        control_method = Control()
    initial_conditions = [radians(x) for x in initial_conditions]
    t_eval = np.arange(0, length, 0.01)
    f = np.array([perturbation, control_method.get_control(None)])
    results = solve_ivp(bike_model.linearized_1st_order(velocity, f), [0, length], initial_conditions, t_eval=t_eval)

    torque = []
    control_method.reset_registers()
    for i in range(len(results.y[0])):
        e = [results.y[0][i], results.y[1][i], results.y[2][i], results.y[3][i]]
        temp = control_method.get_control(goals=None)(t=results.t[i], e=e, v=velocity)
        torque.append(temp)

    ret = {
        't': results.t,
        'phi': [degrees(x) for x in results.y[0]],
        'delta': [degrees(x) for x in results.y[1]],
        'dphi': [degrees(x) for x in results.y[2]],
        'ddelta': [degrees(x) for x in results.y[3]],
        'torque': torque,
    }

    # ret['delta'] = [min(pi, max(x, -pi)) for x in ret['delta']]
    # ret['phi'] = [min(pi/2, max(x, -pi/2)) for x in ret['phi']]

    return ret
