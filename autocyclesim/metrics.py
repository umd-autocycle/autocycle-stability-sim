from math import inf as infinity
from simulation import simulate

THRESHOLD = 0.001


def settling_threshold(time, variable, goal):
    return abs(variable[0] - goal) * 0.02


def settling_time(time, variable, goal):
    threshold = settling_threshold(time, variable, goal)

    if not settles(time, variable, goal):
        return infinity
    else:
        for t, v in zip(reversed(time), reversed(variable)):
            if abs(v - goal) >= threshold:
                return t - time[0]

    return 0


def settles(time, variable, goal):
    """
    does not account well for oscillating behavior, need more sophisticated approach
    """
    threshold = settling_threshold(time, variable, goal)
    return abs(variable[-1] - goal) < threshold


def overshoot(time, variable, goal):
    if variable[0] < goal:
        max_over = variable[0] - goal
        max_t = 0

        for v, t in zip(variable, time):
            if v - goal > max_over:
                max_over = v - goal
                max_t = t

    else:
        max_over = variable[0] - goal
        max_t = 0

        for v, t in zip(variable, time):
            if v - goal < max_over:
                max_over = v - goal
                max_t = t

    return max_over, max_t


def response_time(time, variable, goal):
    for v, t in zip(variable, time):
        if abs(v - goal) < THRESHOLD:
            return t

    return time[-1]


def robustness(init_parameters, velocity, model, control):
    phi, delta, d_phi, d_delta = init_parameters
    to_vary = phi, delta, d_phi, d_delta, velocity

    for vary in to_vary:
        results = simulate(model, init_parameters, 60, velocity, control, None)
        if settles(results['t'], results['phi'], 0):
            pass


def max_torque(torque):
    max(torque)
