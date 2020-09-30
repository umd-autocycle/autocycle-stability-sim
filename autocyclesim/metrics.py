from math import inf as infinity
from simulation import simulate
import numpy as np
import scipy.linalg

THRESHOLD = 0.001


def settling_threshold(time, variable, goal):
    return abs(max(variable[0],1) - goal) * 0.02


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
    return abs(variable[-1] - goal) <= threshold


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


def robustness_phi(init_parameters, velocity, model, control, timespan):
    phi, delta, d_phi, d_delta = init_parameters
    init_pkg = {'phi': phi, 'delta': delta, 'd_phi': d_phi, 'd_delta': d_delta}
    results = simulate(model, init_parameters, timespan, velocity, control, None,None)
    if not settles(results['t'], results['phi'], 0):
        return 0
    start = phi
    # find the max
    lowbd = start
    upbd = start + 30
    max_var = (lowbd + upbd) / 2
    while (abs(upbd) - abs(lowbd)) > 1e-2:
       init_parameters = max_var, init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
       results = simulate(model, init_parameters, timespan, velocity, control, None,None)
       if settles(results['t'], results['phi'], 0):
           lowbd = max_var
       else:
            upbd = max_var - 1
       max_var = (lowbd + upbd) / 2
    # find the min
    upbd = start
    lowbd = start - 30
    min_var = (lowbd + upbd) / 2
    while (abs(lowbd) - abs(upbd)) > 1e-2:
        init_parameters = min_var, init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
        results = simulate(model, init_parameters, timespan, velocity, control, None)
        if settles(results['t'], results['phi'], 0):
            upbd = min_var
        else:
            lowbd = min_var
        min_var = (lowbd + upbd) / 2

    return max_var - min_var

def robustness_delta(init_parameters, velocity, model, control, timespan,goals):
    phi, delta, d_phi, d_delta = init_parameters
    init_pkg = {'phi': phi, 'delta': delta, 'd_phi': d_phi, 'd_delta': d_delta}
    results = simulate(model, init_parameters, timespan, velocity, control, None,goal=goals)
    if not settles(results['t'], results['delta'], goals[1]):
        return 0
    ################################continue here  start =
    # find the max
    lowbd = start
    upbd = start + 30
    max_var = (lowbd + upbd) / 2
    while (abs(upbd) - abs(lowbd)) > 1e-2:
       init_parameters = max_var, init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
       results = simulate(model, init_parameters, timespan, velocity, control, None)
       if settles(results['t'], results['phi'], 0):
           lowbd = max_var
       else:
            upbd = max_var - 1
       max_var = (lowbd + upbd) / 2
    # find the min
    upbd = start
    lowbd = start - 30
    min_var = (lowbd + upbd) / 2
    while (abs(lowbd) - abs(upbd)) > 1e-2:
        init_parameters = min_var, init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
        results = simulate(model, init_parameters, timespan, velocity, control, None)
        if settles(results['t'], results['phi'], 0):
            upbd = min_var
        else:
            lowbd = min_var
        min_var = (lowbd + upbd) / 2

    return max_var - min_var


def robust(init_parameters, velocity, model, control, timespan):
    phi, delta, d_phi, d_delta = init_parameters
    init_pkg = {'phi': phi, 'delta': delta, 'd_phi': d_phi, 'd_delta': d_delta}
    ret = {}
    results = simulate(model, init_parameters, timespan, velocity, control, None)
    if not settles(results['t'], results['phi'], 0):
        return {'phi': 0, 'delta': 0, 'd_phi': 0, 'd_delta': 0}

    for vary in init_pkg.keys():
        # find the max
        start = init_pkg[vary]
        lowbd = start
        upbd = start + 30
        max_var = (lowbd + upbd) / 2
        while (abs(upbd) - abs(lowbd)) > 1e-2:
            init_pkg = {'phi': phi, 'delta': delta, 'd_phi': d_phi, 'd_delta': d_delta, vary: max_var}
            init_parameters = init_pkg['phi'], init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
            results = simulate(model, init_parameters, timespan, velocity, control, None)
            if settles(results['t'], results['phi'], 0):
                lowbd = max_var
            else:
                upbd = max_var - 1
            max_var = (lowbd + upbd) / 2
        # find the min
        upbd = start
        lowbd = start - 30
        min_var = (lowbd + upbd) / 2
        while (abs(lowbd) - abs(upbd)) > 1e-2:
            init_pkg = {'phi': phi, 'delta': delta, 'd_phi': d_phi, 'd_delta': d_delta, vary: min_var}
            init_parameters = init_pkg['phi'], init_pkg['delta'], init_pkg['d_phi'], init_pkg['d_delta']
            results = simulate(model, init_parameters, timespan, velocity, control, None)
            if settles(results['t'], results['phi'], 0):
                upbd = min_var
            else:
                lowbd = min_var
            min_var = (lowbd + upbd) / 2

        ret[vary] = max_var - min_var

    return ret





def lqr(A, B, Q, R):
    """Solve the continuous time lqr controller.

    dx/dt = A x + B u

    cost = integral x.T*Q*x + u.T*R*u
    """


# ref Bertsekas, p.151

# first, try to solve the ricatti equation
    X = np.array(scipy.linalg.solve_continuous_are(A, B, Q, R))

# compute the LQR gain
    print((B.T * X))
    #K = scipy.linalg.inv(R) * (B.T * X)
    K = (B.T * X)

    eigVals, eigVecs = scipy.linalg.eig(A - B * K)
    #print(eigVals)
    return K, X, eigVals


def dlqr(A, B, Q, R):

    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

    K = np.matrix(scipy.linalg.inv(B.T * X * B + R) * (B.T * X * A))

    eigVals, eigVecs = scipy.linalg.eig(A - B * K)
    return K, X, eigVals


def max_torque(torque):
    max(torque)
