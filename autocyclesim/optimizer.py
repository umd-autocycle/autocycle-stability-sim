from controls import PIDPhi, PIDDelta
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics
from math import inf as infinity
import numpy as np


def optimize(intial_val, vel, tspan, intial_constants, c_robust, c_response, max_torque, max_response, min_robust):
    goals = [0,20]
    def func(x):
        return simulate(MeijaardModel(), intial_val, tspan, vel,
                        PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2], max_torque=max_torque), None, goal=goals)

    def time(x):
        results = func(x)
        con = float(metrics.settling_time(results['t'], results['delta'], goals[1]))
        if con > tspan:
            con = (tspan + abs(metrics.overshoot(results['t'], results['delta'], goals[1])[0])) * c_response
        #     print('over', (con / c_response) - tspan)
        # else:
        #     print('time', con)
        return con

    def robust(x):
        con = metrics.robustness(intial_val, vel, MeijaardModel(),
                                 PIDDelta(k_p=x[0], k_i=x[1], k_d=x[2], max_torque=max_torque), tspan)
        return con

    def torque(x):
        return max(func(x)['torque'])

    def objective(x):
        time_obj = (time(x) - max_response)
        robust_x = robust(x)
        robust_obj = (robust_x - min_robust) ** 2 - (min_robust - max(min_robust, robust_x)) ** 2
        tor = torque(x)
        # print('iteration:', x,'time',time_obj,'robust',robust_x)
        # print('max torque:', tor)
        torque_obj = max(0, tor - (max_torque - 2))
        return c_response * time_obj + c_robust * robust_obj + 10 ** torque_obj

    res = minimize(objective, intial_constants, method='Powell', options={'xtol': 1e-2, 'ftol': 1e-1})
    # print(res.success)
    return res.x, time(res.x), robust(res.x)


if __name__ == '__main__':
    test = optimize(intial_val=[10, 0, 0, 0], vel=7.5, tspan=30,
                    intial_constants=[104.35433225, 6.18809353, 27.85102572], c_robust=10, c_response=1e2,
                    max_torque=20, max_response=0, min_robust=50)
    print(test)

    # test = metrics.robustness([10, 0, 0, 0], 5.5,MeijaardModel(),PIDPhi(104.35433225,   6.18809353,  27.85102572, 20),30)
    # print(test)

    # c_array = []
    # init_k = [[40, 1.5, 15], ]
    # for v in np.arange(4, 15.5, .5):
    #     test = optimize(intial_val=[10, 0, 0, 0], vel=v, tspan=30,
    #                     intial_constants=init_k[-1], c_robust=10, c_response=1e2,
    #                     max_torque=20, max_response=0, min_robust=10)
    #     c_array.append(test)
    #     init_k.append(list(test[0]))
    #     print(v, test[1])
    # print(c_array)
