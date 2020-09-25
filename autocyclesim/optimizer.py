from controls import PIDPhi, PIDDelta
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics
from math import inf as infinity
import numpy as np


def optimize(intial_val, vel, tspan, intial_constants, c_robust, c_response, c_fail, max_torque, max_response, min_robust):
    goals = np.radians([45,10])
    def func(x):
        return simulate(MeijaardModel(), intial_val, tspan, vel,
                        PIDDelta(k_p=x[0], k_i=x[1], k_d=x[2], max_torque=max_torque), None, goal=goals)

    def time(x):
        results = func(x)
        con = float(metrics.settling_time(results['t'], np.radians(results['delta']), goals[1]))
        if con > tspan:
            con = (tspan + abs(metrics.overshoot(results['t'], np.radians(results['delta']), goals[1])[0])) * c_response
        #     print('over', (con / c_response) - tspan)
        # else:
        #     print('time', con)
        return con

    def robust(x):

        return 0
        con = metrics.robustness_phi(intial_val, vel, MeijaardModel(),
                                 PIDDelta(k_p=x[0], k_i=x[1], k_d=x[2], max_torque=max_torque), tspan)
        return con

    def fail(x):
        results = func(x)
        if(np.radians(results['phi'][-1])>goals[0]):
            con = results['phi'][-1]+goals[0]
        else:
            con=0
        return con

    def torque(x):
        return max(func(x)['torque'])

    def objective(x):
        time_obj = (time(x) - max_response)
        #robust_x = robust(x)
        #robust_obj = (robust_x - min_robust) ** 2 - (min_robust - max(min_robust, robust_x)) ** 2
        robust_obj = 0
        tor = torque(x)
        print('iteration:', x,'time',time_obj,'torque',tor)
        # print('max torque:', tor)
        torque_obj = max(0, tor - (max_torque - 2))
        return c_response * time_obj + c_robust * robust_obj + 10 ** torque_obj + c_fail * fail(x)

    res = minimize(objective, intial_constants, method='Powell', options={'xtol': 1e-2, 'ftol': 1e-1})
    # print(res.success)
    return res.x, time(res.x), robust(res.x)


if __name__ == '__main__':
    test = optimize(intial_val=[0, 0, 0, 0], vel=4.5, tspan=30,
                    intial_constants=[-46.03587539,   1.12907121,   0.83658329], c_robust=10, c_response=1e5, c_fail=1e10,
                    max_torque=40, max_response=0, min_robust=50)
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
