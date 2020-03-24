from controls import PIDPhi
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics
from math import inf as infinity
import numpy as np


def optimize(intial_val, vel, tspan, intial_constants, c_robust, c_response, max_torque, max_response, min_robust):
    def func(x):
        return simulate(MeijaardModel(), intial_val, tspan, vel,
                        PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2], max_torque=max_torque), None)

    def time(x):
        results = func(x)
        con = float(metrics.settling_time(results['t'], results['phi'], 0))
        if con > tspan:
            con = (tspan + abs(metrics.overshoot(results['t'], results['phi'], 0)[0])) * c_response
        #     print('over', (con/c_response) - tspan)
        # else:
        #     print('time', con)
        return con

    # def robust(x):
    #     results = metrics.robustness(intial_val, vel, MeijaardModel(), PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2]), tspan)
    #     con = results['phi']
    #     print('robust', con)
    #     return con

    def torque(x):
        return max(func(x)['torque'])

    def objective(x):
        print('iteration:', x)
        time_obj = (time(x) - max_response)
        # robust_obj = (robust(x) - min_robust) ** 2
        robust_obj = 0
        tor = torque(x)
        print('max torque:', tor)
        torque_obj = 0  # 10000 ** (tor - max_torque)
        return c_response * time_obj + c_robust * robust_obj + torque_obj

    res = minimize(objective, intial_constants, method='Powell')
    print(res.success)
    return res.x, time(res.x)


if __name__ == '__main__':
    test = optimize(intial_val=[10, 0, 0, 0], vel=15, tspan=60,
                    intial_constants=[0, 0, 0], c_robust=1, c_response=1e2,
                    max_torque=20, max_response=0, min_robust=10)
    print(test)
    # 5.5 4.34s [ 3.44077315e+00, -2.23763598e-06,  8.66977482e-02] then @100 3.12s [ 8.38025928e+00, -1.32098845e-06,  3.49506937e+00]
    # 10  29s [ 1.19326220e+00, -6.49562030e-06,  1.31230247e+00] then @100 t 3.36s [ 9.76156844e+00, -9.45659175e-07, -4.11625537e-01]
    # 11 29.72s [ 1.31204884e+00, -6.51253072e-06,  1.31193448e+00] then @100t 3.84s [ 9.78287177e+00, -8.90521735e-07, -4.15691395e-01]
    # c_array = []
    # for v in np.arange(4, 15, .5):
    #     test = optimize(intial_val=[10, 0, 0, 0], vel=v, tspan=30,
    #                     intial_constants=[0, 0, 0], c_robust=1, c_response=1e2,
    #                     max_torque=100, max_response=0, min_robust=10)
    #     c_array.append(test)
    #     print(v, test[1])
    # print(c_array)
