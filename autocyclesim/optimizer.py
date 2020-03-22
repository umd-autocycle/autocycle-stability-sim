from controls import PIDPhi
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics
from math import inf as infinity
import numpy as np


def optimize(intial_val, vel, tspan, intial_constants, c_robust, c_response, max_torque, max_response, min_robust):
    i = 0
    def func(x):
        return simulate(MeijaardModel(), intial_val, tspan, vel, PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2]), None)

    def time(x):
        results = func(x)
        con = float(metrics.settling_time(results['t'], results['phi'], 0))
        print('time', con)
        return con

    # def robust(x):
    #     results = metrics.robustness(intial_val, vel, MeijaardModel(), PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2]), tspan)
    #     con = results['phi']
    #     print('robust', con)
    #     return con

    def torque(x):
        results = func(x)
        range_torque = []
        controller = PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2])
        for i in range(0, np.size(results['phi'])-1):
            e = [results['phi'][i], results['delta'][i], results['dphi'][i], results['ddelta'][i]]
            temp = controller.get_control(goals=None)(t=results['t'][i], e=e, v=vel)
            range_torque.append(temp)
        #print('torque', range_torque[0], max(range_torque), range_torque.index(max(range_torque)))
        con = 1*(max(range_torque) - max_torque)
        return con

    def objective(x):
        print('iteration:', x)
        time_obj = (time(x) - max_response) ** 2
        # robust_obj = (robust(x) - min_robust) ** 2
        robust_obj = 0
        tor = torque(x)
        print(tor)
        if tor <= max_torque:
            torque_obj = 0
        else:
            torque_obj = infinity
        return c_response * time_obj + c_robust * robust_obj + torque_obj

    # cons = {'type': 'ineq', 'fun': con_torque}
    # bon = [[0, 300], [0, 10], [0, 300]]
    # opt = {'maxiter': 100, 'disp': True}

    res = minimize(objective, intial_constants, method='Powell')
    print(res.success)
    return res.x



if __name__ == '__main__':
    test = optimize(intial_val=[10, 0, 0, 0], vel=5.5, tspan=10, intial_constants=[0, 0, 0], c_robust=1, c_response=5,
             max_torque=20, max_response=0, min_robust=10)
    print(test)
    print('Powell at v=5.5')
