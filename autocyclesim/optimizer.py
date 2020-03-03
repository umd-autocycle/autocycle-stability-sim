from controls import PIDPhi
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics

def optimize(intial_val, vel, tspan, intial_constants, focus, max_T, max_response, min_robust):
    def func(x):
        return simulate(MeijaardModel, intial_val, tspan, vel, PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2]), None)

    def con_time(x):
        results = func(x)
        con = metrics.settling_time(results['t'], results['phi'], 0)
        return con

    def con_robust(x):
        results = metrics.robustness(intial_val, vel, MeijaardModel, PIDPhi(k_p=x[0], k_i=x[1], k_d=x[2]))
        con = results['phi']
        return con

    def con_torque(x):
        results = func(x)
        range_T = PIDPhi.get_control
        con = metrics.max_torque(range_T)-max_T #not sure how to get a range of torques
        return con

    def objective(x):
        if focus == 'Time':
            return con_time(x)
        elif focus == 'Robust':
            return con_robust(x)

    def con1(x):
        if focus == 'Time':
            return con_robust(x) - min_robust
        elif focus == 'Robust':
            return max_response - con_time(x)


    cons = [{'type':'ineq', 'fun':con1}, {'type': 'ineq', 'fun': con_torque}]

    res = minimize(objective, intial_constants, constraints=cons)

    return res.x
