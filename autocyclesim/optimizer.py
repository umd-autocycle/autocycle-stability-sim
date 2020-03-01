from controls import PIDPhi
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics
import numpy as np
from scipy.optimize import LinearConstraint

def optimize(intial_val, vel, tspan, intial_constants, max_T):
    def func(x):
        return simulate(MeijaardModel, intial_val, tspan, vel, PIDPhi(x), None)

    def objective(x):
        results = func(x)
        obj = float(metrics.settling_time(results['t'], results['phi'], 0))
        return obj
    def con_robust(x):
        results = metrics.robustness(x, MeijaardModel, PIDPhi, 0)
        con = results[0,0] - results[0,1]
        return con
    def con_torque(x):
        results = func(x)

        con = -metrics.max_torque(p)
        return con

    con1 = {'type':'eq','fun':con_robust}
    con2 = {'type': 'ineq', 'fun': con_torque}
    cons = [con1,con2]

    res = minimize(objective, intial_constants,constraints=cons)

    return res.x


out P I D