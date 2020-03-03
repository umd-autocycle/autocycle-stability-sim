from controls import PIDPhi
from simulation import simulate
from scipy.optimize import minimize
from bikemodel import MeijaardModel
import metrics

def optimize(intial_val, vel, tspan, intial_constants, max_T, focus):
    def func(x):
        return simulate(MeijaardModel, intial_val, tspan, vel, PIDPhi(x), None)

    def con_time(x):
        results = func(x)
        con = float(metrics.settling_time(results['t'], results['phi'], 0))
        return con
    def con_robust(x):
        results = metrics.robustness(x, MeijaardModel, PIDPhi, 0)
        con = results[0,0] - results[0,1]
        return con
    def con_torque(x):
        con = metrics.max_torque(range_torque)-max_T #not sure how to get a range of torques
        return con
    def objective(x):
        if focus == 'Time':
            return con_time(x)
        elif focus == 'Robust':
            return con_robust(x)
    def con1(x):
        if focus == 'Time':
            return con_robust(x)
        elif focus == 'Robust':
            return con_time(x)


    cons = [{'type':'eq', 'fun':con1}, {'type': 'ineq', 'fun': con_torque}]

    res = minimize(objective, intial_constants, constraints=cons)

    return res.x
