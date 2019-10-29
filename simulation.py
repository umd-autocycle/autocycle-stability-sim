import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from sympy import *
import sympy as sp

M = np.array([[1, 2], [3, 4]])
C = np.array([[1, 2], [3, 4]])
K = np.array([[1, 2], [3, 4]])


'''from scipy.integrate import solve_ivp
def rhs(s, v):
    return [-12*v[2]**2, 12*v[2]**2, 6*v[0]*v[2] - 6*v[2]*v[1] - 36*v[2]]
res = solve_ivp(rhs, (0, 0.1), [2, 3, 4])
print(res.t)
print(res.y)'''

t = sp.symbols('t')
y0 = Function('y0')(t)
y1 = Function('y1')(t)
y = np.array([y0, y1])
dy = np.array([y0.diff(t), y1.diff(t)])
d2y = np.array([y0.diff(t,2), y1.diff(t,2)])

diffeq = M*d2y+C*dy+K*y
#print(Eq(diffeq[0][0], 0))

print(dsolve([Eq(diffeq[0][0], 0), Eq(diffeq[1][0], 0)],y))

'''
x = sp.Function('x')(t)
y = sp.Function('y')(t)
diffeq1 = Eq(x.diff(t, 2) + y.diff(t, 2) + x.diff(t) + y.diff(t) + x + y, 0)
print('almost')
#diffeq2 = Eq()
eq1 = dsolve(diffeq1, x, y)
print(eq1)
'''


