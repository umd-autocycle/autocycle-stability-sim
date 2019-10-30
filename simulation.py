import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from sympy import *
import sympy as sp

'''m11, m12, m21, m22, c11, c12, c21, c22, k11, k12, k21, k22 = sp.symbols('m11 m12 m21 m22 c11 c12 c21 c22 k11 k12 k21 k22')
M = sp.Matrix([[m11, m12], [m21, m22]])
C = sp.Matrix([[c11, c12], [c21, c22]])
K = sp.Matrix([[k11, k12], [k21, k22]])
'''
M = sp.Matrix([[1, 2], [3, 4]])
C = sp.Matrix([[1, 2], [3, 4]])
K = sp.Matrix([[1, 2], [3, 4]])

'''from scipy.integrate import solve_ivp
def rhs(s, v):
    return [-12*v[2]**2, 12*v[2]**2, 6*v[0]*v[2] - 6*v[2]*v[1] - 36*v[2]]
res = solve_ivp(rhs, (0, 0.1), [2, 3, 4])
print(res.t)
print(res.y)'''

t = sp.symbols('t')
y0 = Function('y0')(t)
y1 = Function('y1')(t)
y = sp.Matrix([y0, y1])
dy = sp.Matrix([y0.diff(t), y1.diff(t)])
d2y = sp.Matrix([y0.diff(t,2), y1.diff(t,2)])


diffeq = M*d2y+C*dy+K*y
print(Eq(diffeq[0], 0))
print(Eq(diffeq[1], 0))
print([y[0], y[1]])
print(dsolve([Eq(diffeq[0], 0), Eq(diffeq[1], 0)], [y0, y1]))

'''
x = sp.Function('x')(t)
y = sp.Function('y')(t)
diffeq1 = Eq(x.diff(t, 2) + y.diff(t, 2) + x.diff(t) + y.diff(t) + x + y, 0)
print('almost')
#diffeq2 = Eq()
eq1 = dsolve(diffeq1, x, y)
print(eq1)
'''


