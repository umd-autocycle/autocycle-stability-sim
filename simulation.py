import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from sympy import *
import sympy as sp

M = np.array([[1, 2], [3, 4]])
C = np.array([[1, 2], [3, 4]])
K = np.array([[1, 2], [3, 4]])

t = sp.symbols('t')
y0 = Function('y0')(t)
y1 = Function('y1')(t)
y = np.array([y0, y1])

diffeq = Eq(M*y.diff(t,2))

'''
x = sp.Function('x')(t)
y = sp.Function('y')(t)
diffeq1 = Eq(x.diff(t, 2) + y.diff(t, 2) + x.diff(t) + y.diff(t) + x + y, 0)
print('almost')
#diffeq2 = Eq()
eq1 = dsolve(diffeq1, x, y)
print(eq1)
'''


