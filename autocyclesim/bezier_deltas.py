from sympy import *
import re

class BezierDeltas:
    s = Symbol.symbols('s')

    def get_deltas(self):
        phi_sym = Symbol.symbols('phi')
        curve = open("../BezierAutocycle/output_data.txt", "r")
        matchcase = re.match('Matrix\(\[\[([\S]*)\],\s\[([\S]*)\]\]\)', curve.read())
        curve_parametrized = [eval(matchcase.group(1)), eval(matchcase.group(2))]

        curve_deltas = [[], []]
        step = 0.01  # decrease step size for greater precision
        for i in [1, 1 + step, step]:
            curve_deltas[100 * i][0] = curve_parametrized[0].subs(self.s, i)
            curve_deltas[100 * i][1] = 1.02 * cos(phi_sym) / \
                                       (cos(0.08) *
                                        (((Derivative(curve_parametrized[0], self.s).subs(self.s, i) ** 2 +
                                           Derivative(curve_parametrized[1], self.s).subs(self.s, i) ** 2) ** (3 / 2)) /
                                         abs(Derivative(curve_parametrized[0], self.s).subs(self.s, i) *
                                             Derivative(Derivative(curve_parametrized[1], self.s), self.s).subs(self.s,
                                                                                                                i) -
                                             Derivative(curve_parametrized[1], self.s).subs(self.s, i) *
                                             Derivative(Derivative(curve_parametrized[0], self.s), self.s).subs(self.s,
                                                                                                                i))))

        return curve_deltas
