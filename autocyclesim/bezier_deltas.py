from sympy import *
import re

class BezierDeltas:
    s = Symbol.symbols('s')

    def read_curve(self, st):
        if ()

    def get_deltas(self):
        curve = open("../BezierAutocycle/output_data.txt", "r")
        matchcase = re.match('Matrix\(\[\[([\S]*)\],\[([\S]*)\]\]\)', curve.read().replace(" ", ""))
        curve_parametrized = [self.read_curve(matchcase.group(1)), self.read_curve(matchcase.group(2))]

        curve_deltas = [[], []]
        for i in [1, 1.1, 0.01]:
            curve_deltas[100 * i][0] = curve_parametrized[0].subs(self.s, i)
            curve_deltas[100 * i][1] = wheelbase * cos(lean_angle) /\
                                       (cos(caster_angle) *
                                        (((Derivative(curve_parametrized[0], self.s).subs(self.s, i) ** 2 +
                                           Derivative(curve_parametrized[0], self.s).subs(self.s, i) ** 2) ** (3 / 2)) /
                                         abs(Derivative(curve_parametrized[0], self.s).subs(self.s, i) *
                                             Derivative(Derivative(curve_parametrized[1], self.s), self.s).subs(self.s, i) -
                                             Derivative(curve_parametrized[1], self.s).subs(self.s, i) *
                                             Derivative(Derivative(curve_parametrized[0], self.s), self.s).subs(self.s, i)
                                             )))

        return curve_parametrized
