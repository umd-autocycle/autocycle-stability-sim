# apply torque to bike model -> new state vector -> output theoretical sensor readings
import bikemodel
import numpy as np


class HILsim:
    def __init__(self, state_vector):  # [phi, delta, dphi, ddelta]
        self.state_vector = state_vector

    def apply_torque(self, torque, v):
        a = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [9.4702, -0.5888 - 0.8868 * v * v, -0.104 * v, -0.3277 * v],
                      [12.3999, 31.5587 - 2.0423 * v * v, 3.6177 * v, -3.1388 * v]])
        b = np.array([[0], [0], [-0.1226], [4.265]])
        x = np.array(self.state_vector)
        u = np.array([torque])
        xdot = a.dot(x) + b.dot(u)  # compute xdot using A, B, x, u
        t = 1 / 20
        new_state_vector = [self.state_vector[0] + xdot[0] * t + 1 / 2 * xdot[2] ** 2,
                            self.state_vector[1] + xdot[1] * t + 1 / 2 * xdot[3] ** 2,
                            xdot[0] + xdot[2] * t, xdot[1] + xdot[3] * t]
        self.state_vector = new_state_vector

        # use the new state vector to output theoretical sensor readings (need to find which sensors to use)
        # turn angle sensor, X, Y, Z, acceleration
        return sensor_readings
