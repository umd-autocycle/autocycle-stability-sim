# apply torque to bike model -> new state vector -> output theoretical sensor readings
import bikemodel
import numpy as np
import bikemodel
import serial


class HILsim:
    def __init__(self, state_vector):
        self.state_vector = state_vector

    def get_sensor_readings(self, v):
        ser = serial.Serial('COM1', 9600, timeout=0)
        torque = serial.read()

        xdot = bikemodel.MeijaardModel.linearized_1st_order(v, [0, torque])
        t = 1 / 20
        new_state_vector = [self.state_vector[0] + xdot[0] * t + 1 / 2 * xdot[2] * t ** 2,
                            self.state_vector[1] + xdot[1] * t + 1 / 2 * xdot[3] * t ** 2,
                            xdot[0] + xdot[2] * t, xdot[1] + xdot[3] * t]
        self.state_vector = new_state_vector

        # use the new state vector to output theoretical sensor readings
        x_accel = 0
        y_accel = 0
        z_accel = xdot[2] / np.sin(self.state_vector[0])
        x_gyro = self.state_vector[2]
        y_gyro = 0
        z_gyro = v * self.state_vector[1] * np.cos(0.08) / (1.02 * np.cos(self.state_vector[0]))
        ser.write("[" + str(x_accel) + ", " + str(y_accel) + ", " + str(z_accel) + ", "
                  + str(x_gyro) + ", " + str(y_gyro) + ", " + str(z_gyro) + ", "
                  + str(v) + ", " + str(self.state_vector[1]) + ", " + str(self.state_vector[3]) + "]")
        ser.close()
