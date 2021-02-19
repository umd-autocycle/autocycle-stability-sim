# apply torque to bike model -> new state vector -> output theoretical sensor readings
import bikemodel
import numpy as np
import bikemodel
import serial


class HILsim:
    def __init__(self, state_vector):
        self.state_vector = state_vector
        self.t = 1 / 20
        self.r_ao = [1, 1]

    def get_sensor_readings(self, v):
        ser = serial.Serial('COM6', 115200, timeout=0)  # will only work if speeds are capped
        torque = ser.readline().decode('utf-8')

        model = bikemodel.MeijaardModel()
        xdot = (model.linearized_1st_order(v, [None, lambda t, e, v: torque]))(0, self.state_vector)
        self.state_vector = [self.state_vector[0] + xdot[0] * self.t + 1 / 2 * xdot[2] * self.t ** 2,
                             self.state_vector[1] + xdot[1] * self.t + 1 / 2 * xdot[3] * self.t ** 2,
                             xdot[0] + xdot[2] * self.t, xdot[1] + xdot[3] * self.t]

        # use the new state vector to output theoretical sensor readings
        x_accel = 0
        y_accel = xdot[2] * self.r_ao[0] + 9.8 * np.cos(self.state_vector[0])
        z_accel = -self.state_vector[2] * self.r_ao[1] - 9.8 * np.sin(self.state_vector[0])
        x_gyro = self.state_vector[2]
        y_gyro = 0
        z_gyro = v * self.state_vector[1] * np.cos(0.08) / (1.02 * np.cos(self.state_vector[0]))
        ser.write("[" + str(x_accel) + ", " + str(y_accel) + ", " + str(z_accel) + ", "
                  + str(x_gyro) + ", " + str(y_gyro) + ", " + str(z_gyro) + ", "
                  + str(v) + ", " + str(self.state_vector[1]) + ", " + str(self.state_vector[3]) + "]")
        ser.close()
