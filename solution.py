from dataclasses import dataclass
from time import perf_counter
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np

from libs.can import CANSocket
from libs.motors.gyems import GyemsDRC


@dataclass()
class DeviceParams:
    """
    Dataclass to collect the base parameters of the system
    """
    kp: float
    kd: float
    q_def: float
    dq_def: float
    current_limit: float
    time_stop: float


class Model:

    def __init__(self, device: GyemsDRC, device_params: DeviceParams) -> None:
        """
        Class model to show the work of PD Regulator for the motor
        :param device: Class object of the motor
        :param device_params: Parameters of the system with pd coefficients and def values
        """
        self.device = device
        self.kp = device_params.kp
        self.kd = device_params.kd
        self.initial_velocity = device_params.dq_def
        self.q_def = device_params.q_def
        self.time_stop = device_params.time_stop

        self.setup(device_params.current_limit)
        self.state = []
        self.error = []
        self.time = []

    def setup(self, current_limit: float) -> None:
        """
        Setup function, init the motor parameters and turn on it
        :param current_limit: current limitof the motor
        :return: None
        """
        self.device.set_degrees()
        self.device.current_limit = current_limit
        self.device.enable()

    def get_params(self) -> Tuple[float, float]:
        """
        Function to calculate the Moment of inertia and Friction coefficient
        :return: Tuple["Moment of inertia", "Friction coefficient"]
        """
        t, state = np.asarray(self.time), np.asarray(self.state)
        q, dq, I = state[:, 0], state[:, 1], state[:, 2]
        ddq = np.diff(dq) / np.diff(t)
        dq, t, I = dq[1:], t[1:], I[1:]
        state = np.asarray([dq, ddq])
        I = np.reshape(I, (len(I), 1))

        params = I.T @ np.linalg.pinv(state)
        return params[0, 0], params[0, 1]

    def pd_regulator(self) -> Tuple[float, float]:
        """
        PD Regulator
        :return: Tuple["Position Error", "Current def"]
        """
        q, dq = self.device.state["angle"], self.device.state["speed"]
        q_des, dq_des = self.q_def, self.initial_velocity
        u = self.kp * (q_des - q) + self.kd * (dq_des - dq)
        self.device.set_current(u)
        self.state.append([q, dq, self.device.state['current'], u])
        return q_des - q, u

    def plot(self) -> None:
        """
        Function to plot the results of the training
        :return: None
        """
        t, state = np.asarray(self.time), np.asarray(self.state)
        q, dq, I, I_des = state[:, 0], state[:, 1], state[:, 2], state[:, 3]
        f, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 15))
        axes = axes.ravel()

        axes[0].plot(t, q, linewidth=1, color='blue')
        axes[0].set(ylabel='Motor angle [deg]', xlabel='Time [sec]')
        axes[1].plot(t, dq, linewidth=1, color='red')
        axes[1].set(ylabel='Velocity [rad/sec]', xlabel='Time [sec]')
        axes[2].plot(t, I, linewidth=1, color='blue', label='Actual control')
        axes[2].plot(t, I_des, linewidth=1, color='red', label='Desired control')
        axes[2].set(ylabel='Current [units]', xlabel='Time [sec]')
        axes[2].legend()

        for i in range(3):
            axes[i].grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.show()

        J, B = self.get_params()
        print("Moment of inertia:", J, "units")
        print("Friction coefficient:", B, "units")

    def simulate(self) -> None:
        """
        Base function to simulate the PD Regulator for the motor
        :return: None
        """
        t0 = perf_counter()
        t = 0
        try:
            while t < self.time_stop:
                t = perf_counter() - t0
                e, I_def = self.pd_regulator()
                self.error.append(e)
                self.time.append(t)
                print(f"Time: {t:.2f}, Current: {I_def:.5f}\r", end="")
        except KeyboardInterrupt:
            print("  " * 14 + "\r", end="")
            self.device.set_current(0)
            print('Something happends :(')
        finally:
            self.device.set_current(0)
            self.device.disable()
            self.plot()


bus = CANSocket(interface='can0', serial_port="ttyACM11")
motor = GyemsDRC(can_bus=bus, device_id=0x141)

dev_param = DeviceParams(kp=3, kd=3, q_def=90, dq_def=0, current_limit=200, time_stop=10)

model = Model(motor, dev_param)
model.simulate()
