from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from libs.can import CANSocket
from libs.motors.gyems import GyemsDRC


def w_f(t, traj_parameters):
    w_max = traj_parameters['w_max']
    w_min = traj_parameters['w_min']
    tf = traj_parameters['tf']
    w = (w_max - w_min) / tf * t + w_min
    return w


def chirp(t, traj_parameters):
    Ai = traj_parameters['Ai']
    w = w_f(t, traj_parameters)
    I = Ai * np.sin(w * t)
    return I


traj_param = {'Ai': 50,
              'w_max': 6,
              'w_min': 3,
              'tf': 20}

motor_param = {'interface': 'can0',
               'id_motor': 0x141,
               'current_limit': 200}

bus = CANSocket(interface=motor_param['interface'], serial_port="ttyACM11", reset=True)

print('CAN BUS connected successfully')

motor = GyemsDRC(can_bus=bus, device_id=motor_param['id_motor'])
motor.set_degrees()
motor.current_limit = motor_param['current_limit']
motor.enable()

t0 = perf_counter()
t = 0

dq_data, I_data, t_data = [], [], []

try:
    while t < traj_param['tf']:
        I = chirp(t, traj_param)
        motor.set_current(I)
        t = perf_counter() - t0

        dq_data.append(motor.state['speed'])
        I_data.append(motor.state['current'])
        t_data.append(t)

except KeyboardInterrupt:
    motor.set_current(0)
    print('Something happends :(')

finally:
    motor.disable()
    dq, t, I = np.asarray(dq_data), np.asarray(t_data), np.asarray(I_data)
    ddq = np.diff(dq) / np.diff(t)
    dq, t, I = dq[1:], t[1:], I[1:]

    state = np.asarray([dq, ddq])
    I = np.reshape(I, (len(I), 1))

    params = I.T @ np.linalg.pinv(state)

    print("Moment of inertia:", params[0, 0], "units")
    print("Friction coefficient:", params[0, 1], "units")

    I_contr = params @ state

    I_contr = I_contr.T

    f, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 15))
    ax1.plot(t, dq, linewidth=1, color='red')
    ax1.set(ylabel='Velocity [rad/sec]')
    ax1.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)

    ax2.plot(t, ddq, linewidth=1, color='red')
    ax2.set(ylabel='Acceleration [rad/sec^2]', xlabel='Time [sec]')
    ax2.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    plt.show()

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, I, linewidth=1, color='blue', label='Experimental data')
    ax.plot(t, I_contr, linewidth=1, color='red', label='Validation data')
    ax.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    ax.set(ylabel='Current [units]', xlabel='Time [sec]')
    ax.legend()
    plt.show()
