from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from libs.can import CANSocket
from libs.motors.gyems import GyemsDRC


def control(motor, control_param, state):
    q, dq = motor.state['angle'], motor.state['speed']
    q_des, dq_des = control_param['q_des'], control_param['dq_des']
    Kp, Kd = control_param['Kp'], control_param['Kd']

    I_des = Kp * (q_des - q) + Kd * (dq_des - dq)
    motor.set_current(I_des)

    state.append([q, dq, motor.state['current'], I_des])
    return q_des - q, state


control_param = {'Kp': 10,
                 'Kd': 7,
                 'q_des': 90,
                 'dq_des': 2}

# control_param = {'Kp': -2,
#                  'Kd': 1,
#                  'q_des': 90,
#                  'dq_des': 0}

# control_param = {'Kp': 0,
#                  'Kd': 2,
#                  'q_des': 90,
#                  'dq_des': 1}

motor_param = {'interface': 'can0',
               'id_motor': 0x141,
               'current_limit': 200}

bus = CANSocket(interface=motor_param['interface'], serial_port="ttyACM10")

print('CAN BUS connected successfully')

motor = GyemsDRC(can_bus=bus, device_id=motor_param['id_motor'])
motor.set_degrees()
motor.current_limit = motor_param['current_limit']
motor.enable()

t0 = perf_counter()
t = []

state = []
e = 10

try:
    while e > 1:
        e, state = control(motor, control_param, state)
        t.append(perf_counter() - t0)

except KeyboardInterrupt:
    motor.set_current(0)
    print('Something happends :(')

finally:
    motor.disable()
    t, state = np.asarray(t), np.asarray(state)
    q, dq, I, I_des = state[:, 0], state[:, 1], state[:, 2], state[:, 3]
    q_des = np.zeros(t.shape) + control_param['q_des']

    f, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 15))
    ax1.plot(t, q, linewidth=1, color='red', label='Actual state')
    ax1.plot(t, q_des, linewidth=1, color='black', linestyle='--', label='Actual state')
    ax1.set(ylabel='Motor angle [deg]')
    ax1.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    ax1.legend()
    ax2.plot(t, dq, linewidth=2, color='red')
    ax2.set(ylabel='Velocity [rad/sec]', xlabel='Time [sec]')
    ax2.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    plt.show()

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, I, linewidth=1, color='blue', label='Actual control')
    ax.plot(t, I_des, linewidth=1, color='red', label='Desired control')
    ax.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    ax.set(ylabel='Current [units]', xlabel='Time [sec]')
    ax.legend()
    plt.show()
