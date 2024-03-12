# sudo "$(which python)" test.py

import numpy as np

import signal
import time

from build import ethercat_motor_py

# CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
# CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
# CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode
    
interface_name = "enp3s0"
control_mode_int8 = 8
max_velocity = 1.5
max_torque= 200
motor=ethercat_motor_py.PyMotor("enp3s0",control_mode_int8, max_velocity, max_torque)
motor.should_print = 1
target_offset = [0.00051, 0.22990, 0.16049, 0.31407, 0.40668, 0.00145]
# motor.set_target_offset(target_offset)

# target_inputs = [
#     [0,1.047,0,1.047,1.047,0],
#     [0,1.047,0,0,0,0],
#     [0,0,0,0,0,0],
# ]

from roboticstoolbox import mstraj
target_inputs = [
    [0,0,0,0,0,0],
    [-0.1,1.0,0,0.2,0.2,0],
    [0,1.0,0,1.0,1.0,0],
    [0,0,0,0,0,0]
]


target_inputs= np.array(target_inputs)
target_inputs = target_inputs+np.array([target_offset])
dt=0.002
tg = mstraj(viapoints=target_inputs,dt=dt,tacc=0.5,qdmax=max_velocity)


print(motor.target_int32)
motor.set_target_input(target_inputs[0])
print(motor.target_int32)

def signal_handler(sig, frame):
    print("Keyboard Interrupt detected. Attempting to stop background thread...")
    motor.set_should_terminate(True)
    exit()
signal.signal(signal.SIGINT, signal_handler)

motor.run()
for i in range(1000000):
    # print(list(target_inputs[i%len(target_inputs)]))
    # motor.set_target_input(list(target_inputs[i%len(target_inputs)]))
    
    motor.set_target_input(list(tg.q[i%len(tg.q)]))
    # print(f"Doing other Python tasks")
    time.sleep(dt)

