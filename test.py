# sudo "$(which python)" test.py

import numpy as np

import signal
import time

from build import ethercat_motor_py as MOTOR

# CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
# CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
# CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode

interface_name = "enp3s0"
control_mode_int8 = MOTOR.CONTROL_MODE.CYCLIC_SYNC_POSITION

# control_mode_int8 = MOTOR.CONTROL_MODE.CYCLIC_SYNC_VELOCITY

max_velocity = 0.5


max_torque = 240
motor = MOTOR.PyMotor("enp3s0", control_mode_int8, max_velocity, max_torque)
motor.should_print = False
motor.debug = False
target_offset = np.array([0.0116,0.2417,0.1429,0.3054,0.3872,0.0015])
# motor.set_target_position_offset(target_offset)


from roboticstoolbox import mstraj

target_inputs = np.array(
    [
        [0, 0, 0, 0, 0, 0],
        [-0.3, 1.0, 0.2, 0.2, 0.2, 0],
        [0, 1.0, 0, 1.0, 1.0, 0],
        [0, 0, 0, 0, 0, 0],
    ]
)

target_inputs = target_inputs + target_offset
dt = 0.002
tg = mstraj(viapoints=target_inputs, dt=dt, tacc=0.2, qdmax=max_velocity)

def limit_joint_input(inputs): 
    min_limit = np.array([-1.047197,-0.523599,-0.349066,0,-0.785398,-np.inf]) #[hip1,hip2,hip3,knee,ankle,extra]
    max_limit = np.array([0.523599,1.570796,0.785398,1.570796,0.785398,np.inf])
    limited = np.maximum(inputs, min_limit)
    limited = np.minimum(limited, max_limit)
    return limited

def get_motor_angles(joint_angles):
    return joint_angles + np.array([0,0,0,0,joint_angles[3],0])  

print(motor.target_int32)
motor.set_target_input(target_inputs[0])
print(motor.target_int32)


def signal_handler(sig, frame):
    print("Keyboard Interrupt detected. Attempting to stop background thread...")
    motor.set_should_terminate(True)
    exit()


signal.signal(signal.SIGINT, signal_handler)

np.set_printoptions(formatter={'float': '{: 9.4f}'.format}) # pretty print numpy
motor.run()
for i in range(100000000):
    
    # motor.set_target_input(tg.q[i % len(tg.q)])
    # motor.set_target_input(np.array([0.,-0.40882653,0,0,0,0])+target_offset)
    # motor.set_target_input(target_offset)
    
    motor.set_target_input(limit_joint_input(tg.q[i % len(tg.q)]))
    
    # motor.set_target_input(get_motor_angles(limit_joint_input(np.array([0,0,0,0,0,0])))+target_offset)
    
    # motor.set_target_input(np.array([0.,0.,0.,0.,0.,0.]))
 

    print(f"actual_position      : {motor.actual_position}")
    print(f"actual_position_error: {motor.actual_position_error}")
    print(f"actual_velocity      : {motor.actual_velocity}")
    print(f"actual_torque_raw    : {motor.actual_torque_raw}")
    # print(f"Doing other Python tasks")
    
    time.sleep(dt)

