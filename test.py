# conda activate py38 && sudo "$(which python)" test.py

import numpy as np

import signal
import time
import publisher
import serial # conda install -c conda-forge pyserial
from serial_communication import SerialCommunication

from build import ethercat_motor_py as MOTOR

# CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
# CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
# CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode

interface_name = "enp3s0"
control_mode_int8 = MOTOR.CONTROL_MODE.CYCLIC_SYNC_POSITION
# control_mode_int8 = MOTOR.CONTROL_MODE.CYCLIC_SYNC_VELOCITY

# control_mode_int8 = MOTOR.CONTROL_MODE.CYCLIC_SYNC_VELOCITY

max_velocity = 2
max_torque = 500
target_offset = np.array([0.00016, 0.27583, 0.12610, 0.29630, 0.38658, 0])
# motor.set_target_position_offset(target_offset)

# constants (asssume 6 motors)
min_limit = np.array([-1.047197,-0.523599,-0.349066,0,-0.785398,-np.inf]) #[hip1,hip2,hip3,knee,ankle,extra]
max_limit = np.array([0.523599,1.570796,0.785398,1.570796,0.785398,np.inf])

from roboticstoolbox import mstraj

target_joint_positions = np.array(
    [
        # [0, 0.5, 0,  1,  -0.5, 0],
        # [0, 0.1, 0, 0.2, -0.1, 0],
        # [0, 0.5, 0,  1,  -0.5, 0],
        
        [0,0,0,0,0,0],
        [0,0.4,0,0.8,-0.3,0],
        [0,0.4,0,0,0,0],
        [0,-0.4,0,-0.8,0.3,0]
        
        
        # [0, 0, 0,  0, 0, 0],
        # [0, 0, 0,  0, 0, 0]

    ]
)

dt = 0.001
tg = mstraj(viapoints=target_joint_positions, dt=dt, tacc=0.2, qdmax=max_velocity)


def limit_joint_input(inputs): 
    return np.minimum(np.maximum(inputs, min_limit),max_limit)

def get_motor_position(joint_angles):
    motor_positions = joint_angles[:]
    motor_positions[:,4]+=motor_positions[:,3]
    return motor_positions  

motor_positions = get_motor_position(limit_joint_input(tg.q)) + target_offset

def signal_handler(sig, frame):
    print("Keyboard Interrupt detected. Attempting to stop background thread...")
    motor.set_should_terminate(True)
    exit()

signal.signal(signal.SIGINT, signal_handler)


data_publisher = publisher.DataPublisher(
    target_url='udp://10.197.197.153:9870',
    encoding_type='msgpack',
    is_broadcast=False,
    is_enabled=True
    )

# teensy serial communication
# ser = SerialCommunication()

motor = MOTOR.PyMotor("enp3s0", control_mode_int8, max_velocity, max_torque)
motor.should_print = False
motor.debug = False
init_positions = np.array([
    motor.actual_position,
    motor_positions[0]
])

print(init_positions)

tg0 = mstraj(viapoints=init_positions, dt=dt, tacc=1, qdmax=0.1)

print(tg0.qd)
np.set_printoptions(formatter={'float': '{: 9.4f}'.format}) # pretty print numpy
motor.run()



# for i in range(len(tg0.q)):
#     motor.set_target_input(tg0.q[i])
#     motor.set_velocity_offset(tg0.qd[i])
#     # print(f"actual_position      : {motor.actual_position}")
#     # print(f"actual_position_error: {motor.actual_position_error}")
#     # print(f"actual_velocity      : {motor.actual_velocity}")
#     # print(f"actual_torque_raw    : {motor.actual_torque_raw}")


# motor.set_velocity_offset(np.zeros(6))

for i in range(100000000):
    
    
    # motor.set_target_input(tg.q[i % len(tg.q)])
    # motor.set_target_input(np.array([0.,-0.40882653,0,0,0,0])+target_offset)
    # motor.set_target_input(target_offset)
    
    motor.set_target_input(motor_positions[i % len(tg.q)])
    
    
    # motor.set_target_input(np.array([0.,0.,0.,0.,0.,0.]))
    
    # if i%1000<500:
    #     motor.set_max_torque(np.array([1,1,1,1,1,1],dtype=np.float64))
    #     print("set_max_torque:1")
    # else:
    #     motor.set_max_torque(np.array([200,200,200,200,200,200],dtype=np.float64))
    #     print("set_max_torque:500")
        
    # motor.set_target_input(get_motor_angles(limit_joint_input(np.array([0,0,0,0,0,0])))+target_offset)
    
    
    # ser.read()
    
    # data = {
    #     "actual_position":motor.actual_position,
    #     "actual_position_error":motor.actual_position_error,
    #     "actual_velocity":motor.actual_velocity,
    #     "actual_torque_raw":motor.actual_torque_raw,
    #     "force_measurement":ser.unpacked[2:]
    # }
    # data_publisher.publish(data=data)
     
    # time.sleep(0.1)
    # print(f"actual_position      : {motor.actual_position}")
    # print(f"actual_position_error: {motor.actual_position_error}")
    # print(f"actual_velocity      : {motor.actual_velocity}")
    # print(f"actual_torque_raw    : {motor.actual_torque_raw}")
    
    # print(i)
    time.sleep(dt)

