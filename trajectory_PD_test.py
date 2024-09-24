# sudo chrt -f 99 $(which python) trajectory_PD_test.py

from motor_controller import MotorController, CONTROL_MODE
import time
import numpy as np
from publisher import DataPublisher
from publisher import DataReceiver

pi = np.pi


motor = MotorController("enp3s0", CONTROL_MODE.CYCLIC_SYNC_VELOCITY, 20, 1000)
motor.run()

motor.set_max_torque(np.ones(10)*1000)
# motor.set_max_torque(np.ones(10)*40)

# motor.position_offset=motor.motor_pos_offset
# motor.set_position_offset(motor.motor_pos_offset)

motor.kp=np.ones(10)*60
motor.kd=np.ones(10)*5

# motor.kp=np.ones(10)*120
# motor.kd=np.ones(10)*5

# motor.kp=np.zeros(10)
# motor.kd=np.zeros(10)

time.sleep(0.1)

dt = 1/200
# dt = 1/400

motor.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_TORQUE

decimation=4

dof_pos_target = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

np.set_printoptions(formatter={'float': '{: 3.2f}'.format})

startTime = time.time()

data_publisher = DataPublisher()

# Create a receiver instance
receiver = DataReceiver(port=9871, decoding="msgpack",broadcast=True)
# Start continuous receiving in a thread
receiver.receive_continuously()
received_id = 0


t = time.time()

action_is_on = np.ones(10)

should_publish = False

for i in range(1000000):
    if receiver.data is not None:
        if receiver.data_id !=received_id:
            received_id = receiver.data_id
            if "dof_pos_target" in receiver.data:
                dof_pos_target = np.array(receiver.data["dof_pos_target"])
                # print(dof_pos_target)
            if "action_is_on" in receiver.data:
                action_is_on = np.array(receiver.data["action_is_on"], dtype=np.float64)
            if "should_publish" in receiver.data:
                should_publish:bool = receiver.data["should_publish"]
            if "kp" in receiver.data:
                motor.kp = np.array(receiver.data["kp"], dtype=np.float64)
            if "kd" in receiver.data:
                motor.kd = np.array(receiver.data["kd"], dtype=np.float64)
            
    for _ in range(decimation):

        motor.use_position_pd = True
        motor.target_dof_position = dof_pos_target
        motor.torque_multiplier = action_is_on

        data = {
            "dof_pos":motor.dof_pos,
            "dof_vel":motor.dof_vel,
            "dof_vel_raw":motor._dof_vel_raw,
            "dof_current":motor.dof_current,
            "dof_force":motor.dof_force,
            "dof_force_target":  motor.target_dof_torque_Nm,
            # "dof_vel_cpp":motor.dof_velocity,
            # "dof_pos_cpp":motor.dof_position,
            # "motor_pos_offset_cpp": motor.position_offset,
            # "motor_pos_after_offset":motor.motor_pos_after_offset,
            # "motor_pos_after_offset_cpp": motor.actual_position_after_offset,
            # "motor_pos_after_offset_difference":motor.motor_pos_after_offset-motor.actual_position_after_offset,
            # "motor_pos": motor.actual_position,
            # "motor_pos_offset": motor.motor_pos_offset,
            "dof_pos_target": dof_pos_target,
            "target_dof_torque_A":motor.target_dof_torque_A,
            "target_dof_torque_A_adjusted": motor.target_dof_torque_A_adjusted,
            # "dt_measured": motor.dt_measured,
            "action_is_on":action_is_on,
            "t_ns": time.perf_counter_ns()
            # "motor_vel": motor.actual_velocity,
            # "motor_vel_measured": motor.actual_velocity_measured,
        }
        # data_publisher.enable = should_publish
        data_publisher.enable = True
        data_publisher.publish({"real":data})
        
        # time.sleep(dt)
        elapsed = time.time() - t
        if elapsed < dt:
            time.sleep(dt - elapsed)
        t = time.time()

        
