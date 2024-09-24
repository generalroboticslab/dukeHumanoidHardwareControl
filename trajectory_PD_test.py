# sudo chrt -f 99 $(which python) trajectory_PD_test.py

from motor_controller import MotorController, CONTROL_MODE
import time
import numpy as np
from publisher import DataPublisher

pi = np.pi

data_publisher = DataPublisher()

motor = MotorController("enp3s0", CONTROL_MODE.CYCLIC_SYNC_VELOCITY, 20, 1000)
# motor.run()

exit()
motor.set_max_torque(np.ones(10)*1000)

# motor.position_offset=motor.motor_pos_offset
# motor.set_position_offset(motor.motor_pos_offset)

motor.kp=100 #60
motor.kd=5


time.sleep(0.1)

dt = 1/200
# dt = 1/400


motor.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_TORQUE


# kp = 60
# # kp = 70
# # kp = 80
# # kp = 100
# # kp = 120
# # kp = 140
# # kd = 7
# # kd = 6
# kd = 5


decimation=4

dof_pos_target = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

np.set_printoptions(formatter={'float': '{: 3.2f}'.format})

startTime = time.time()

# for i in range(200):
#     target_input_torque = motor.get_dof_pos_pd(dof_pos_target, motor.kp, motor.kd)
#     motor.set_target_input_torque(target_input_torque)
#     time.sleep(dt)

from publisher import DataReceiver
# Create a receiver instance
receiver = DataReceiver(port=9871, decoding="msgpack",broadcast=True)
# Start continuous receiving in a thread
receiver.receive_continuously()
received_id = 0

# t = 8
# a = 0
# b = np.pi / 12
# freq_start = 2
# freq_end = 2
# x = np.arange(0, t, dt*decimation)
# frequencies = np.linspace(freq_start, freq_end, x.size)
# y = (a + b) / 2 + (b - a) / 2 * np.sin(2 * np.pi * frequencies * x + np.pi)

t = time.time()

action_is_on = np.ones(10)

should_publish = False

for i in range(1000000):
    if receiver.data is not None:
        if receiver.data_id !=received_id:
            received_id = receiver.data_id
            # print(f"Received from {receiver.address}: {receiver.data}")
            # if "reset" in receiver.data and receiver.data['reset']:
            #     envs.reset_idx(torch.arange(envs.num_envs,device=envs.device))
            if "dof_pos_target" in receiver.data:
                dof_pos_target = np.array(receiver.data["dof_pos_target"])
                # print(dof_pos_target)
            if "action_is_on" in receiver.data:
                action_is_on = np.array(receiver.data["action_is_on"], dtype=np.float64)
            if "should_publish" in receiver.data:
                should_publish:bool = receiver.data["should_publish"]
                # print("this is should publish state: ",should_publish)
            
                
    # dof_pos_target=np.array([y[i%len(y)], 0, 0, 0, 0, -y[i%len(y)], 0, 0, 0, 0])

    for _ in range(decimation):
        
        # target_input_torque = motor.get_dof_pos_pd(dof_pos_target, motor.kp, motor.kd)
        # motor.set_target_input_torque(target_input_torque)

        motor.use_position_pd = True
        motor.target_dof_position = dof_pos_target
        
        # # passiveness
        # motor.set_max_torque(action_is_on*1000)

        # action_is_on = np.zeros(10) # HACK

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

    # time.sleep(dt*decimation)

        
