# DEBUG LIBRARIES
import sys
import time
import signal
import onnxruntime as ort
import numpy as np
from omegaconf import OmegaConf
import os
import copy

from operator import itemgetter
from sensor_controller import SensorController
from motor_controller import MotorController, CONTROL_MODE
from publisher import DataPublisher, DataReceiver
from numpy_ringbuffer import RingArrayBuffer

class Model:
    def __init__(self, onnx_model_path) -> None:
        self.ort_model = ort.InferenceSession(onnx_model_path)

    def get_action(self, obs):
        mu, log_std, value = self.ort_model.run(
            None,
            {"obs": np.asarray(obs, dtype=np.float32).reshape(1, -1)},
        )
        return mu


class BipedController:

    def __init__(self) -> None:

        # baseline policy
        path = "checkpoint/baseline"
        # # passive (uncomment this line to use it)
        # path = "checkpoint/passive"

        
        if (not path.startswith("checkpoint")) and (not os.path.isabs(path)):  # Check if it's relative
            root_path = "/home/grl/repo/legged_env" # change to your root path fo
            path = os.path.join(root_path, path)
        
        model_path = f"{path}/policy.onnx"
        config_path = f"{path}/config.yaml"

        # init model
        self.model = Model(model_path)
        
        cfg = OmegaConf.load(config_path)

        # cfg.task.env.control.actionScale
        
        self.keep_still_at_zero_command: bool = cfg.task.env.get("keep_still_at_zero_command",False)

        self.obs_names: list = cfg.task.env.observationNames
        self.lin_vel_scale: float = cfg.task.env.learn.linearVelocityScale
        self.ang_vel_scale: float = cfg.task.env.learn.angularVelocityScale
        self.dof_pos_scale: float = cfg.task.env.learn.dofPositionScale
        self.dof_vel_scale: float = cfg.task.env.learn.dofVelocityScale
        
        self.commands_scale = np.array([self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale])
        self.command_zero_threshold = cfg.task.env.commandZeroThreshold
        
        self.dt: float = cfg.task.sim.dt
        self.decimation:int = cfg.task.env.control.decimation
        self.rl_dt_ns: float = self.dt*self.decimation*1e9 # to [ns]
        # print(self.rl_dt_ns)

        self.num_dof = 10
        self.num_foot = 2

        def get_dof_param(param):
            if isinstance(param, (float, int)):
                dof_param = np.array([param]*self.num_dof,dtype=np.float64)
            else: # assuming list
                dof_param = np.array(param,dtype=np.float64)
            return dof_param

        self.kp = get_dof_param(cfg.task.env.control.stiffness)
        self.kd = get_dof_param(cfg.task.env.control.damping)

        # self.kp = self.kp*1.1
    
        self.action_scale:float = cfg.task.env.control.actionScale
        # self.action_scale=0
        
        self.default_dof_pos=np.array(cfg.task.env.defaultJointPositions)

        # #     # HACK
        # #     self.default_dof_pos=np.zeros(10)
        # #     self.action_scale=0
        # #    # # HACK
        #     self.kp[:]=10
        #     self.kd[:]=1

        self.phase_freq: float = cfg.task.env.learn.guided_contact.phase_freq
        self.phase_stance_ratio: float = cfg.task.env.learn.guided_contact.phase_stance_ratio
        self.phase_swing_ratio: float = 1 - self.phase_stance_ratio
        # NOTE! BREAKING CHANGE 09/07 : phase start with swing first instead
        self.phase_start_with_swing: bool = cfg.task.env.learn.guided_contact.get("phase_start_with_swing",False) # default phase_start_with_stance in previous policy
        print(f"\033[93mphase_start_with_swing: {self.phase_start_with_swing}\033[0m")

        self.phase_offset = np.array(cfg.task.env.learn.guided_contact.phase_offset)

        
        if "enablePassiveDynamics" in cfg.task.env.learn and cfg.task.env.learn.enablePassiveDynamics:
            self.enablePassiveDynamics = True
            self.num_action = self.num_dof*2
            self.action_is_on_sigmoid_k: float = cfg.task.env.learn.action_is_on_sigmoid_k
            self.compute_action=self.compute_action_passive
            # BRAKING CHANGE 
            self.min_action_is_on: float = cfg.task.env.learn.get("action_is_on_min",0)
            # self.min_action_is_on: float = cfg.task.env.learn.action_is_on_min

        else:
            self.enablePassiveDynamics = False
            self.num_action = self.num_dof
        self.action = np.zeros(self.num_action)        
        self.action_to_use = np.zeros_like(self.action)

        self.obs_dim_dict = {
            "linearVelocity": 3,
            "angularVelocity": 3,
            "projectedGravity": 3,
            "projected_gravity_filtered": 3,
            "projected_gravity_xy": 2,
            "commands": 3,  # vel_x,vel_y, vel_yaw, (excluding heading)
            "dofPosition": self.num_dof,
            "dofVelocity": self.num_dof,
            "dof_force_target": self.num_dof,
            "dof_strength": self.num_dof,
            "dofForce": self.num_dof,
            "base_height": 1,
            "actions": self.num_action,
            "last_actions": self.num_action,
            "contact": self.num_foot,  # foot contact indicator
            "phase": self.num_foot*2, # phase of each foot (contact sequece)
            "contactTarget": self.num_foot,  # foot contact indicator
        }

        self.num_stacked_obs_frame: int = cfg.task.env.get("num_stacked_obs_frame", 1)
        self.num_obs_single_frame: int = np.sum(itemgetter(*self.obs_names)(self.obs_dim_dict))
        self.num_obs = self.num_obs_single_frame * self.num_stacked_obs_frame
        self.batched_obs_buf = RingArrayBuffer(buffer_len=self.num_stacked_obs_frame,shape=self.num_obs_single_frame,dtype=np.float64)
        # debugging
        import inspect
        for attr in inspect.getmembers(self):
            if not attr[0].startswith('_'):  # Exclude private/internal attributes
                if not inspect.ismethod(attr[1]): # Exclude methods
                    print(f"{attr[0]}: {attr[1]}") 
        
        self.should_reset = False
        

        # command
        self.commands = np.zeros(3)
        self.dof_vel =np.zeros(10)
        
        # filters
        
        # initialize sensors and data collection
        self.sensors = SensorController()
        self.sensors.start()
        self.sensor_data = self.sensors.get_latest_data()
        # wait until sensor data is ready
        while self.sensor_data is None:
            self.sensor_data = self.sensors.get_latest_data()
            print("Waiting for sensor data to be ready...")
            time.sleep(0.01)
        print("Sensor data is ready!")
        
        # data receiver
        self.data_receiver = DataReceiver(port=9871,decoding="msgpack",broadcast=True)
        self.data_receiver_data_id = self.data_receiver.data_id # for check if data is new
        self.data_receiver.receive_continuously()
        
        self.data_receiver2 = DataReceiver(port=9876,decoding="msgpack",broadcast=False)
        self.data_receiver2_data_id = self.data_receiver2.data_id # for check if data is new
        # self.data_receiver2.receive_continuously()
        self.dof_pos_target = self.default_dof_pos
        
        self.data_publisher = DataPublisher('udp://localhost:9870',encoding="msgpack",broadcast=False)
        
        # initialize motors
        self.max_torque = 1000.0
        self.motor = MotorController(
            "enp3s0", CONTROL_MODE.CYCLIC_SYNC_TORQUE, 20, self.max_torque
        )
        self.motor.run()
        
        self.motor_counter = self.motor.loop_counter
        self.decimation_actual = 0
        self.rl_dt_measured = 0
        self.t_ns = time.perf_counter_ns()

        # # # HACK remove later
        # self.motor.set_max_torque(np.ones(10)*1500)

        
        # setup signal handler if it has not been setup elsewhere
        if not signal.getsignal(signal.SIGINT):
            self.setup_signal_handler()


    def update_phase(self):
        """update normalized contact phase for each foot"""
        # TODO: may need to change into relative time
        self.phase = (self.phase_freq * self.t_ns*1e-9  +self.phase_offset) % 1
        # print(self.phase)
        phase_rad = self.phase*np.pi*2
        self.phase_sin_cos = np.hstack([np.sin(phase_rad), np.cos(phase_rad)])
        # print(self.phase_sin_cos)

        if self.phase_start_with_swing:
             self.contact_target = self.phase > self.phase_swing_ratio
        else:
            self.contact_target = self.phase <= self.phase_stance_ratio 

        self.is_zero_command = np.linalg.norm(self.commands[:3]) < self.command_zero_threshold
        
        if self.is_zero_command and self.keep_still_at_zero_command:
            self.contact_target[:] = 1
            self.commands[:] = 0
            # self.contact_target[self.is_zero_command] = 1 # set contact_target to 1 if zero command
            # self.commands[self.is_zero_command]=0


    def initialize_stance(self):
        self.motor.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_TORQUE
        self.motor.kp = np.ones(10)*40
        self.motor.kd = np.ones(10)*8.0 
        self.motor.use_position_pd = True
        self.motor.target_dof_position =  self.default_dof_pos
        time.sleep(3)
        # set back to default kp kd
        self.motor.kp = self.kp
        self.motor.kd = self.kd
        print(f"kp={self.motor.kp}")
        print(f"kd={self.motor.kd}")    
    def check_termination(self):
        if self.projected_gravity[2]>-0.7: # base tilt 45 degree
            self.should_reset = True
    
        if self.should_reset:
            self.motor.set_max_torque(np.zeros(self.motor.num_dof))
            self.shutdown()
            exit()
            
    def step(self):
        # check if cmmd is updated
        self.update_command()
        
        # update observations
        self.update_observation()
        
        self.check_termination()
        
        # # compute action
        self.compute_action()
        # # limit check to prevent overshoot
        self.dof_pos_target = self.motor.limit_check(self.dof_pos_target)

        self.motor.target_dof_position = self.dof_pos_target
        if self.enablePassiveDynamics:
            # self.motor.set_max_torque(self.action_is_on.astype(np.float64)*1000)
            self.motor.torque_multiplier = self.action_is_on.astype(np.float64)
        
        motor_counter = self.motor.loop_counter
        self.decimation_actual = motor_counter - self.motor_counter # masured decimation per RL loop
        self.motor_counter = motor_counter
        
        # # if self.decimation_actual>40: 
        # print(self.decimation_actual, time.perf_counter_ns()-self.t_ns, time.perf_counter_ns(),self.t_ns)

        data = {
            "decimation_actual": self.decimation_actual,
            "dof_pos": self.dof_pos,
            "dof_vel": self.dof_vel,
            "commands": self.commands,
            "base_ang_vel": self.base_ang_vel,
            "projected_gravity": self.projected_gravity,
            "action": self.action,
            "action_filtered":self.action_to_use,
            "phase_sin_cos": self.phase_sin_cos,
            "phase":self.phase,
            # "actions_to_use":self.action_to_use,
            "base_quat": self.base_quat,
            "contact": self.contact,
            "force_measurement": self.force_measurement,
            "contact_target": self.contact_target,
            # "dof_force_target": self.target_input_torque,
            "dof_force_target": self.motor.target_dof_torque_Nm,
            "dof_pos_target": self.dof_pos_target,
            # "rl_dt_measured_div_rl_dt": self.rl_dt_measured/self.rl_dt_ns,
            # "obs_buf": self.obs_buf,
            # "obs_buf_diff": self.obs_buf - self.obs_buf_sim,
            # "obs_buf_diff": np.sum(self.obs_buf - self.obs_buf_sim),

        }

        if self.enablePassiveDynamics:
            data["action_is_on"] = self.action_is_on

        self.data_publisher.publish({"real":data})

        # sleep for rl_dt_ns
        t_ns = time.perf_counter_ns()
        elapsed = t_ns - self.t_ns
        if elapsed < self.rl_dt_ns:
            time.sleep((self.rl_dt_ns - elapsed)*1e-9)
        
        self.rl_dt_measured = time.perf_counter_ns()-self.t_ns
        self.t_ns = time.perf_counter_ns()

    def update_command(self):
        if self.data_receiver_data_id != self.data_receiver.data_id:
            self.data_receiver_data_id = self.data_receiver.data_id
            if "cmd" in self.data_receiver.data:
                self.commands[:] = np.array(self.data_receiver.data["cmd"])
                print(f"{self.commands[:]}")
            if "reset" in self.data_receiver.data and self.data_receiver.data["reset"]:
                self.should_reset = True

    def compute_action(self):
        self.action = self.model.get_action(self.obs_buf)[0]
        alpha = 0.9
        self.action_to_use = alpha*self.action + (1-alpha)*self.action
        self.dof_pos_target = self.action_to_use * self.action_scale + self.default_dof_pos
        
    def compute_action_passive(self):
        self.action = self.model.get_action(self.obs_buf)[0]
        alpha = 0.9
        self.action_to_use = alpha*self.action + (1-alpha)*self.action_to_use
        self.dof_pos_target = self.action_scale * self.action_to_use[:self.num_dof]  + self.default_dof_pos
        self.action_is_on = sigmoid_k(self.action_to_use[self.num_dof:], self.action_is_on_sigmoid_k)
        self.action_is_on = self.min_action_is_on + (1 - self.min_action_is_on) * self.action_is_on

    def update_observation(self):
        # Return a dictionary of observable data
        self.sensor_data = self.sensors.get_latest_data()
        if self.sensor_data is None:
            print("WARNING: No sensor data received!")
            self.should_reset = True
            return

        # print(self.sensor_data["sps"])
        self.base_ang_vel = np.array(self.sensor_data["ang_vel"])
        self.projected_gravity = np.array(self.sensor_data["gravity_vec"])
        # # HACK:TODO VERIFY THIS
        # self.projected_gravity =  np.array([-self.projected_gravity[0],self.projected_gravity[1],self.projected_gravity[2]])
        self.base_quat = np.array(self.sensor_data['base_quat'])

        self.force_measurement = self.sensor_data["force_measurement"]
        self.contact = self.sensor_data["contact"]
        
        # self.base_ang_vel = np.zeros(3)
        # self.projected_gravity = np.array([0,0,-1.0])
        
        self.update_phase()

        # HACK: broken force sensor repalce
        # self.contact = self.contact_target
        # self.contact[:]=0

        # dof_vel = self.motor.dof_vel*0.95 + self.dof_vel * 0.05
        # self.dof_vel = dof_vel
        
        self.dof_pos = self.motor.dof_pos
        self.dof_vel = self.motor.dof_vel


        # data, _ = self.data_receiver2.receive()
        # # if (self.data_receiver2.data is not None):
        # #     data = copy.deepcopy(self.data_receiver2.data)
        # if data is not None:
        #     self.base_ang_vel = np.array(data["base_ang_vel"])
        #     self.projected_gravity = np.array(data["projected_gravity"])
        #     self.dof_pos = np.array(data["dof_pos"])
        #     self.dof_vel = np.array(data["dof_vel"])
        #     self.action = np.array(data["action"])
        #     self.contact = np.array(data["contact"])
        #     # self.contact = np.zeros(2) # HACK ONLY
        #     self.contact_target = np.array(data["contact_target"])
        #     self.phase = np.array(data["phase"])
        #     self.phase_sin_cos = np.array(data["phase_sin_cos"])
        #     self.commands = np.array(data["cmd"])
        #     self.obs_buf_sim = np.array(data["obs_buf"])
        # else:
        #     print("\033[93m" + "self.data_receiver2.data is None!!!!!!!!!!!!!!!!!!!!!!!!" + "\033[0m")
        #     pass

        self.obs_dict = {
            # 'linearVelocity': sensor_data['linearVelocity'], # DOES NOT HAVE THIS IN REAL
            # "linearVelocity": np.zeros(3), # HACK: FOR DEBUGGING
            "angularVelocity": self.base_ang_vel * self.ang_vel_scale,
            "projectedGravity": self.projected_gravity,
            "projected_gravity_filtered": self.projected_gravity,
            "projected_gravity_xy": self.projected_gravity[:2],
            "commands":  self.commands * self.commands_scale,
            "dofPosition": self.dof_pos * self.dof_pos_scale,
            "dofVelocity": self.dof_vel* self.dof_vel_scale,
            # "heightMap":
            # "actions": np.zeros(10), # HACK: FOR DEBUGGING
            "actions": self.action, # TODO
            # "actionFiltered": self.action_to_use,
            "phase": self.phase_sin_cos,
            "contactTarget":self.contact_target,
            "contact":self.contact,
        }
        obs_buf_single_frame = np.concatenate(
            itemgetter(*self.obs_names)(self.obs_dict), axis=-1
        )
        # HACK
        # self.obs_buf=obs_buf_single_frame
        self.batched_obs_buf.add(obs_buf_single_frame)
        self.obs_buf = self.batched_obs_buf.get_last_n(self.num_stacked_obs_frame).ravel()

    def setup_signal_handler(self):
        def signal_handler(sig, frame):
            print("Keyboard Interrupt!")
            self.shutdown()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

    def shutdown(self):
        # Ensure motors and sensors are properly stopped
        self.motor.set_should_terminate(True)
        self.sensors.stop()
        self.data_receiver.stop()
        print("BipedController has been shut down cleanly.")

def sigmoid_k(x: np.ndarray, k: float) -> np.ndarray:
    return 1 / (1 + np.exp(-k*x))

# DEBUGGING
if __name__ == "__main__":
    controller = BipedController()
    controller.initialize_stance()
    np.set_printoptions(formatter={"float": "{: 3.2f}".format})
    t = time.time()
    while True:
        controller.step()
