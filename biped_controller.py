# DEBUG LIBRARIES
import sys
import time
import signal
import onnxruntime as ort
import numpy as np
from omegaconf import OmegaConf


from operator import itemgetter
from sensor_controller import SensorController
from motor_controller import MotorController, CONTROL_MODE
from publisher import DataPublisher, DataReceiver


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
        
        
        # base_path = "checkpoint/biped_noise_kp60kd2/runs/BipedAsymm_07-10-08-59" # evil, messed up encoder 
        # base_path = "checkpoint/biped_kp60kd2/runs/BipedAsymm_07-10-08-30" # better sim2real than higher kd
        # base_path = "checkpoint/biped/runs/BipedAsymm_07-10-09-18_delay_2" # weak, no vibration, delays is important,kp60,kd5
        # base_path = "checkpoint/biped_kp60kd4/runs/BipedAsymm_07-10-12-21" # similar to noise version, ok, no vibration
        # base_path = "checkpoint/biped_noise_kp60kd4/runs/BipedAsymm_07-10-13-53" # no vibration, weak, ok, good start
        # base_path = "checkpoint/biped/runs/BipedAsymm_10-19-03-56_mass_2"
        # base_path = "checkpoint/biped/runs/BipedAsymm_10-19-05-25_mass_5"
        # base_path = "checkpoint/biped/runs/BipedAsymm_11-15-03-24_mass_5_orient_1"
        # base_path = "checkpoint/biped/runs/BipedAsymm_12-23-07-53_action_filt_09"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_passive/runs/BipedAsymm_14-23-06-06"
        # base_path = "checkpoint/biped_passive/runs/BipedAsymm_17-16-25-57" 
        # base_path ="checkpoint/biped/runs/BipedAsymm_17-16-25-21"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped/runs/BipedAsymm_19-17-51-21_motor_strength"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_passive/runs/BipedAsymm_19-17-55-19_sigmoid_10" # too weak
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_debug/runs/BipedAsymm_19-23-10-07_dt_0.0010-decimation_10"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_debug/runs/BipedAsymm_19-23-12-45_dt_0.0025-decimation_4"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_debug/runs/BipedAsymm_19-23-13-58_dt_0.0050-decimation_2"
        base_path ="/home/grl/repo/legged_env/outputs/Biped/export/biped/runs/BipedAsymm_20-01-46-33_0.75freq_foot_h0.2_contact_cuuriculum_later_checkpoint"
        # base_path = "/home/grl/repo/legged_env/outputs/Biped/export/biped_debug/runs/BipedAsymm_20-12-31-13_with_contact"
                
        model_path = f"{base_path}/policy.onnx"
        config_path = f"{base_path}/config.yaml"

        # init model
        self.model = Model(model_path)
        
        cfg = OmegaConf.load(config_path)

        # cfg.task.env.control.actionScale
        
        self.obs_names: list = cfg.task.env.observationNames
        self.lin_vel_scale: float = cfg.task.env.learn.linearVelocityScale
        self.ang_vel_scale: float = cfg.task.env.learn.angularVelocityScale
        self.dof_pos_scale: float = cfg.task.env.learn.dofPositionScale
        self.dof_vel_scale: float = cfg.task.env.learn.dofVelocityScale
        
        self.commands_scale = np.array([self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale])
        self.command_zero_threshold = cfg.task.env.commandZeroThreshold
        
        self.dt: float = cfg.task.sim.dt

        self.decimation:int = cfg.task.env.control.decimation
        self.kp:float = cfg.task.env.control.stiffness
        self.kd:float = cfg.task.env.control.damping

        # self.kp=60
        # self.kd=2
        
        self.action_scale:float = cfg.task.env.control.actionScale
        
        self.default_dof_pos=np.array(cfg.task.env.defaultJointPositions)


        self.phase_freq: float = cfg.task.env.learn.guided_contact.phase_freq
        self.phase_stance_ratio: float = cfg.task.env.learn.guided_contact.phase_stance_ratio
        self.phase_offset = np.array(cfg.task.env.learn.guided_contact.phase_offset)

        self.num_dof = 10
        if "enablePassiveDynamics" in cfg.task.env.learn and cfg.task.env.learn.enablePassiveDynamics:
            self.enablePassiveDynamics = True
            self.action = np.zeros(self.num_dof*2)
            self.action_to_use = np.zeros_like(self.action)
            self.update_action=self.update_action_passive
        else:
            self.enablePassiveDynamics = False
            self.action = np.zeros(self.num_dof)        
            self.action_to_use = np.zeros_like(self.action)

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
        self.data_receiver2.receive_continuously()
        self.dof_pos_target = self.default_dof_pos
        
        self.data_publisher = DataPublisher('udp://localhost:9870',encoding="msgpack",broadcast=False)
        
        # initialize motors
        self.max_torque = 1500.0
        self.motor = MotorController(
            "enp3s0", CONTROL_MODE.CYCLIC_SYNC_TORQUE, 20, self.max_torque
        )
        self.motor.run()
        # # # HACK remove later
        # self.motor.set_max_torque(np.ones(10)*1500)

        
        # setup signal handler if it has not been setup elsewhere
        if not signal.getsignal(signal.SIGINT):
            self.setup_signal_handler()

        self.t = 0

    def update_phase(self):
        """update normalized contact phase for each foot"""
        self.t = time.time()
        self.phase = (self.phase_freq * self.t  +self.phase_offset) % 1
        # self.phase_sin_cos = np.array([np.sin(self.phase), np.cos(self.phase)])
        self.contact_target = self.phase < self.phase_stance_ratio 
        self.is_zero_command = np.linalg.norm(self.commands[:3]) < self.command_zero_threshold
        
        if self.is_zero_command:
            self.contact_target[:] = 1
            self.commands[:] = 0
            # self.contact_target[self.is_zero_command] = 1 # set contact_target to 1 if zero command
            # self.commands[self.is_zero_command]=0


    def initialize_stance(self):
        self.motor.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_TORQUE
        # kp = 40
        # kd = 2.0 
        
        # dof_pos_target =  self.default_dof_pos
        # # dof_pos_target = np.zeros(10)
        # for _ in range(500): # HACK CHANGE BACK
        #     target_input_torque = self.motor.get_dof_pos_pd(dof_pos_target, kp, kd)
        #     self.motor.set_target_input_torque(target_input_torque)
        #     time.sleep(1/200)
        self.motor.kp = 40
        self.motor.kd = 4.0 
        self.motor.use_position_pd = True
        self.motor.target_dof_position =  self.default_dof_pos
        time.sleep(3)

        # set back to default kp kd
        self.motor.kp = self.kp
        self.motor.kd = self.kd
            
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
        
        # compute action
        self.update_action()
        
        # if self.data_receiver2_data_id != self.data_receiver2.data_id:
        #     self.data_receiver2_data_id = self.data_receiver2.data_id
        #     if (self.data_receiver2.data is not None):
        #         # if ("dof_pos_target" in self.data_receiver2.data):
        #         #     self.dof_pos_target = np.array(self.data_receiver2.data["dof_pos_target"])
        #         if ("dof_pos" in self.data_receiver2.data):
        #             self.dof_pos_target = np.array(self.data_receiver2.data["dof_pos"])

        self.motor.target_dof_position = self.dof_pos_target
        if self.enablePassiveDynamics:
            # self.motor.set_max_torque(self.action_is_on.astype(np.float64)*1000)
            self.motor.torque_multiplier = self.action_is_on.astype(np.float64)

        # for _ in range(self.decimation):
        #     time.sleep(1/200)

        # for _ in range(self.decimation):
        #     # do pd control
        #     self.target_input_torque = self.motor.get_dof_pos_pd(self.dof_pos_target, self.kp, self.kd)
        #     # print(self.target_input_torque)
        #     self.motor.set_target_input_torque(self.target_input_torque)
        #     time.sleep(1/200)
        
        data = {
            "dof_pos": self.motor.dof_pos,
            "dof_vel": self.motor.dof_vel,
            "commands": self.commands,
            "base_ang_vel": self.base_ang_vel,
            "projected_gravity": self.projected_gravity,
            "actions": self.action,
            # "actions_to_use":self.action_to_use,
            "base_quat": self.base_quat,
            "contact": self.contact,
            "contact_target": self.contact_target,
            # "dof_force_target": self.target_input_torque,
            "dof_force_target": self.motor.target_dof_torque_Nm,
            "dof_pos_target": self.dof_pos_target,
        }

        if self.enablePassiveDynamics:
            data["action_is_on"] = self.action_is_on

        self.data_publisher.publish({"real":data})
        
        
        sleep_t = self.dt*self.decimation
        # time.sleep(dt)
        elapsed = time.time() - self.t
        if elapsed < sleep_t:
            time.sleep(sleep_t - elapsed)
        self.t = time.time()

    def update_command(self):
        if self.data_receiver_data_id != self.data_receiver.data_id:
            self.data_receiver_data_id = self.data_receiver.data_id
            if "cmd" in self.data_receiver.data:
                self.commands[:] = np.array(self.data_receiver.data["cmd"])
                print(f"{self.commands[:]}")
            if "reset" in self.data_receiver.data and self.data_receiver.data["reset"]:
                self.should_reset = True

    def update_action(self):
        self.action = self.model.get_action(self.obs_buf)[0]
        alpha = 0.9
        self.action_to_use = alpha*self.action + (1-alpha)*self.action
        # self.action = self.model.get_action(self.obs_buf)[0]
        self.dof_pos_target = self.action_to_use * self.action_scale + self.default_dof_pos
        
    def update_action_passive(self):
        self.action = self.model.get_action(self.obs_buf)[0]
        alpha = 0.9
        self.action_to_use = alpha*self.action + (1-alpha)*self.action_to_use
        
        # self.action = self.model.get_action(self.obs_buf)[0]
        self.dof_pos_target = self.action_to_use[:self.num_dof] * self.action_scale + self.default_dof_pos
        # self.action_is_on = self.action_to_use[self.num_dof:]>-0.5
        self.action_is_on = sigmoid_k(self.action_to_use[self.num_dof:], 10)
    
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
        self.base_quat = np.array(self.sensor_data['base_quat'])

        self.force_measurement = self.sensor_data["force_measurement"]
        self.contact = self.sensor_data["contact"]
        
        # self.base_ang_vel = np.zeros(3)
        # self.projected_gravity = np.array([0,0,-1.0])
        
        self.update_phase()

        # dof_vel = self.motor.dof_vel*0.95 + self.dof_vel * 0.05
        # self.dof_vel = dof_vel
        
        # TODO, add scales
        self.obs_dict = {
            # 'linearVelocity': sensor_data['linearVelocity'], # DOES NOT HAVE THIS IN REAL
            # "linearVelocity": np.zeros(3), # HACK: FOR DEBUGGING
            "angularVelocity": self.base_ang_vel * self.ang_vel_scale,
            "projectedGravity": self.projected_gravity,

            "commands":  self.commands * self.commands_scale,
            "dofPosition": self.motor.dof_pos * self.dof_pos_scale,
            "dofVelocity": self.motor.dof_vel* self.dof_vel_scale,
            # "heightMap":
            # "actions": np.zeros(10), # HACK: FOR DEBUGGING
            "actions": self.action,
            "actionFiltered": self.action_to_use,
            # "phase": self.phase_sin_cos,
            "contactTarget":self.contact_target,
            "contact":self.contact,
        }
        self.obs_buf = np.concatenate(
            itemgetter(*self.obs_names)(self.obs_dict), axis=-1
        )


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

    # time.sleep(3)

    controller.initialize_stance()
    
    dt = 0.0001
    
    np.set_printoptions(formatter={"float": "{: 3.2f}".format})
    
    t = time.time()
    
    while True: # i in range(1000000):
        controller.step()
        
        # elapsed = time.time() - t
        # if elapsed < dt:
        #     time.sleep(dt - elapsed)
        # t = time.time()
            

