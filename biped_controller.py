# DEBUG LIBRARIES
import sys
import time
import signal
import onnxruntime as ort
import numpy as np


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

    list_of_ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]

    def __init__(self) -> None:

        # self.obs_names = [
        #     # "linearVelocity",
        #     "angularVelocity",
        #     "projectedGravity",
        #     "commands",
        #     "dofPosition",
        #     "dofVelocity",
        #     "actions",
        # ]
        
        self.obs_names = [
            # "linearVelocity",
            "angularVelocity",
            "projectedGravity",
            "commands",
            "dofPosition",
            "dofVelocity",
            "actions",
            "contactTarget",
            "phase",
        ]
        self.lin_vel_scale = 2.0
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.commands_scale = np.array([self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale])
        
        self.command_zero_threshold = 0.05
        
        self.action = np.zeros(10)
        
        self.decimation = 4
        self.kp = 60
        self.kd = 2
        self.action_scale = 1
        self.default_dof_pos=np.array([0.000,0.175,0.000,0.387,-0.213,0.000,-0.175,0.000,-0.387,0.213])

        self.phase_freq = 1.4 # [Hz]
        self.phase_stance_ratio = 0.6 # [1]
        self.phase_offset = np.array([0,0.5])
        
        # initialize sensors and data collection
        self.sensors = self.initialize_serial_comm() # Initialize serial comm for sensor data
        self.sensor_data = self.sensors.get_latest_data()
        # wait until sensor data is ready
        while self.sensor_data is None:
            self.sensor_data = self.sensors.get_latest_data()
            print("Waiting for sensor data to be ready...")
            time.sleep(0.01)
        print("Sensor data is ready!")
        
        # initialize motors
        self.motor = MotorController(
            "enp3s0", CONTROL_MODE.CYCLIC_SYNC_TORQUE, 20, 1000
        )
        self.motor.run()
        
        # data receiver
        self.data_receiver = DataReceiver(port=9871,decoding="msgpack",broadcast=True)
        self.data_receiver_data_id = self.data_receiver.data_id # for check if data is new
        self.data_receiver.receive_continuously()
        
        self.data_receiver2 = DataReceiver(port=9876,decoding="msgpack",broadcast=False)
        self.data_receiver2_data_id = self.data_receiver2.data_id # for check if data is new
        self.data_receiver2.receive_continuously()
        self.dof_pos_target = self.default_dof_pos
        
        self.data_publisher = DataPublisher('udp://localhost:9870',encoding="msgpack",broadcast=False)
        
        # command
        self.commands = np.zeros(3)
        
        
        # model_path = "policy.onnx"
        model_path="checkpoint/biped_c2r5h3e3/policy.onnx" # GOOD
        
        # model_path="checkpoint/biped_stepping_stone_trimesh_no_push/policy.onnx"
        
        # model_path="checkpoint/biped_c2r5h3e3p4/policy.onnx"  -BAD
        # model_path="checkpoint/biped_c2r6/policy.onnx" -BAD
        # model_path = "checkpoint/biped_c2r10/policy.onnx" -BAD
        # model_path = "checkpoint/biped_c2r5h3/policy.onnx"
        # model_path="checkpoint/biped_c2r5/policy.onnx" # bad
        # model_path="checkpoint/biped_dyn/policy.onnx" # bad
        # model_path ="checkpoint/biped_dyn_no_stance/policy.onnx" # bad
        # model_path = "checkpoint/biped_c2r5h3e3p6/policy.onnx" # bad
        # model_path = os.
        # init model
        self.model = Model(model_path)

        # setup signal handler if it has not been setup elsewhere
        if not signal.getsignal(signal.SIGINT):
            self.setup_signal_handler()

    def update_phase(self):
        """update normalized contact phase for each foot"""
        self.t = time.time()
        self.phase = (self.phase_freq * self.t  +self.phase_offset) % 1
        self.phase_sin_cos = np.array([np.sin(self.phase), np.cos(self.phase)])
        self.contact_target = self.phase < self.phase_stance_ratio
        self.is_zero_command = np.linalg.norm(self.commands[:3]) < self.command_zero_threshold
        self.contact_target[self.is_zero_command] = 1 # set contact_target to 1 if zero command
        self.commands[self.is_zero_command]=0

    def initialize_serial_comm(self):
        # Initialize serial communication for sensor data
        for (
            port
        ) in self.list_of_ports:  # try to connect to sensors on every possible port
            try:
                sensors = SensorController(port)
                sensors.start()
                print(f"Sensors connected to {port}")
                return sensors  # Return sucessful connection
            except Exception as e:
                print(f"Sensors failed to connect to {port}: {e}")

        raise (
            ConnectionError(
                "Failed to connect sensors to any port. Check USB is plugged in!"
            )
        )

    def initialize_stance(self):
        self.motor.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_TORQUE
        kp = 40
        kd =4.0 
        
        dof_pos_target =  self.default_dof_pos
        for _ in range(500):
            target_input_torque = self.motor.get_dof_pos_pd(dof_pos_target, kp, kd)
            
            self.motor.set_target_input_torque(target_input_torque)
            time.sleep(0.005)
        
        
    def step(self):
        # check if cmmd is updated
        self.update_command()
        
        # update observations
        self.update_observation()
        
        # compute action
        self.update_action()
        
        for _ in range(self.decimation):
            # do pd control
            self.target_input_torque = self.motor.get_dof_pos_pd(self.dof_pos_target, self.kp, self.kd)
            # print(self.target_input_torque)
            # self.target_input_torque = self.motor.get_dof_pos_pd(self.default_dof_pos, self.kp, self.kd)
            self.motor.set_target_input_torque(self.target_input_torque*1.05)
            time.sleep(1/400)

            
        
    def update_command(self):
        if self.data_receiver_data_id != self.data_receiver.data_id:
            self.data_receiver_data_id = self.data_receiver.data_id
            if "cmd" in self.data_receiver.data:
                self.commands[:] = np.array(self.data_receiver.data["cmd"])
                print(f"{self.commands[:]}")

    def update_action(self):
        # action = self.model.get_action(self.obs_buf)[0]
        # alpha = 0.9
        # self.action = alpha*action + (1-alpha)*self.action
        
        self.action = self.model.get_action(self.obs_buf)[0]
        self.dof_pos_target = self.action * self.action_scale + self.default_dof_pos
        
        # if self.data_receiver2_data_id != self.data_receiver2.data_id:
        #     self.data_receiver2_data_id = self.data_receiver2.data_id
        #     if "dof_pos_target" in self.data_receiver2.data:
        #         self.dof_pos_target[:] = np.array(self.data_receiver2.data["dof_pos_target"])
    
    def update_observation(self):
        # Return a dictionary of observable data

        self.sensor_data = self.sensors.get_latest_data()
        # print(self.sensor_data["sps"])
        self.base_ang_vel = np.array(self.sensor_data["gyroData"])
        self.projected_gravity = np.array(self.sensor_data["gravityVector"])
        self.base_quat = np.array(self.sensor_data['quaternionData'])
        
        # self.base_ang_vel = np.zeros(3)
        # self.projected_gravity = np.array([0,0,-1.0])
        
        self.update_phase()
        
        # TODO, add scales
        self.obs_dict = {
            # 'linearVelocity': sensor_data['linearVelocity'], # DOES NOT HAVE THIS IN REAL
            # "linearVelocity": np.zeros(3),  # TODO: Fix this
            "angularVelocity": self.base_ang_vel * self.ang_vel_scale,
            "projectedGravity": self.projected_gravity,
            "commands":  self.commands * self.commands_scale,
            "dofPosition": self.motor.dof_pos * self.dof_pos_scale,
            "dofVelocity": self.motor.dof_vel* self.dof_vel_scale,
            # "heightMap":
            # "actions": np.zeros(10),  # TODO: Fix this
            "actions": self.action,  # TODO: Fix this
            "phase": self.phase_sin_cos,
            "contactTarget":self.contact_target,
        }
        self.obs_buf = np.concatenate(
            itemgetter(*self.obs_names)(self.obs_dict), axis=-1
        )
        data = {
            "dof_pos": self.motor.dof_pos,
            "dof_vel": self.motor.dof_vel,
            "commands": self.commands,
            "base_ang_vel": self.base_ang_vel,
            "projected_gravity": self.projected_gravity,
            "actions": self.action,
            "base_quat": self.base_quat,
        }

        self.data_publisher.publish({"real":data})

    def setup_signal_handler(self):
        def signal_handler(sig, frame):
            print("Keyboard Interrupt!")
            self.shutdown()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

    def shutdown(self):
        # Ensure motors and sensors are properly stopped
        if self.motor:
            self.motor.set_should_terminate(True)
        if self.sensors:
            self.sensors.stop()
        print("BipedController has been shut down cleanly.")



# DEBUGGING
if __name__ == "__main__":
    controller = BipedController()

    time.sleep(3)

    controller.initialize_stance()
    
    dt = 0.0001
    
    np.set_printoptions(formatter={"float": "{: 3.2f}".format})
    
    t = time.time()
    
    for i in range(1000000):
        controller.step()
        
        # elapsed = time.time() - t
        # if elapsed < dt:
        #     time.sleep(dt - elapsed)
        # t = time.time()
            
