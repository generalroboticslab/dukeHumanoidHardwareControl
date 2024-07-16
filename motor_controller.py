import numpy as np
import signal
import time
import publisher

import torch
import yaml

from build import ethercat_motor_py as MOTOR


pi = np.pi

CONTROL_MODE = MOTOR.CONTROL_MODE


class MotorController(MOTOR.PyMotor):
    
    def __init__(
        self,
        interface_name = "enp3s0", 
        control_mode_int8 = None, 
        max_velocity = None, 
        max_torque=None,
        ):
        super().__init__(interface_name, control_mode_int8, max_velocity, max_torque)
        self.should_print = False
        self.debug=False
        
        self.running = False
        
        self.rated_current = np.asarray(self.rated_torque)
        
        self.dof = 10
        # motor joint limits
        # [l_hip1,l_hip2,l_hip3,l_knee,l_ankle,r_hip1,r_hip2,r_hip3,r_knee,r_ankle]
        self.min_limit = np.array([-pi/3, -pi/6, -pi/9,    0,   -pi/5, -pi/6, -pi/2, -pi/5, -pi/2, -pi/5])
        self.max_limit = np.array([pi/6,   pi/2,  pi/5, pi/2,    pi/5,  pi/3,  pi/6,  pi/9,     0,  pi/5])
        self.gear_ratio = np.array([18,20,18,18,10, 18,20,18,18,10],dtype=np.float64)
        self.rotor_current_to_torque_ratio = 0.155
        self.current_to_torque_ratio = self.rotor_current_to_torque_ratio*self.gear_ratio
    
        fix_yaml_sequence()

        # self.motor_pos_offset = np.zeros(self.dof)
        with open("motor_config.yaml", "r") as file:
            config = yaml.safe_load(file)
            self.motor_pos_offset = np.array(config["motor_pos_offset"])
            print("motor_pos_offset:",repr(self.motor_pos_offset))
        # self.motor_pos_offset = np.array([0.06983874, 0.07876512, 0.30102242, 0.1620134 , 0.45463356,0.32062063, 0.21344384, 0.33541981, 0.22198779, 0.12190354])
        
        def signal_handler(sig, frame):
            print("Keyboard Interrupt detected. Attempting to stop background thread...")
            self.set_should_terminate(True)
            exit()

        signal.signal(signal.SIGINT, signal_handler)

    @property
    def dof_pos(self) -> np.ndarray:
        """returns current dof positions"""
        return self._motor_value_to_dof_value(np.asarray(self.actual_position).copy()-self.motor_pos_offset)
    
    @property
    def dof_vel(self) -> np.ndarray:
        """returns current dof joint velocities"""
        return self._motor_value_to_dof_value(np.asarray(self.actual_velocity).copy())
    
    @property
    def dof_current(self) -> np.ndarray:
        return np.asarray(self.actual_torque)

    @property
    def dof_effort(self):
        return self.actual_torque*self.current_to_torque_ratio
    
    def run(self):
        if not self.running:
            self.running = True
            super().run()
            time.sleep(0.05)

    def get_dof_pos_pd(self, target_dof_pos, kp, kd):
        # TODO: verify this!
        dof_torque = kp*(target_dof_pos - self.dof_pos) - kd*self.dof_vel
        return dof_torque
    
    def limit_dof_pos(self, unsafe_joint_pos):
        safe_joint_pos = np.clip(unsafe_joint_pos)
        return
    
    def dof_pos_to_motor_pos_safe(self, joint_pos):
        motor_pos = np.clip(self._dof_value_to_motor_value(joint_pos), self.min_limit, self.max_limit) + self.motor_pos_offset
        return motor_pos
    
    def set_motor_pos_from_dof_pos_safe(self, joint_pos):
        motor_pos = np.clip(self._dof_value_to_motor_value(joint_pos), self.min_limit, self.max_limit) + self.motor_pos_offset
        self.set_target_input(motor_pos)
        
    def set_motor_pos(self, motor_pos):
        motor_pos = np.clip(motor_pos, self.min_limit, self.max_limit) + self.motor_pos_offset
        self.set_target_input(motor_pos)
    
    def set_motor_pos_without_offset(self, motor_pos):
        """directly set motor positions, not limited by joint limits, no offset"""
        self.set_target_input(np.asarray(motor_pos))
    
    def set_motor_pos_offset(self, motor_pos_offset):
        """set motor position offset"""
        self.motor_pos_offset = np.asarray(motor_pos_offset)
    
    def initialize_motor_pos_offset(self):
        self.control_mode_int8 = CONTROL_MODE.CYCLIC_SYNC_VELOCITY
        self.run()
        motor_actual_pos=np.asarray(self.actual_position).copy()
        # initialize the ankle to be poinitng downward
        desire_motor_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.698, 0.0, 0.0, 0.0, 0.0, -0.698])
        # motor_actual_pos - offset = desire_motor_pos
        offset = - desire_motor_pos + motor_actual_pos
        # np.set_printoptions(precision=4, formatter={'float_kind': lambda x: f"{x:.4f}"})  # Format floats to 4 decimal places
        with open("motor_config.yaml", "w") as file:
            yaml.dump({"motor_pos_offset": offset}, file, width=float("inf"))
        print(np.array2string(offset, separator=', ',precision=5,max_line_width=200))

    def set_target_input_torque(self,target_input_torque):
        target_current = target_input_torque/self.current_to_torque_ratio
        target_current_input = target_current/self.rated_current*1000
        # print(target_current_input)
        self.set_target_input(target_current_input)


    
    # return joint position given actual motor position
    def _motor_value_to_dof_value(self, motor_value):
        """convert motor value to joint value, value can be position/velocity"""
        joint_value = motor_value
        joint_value[4] -= motor_value[3]
        joint_value[9] -= motor_value[8]
        return joint_value

    def _dof_value_to_motor_value(self, joint_value):
        """convert joint value to motor value, value can be position/velocity"""
        motor_value = joint_value[:]
        motor_value[4] += motor_value[3] # left ankle
        motor_value[9] += motor_value[8] # right ankle
        return motor_value

    # get motor position given joint positions
    def _dof_pos_to_motor_pos_batch(self, joint_positions):
        motor_positions = joint_positions[:]
        motor_positions[:, 4] += motor_positions[:, 3] # left ankle
        motor_positions[:, 9] += motor_positions[:, 8] # right ankle
        return motor_positions

def fix_yaml_sequence(precision=5):
    """Fix yaml sequence dumper to have numerical sequence in one line"""
    
    def represent_array_sequence(dumper, data, precision=precision):
        if isinstance(data, torch.Tensor):
            data = data.cpu().numpy().astype(np.float64).round(precision).tolist()
        elif isinstance(data, np.ndarray):
            data = data.round(precision).tolist()
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

    # Register representers for lists, tuples, numpy arrays, and torch tensors
    for type in (list, tuple, np.ndarray, torch.Tensor):
        yaml.add_representer(type, represent_array_sequence)
    
# py38 && export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib && sudo $(which python) motor.py
if __name__ == "__main__":
    motor = MotorController("enp3s0", CONTROL_MODE.CYCLIC_SYNC_VELOCITY, 1, 1000)    
    motor.initialize_motor_pos_offset()
