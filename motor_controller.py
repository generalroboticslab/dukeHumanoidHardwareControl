import numpy as np
import signal
import time
import publisher

import torch
import yaml

from numpy_ringbuffer import filterBuffer

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
        
        self.num_dof = 10
        # motor joint limits
        # [l_hip1,l_hip2,l_hip3,l_knee,l_ankle,r_hip1,r_hip2,r_hip3,r_knee,r_ankle]
        # self.min_limit = np.array([-pi/3, -pi/6, -pi/9,-pi/24,   -pi/5, -pi/4, -pi/2, -pi/5, -pi/2, -pi/5])
        # self.max_limit = np.array([pi/4,   pi/2,  pi/5,  pi/2,    pi/5,  pi/3,  pi/6,  pi/9, pi/24,  pi/5])
        min_limit = np.array([     -pi/4, -pi/2.5, -pi/8,   -0.3   ,  -pi*38/180,  -pi/4, -pi/2.5, -pi/5,   -pi/3,       -pi*40/180])
        max_limit = np.array([      pi/4,  pi/2.5,  pi/5,    pi/3   ,   pi*40/180,   pi/4,  pi/2.5,  pi/8,    0.3  ,      pi*38/180])
        # hard_min_limit = np.array([-pi/2, -pi/2,   -pi*2/9, -0.02  ,   -pi*36/180,  -pi/2, -pi/2,   -pi*2/9, -pi*5/9,     -pi*40/180])
        # hard_max_limit = np.array([ pi/2,  pi/2,    pi*2/9,  pi*5/9,    pi*40/180,   pi/2,  pi/2,    pi*2/9,  0.02  ,      pi*36/180])
        # soft_limit_offset =  min_limit - hard_min_limit
        # def arry2string(arr):
        #     return np.array2string(arr, separator=',',max_line_width=999,precision=5).replace(" ", "")
        # print(f"++task.env.assetDofProperties.lower={arry2string(hard_min_limit)}")
        # print(f"++task.env.assetDofProperties.upper={arry2string(hard_max_limit)}")
        # print(f"++task.env.dof_soft_limit.lower={arry2string(min_limit)}")
        # print(f"++task.env.dof_soft_limit.upper={arry2string(max_limit)}")
        
        self.min_limit = min_limit
        self.max_limit = max_limit

        gear_ratio =                    [18,   20,   18,   18,   10,   18,   20,   18,   18,   10]
        #                                0    ,1    ,2    ,3    ,4    ,5    ,6    ,7    ,8    ,9
        rotor_current_to_torque_ratio = [0.165,0.165,0.165,0.165,0.165,0.165,0.165,0.165,0.165,0.165] #default
        # defined in c++
        self.gear_ratio = np.array(gear_ratio,dtype=np.float64)
        self.rotor_current_to_torque_ratio=np.array(rotor_current_to_torque_ratio, dtype=np.float64)
        
        self.last_limit_check_time= time.time()

        fix_yaml_sequence()

        # self.motor_pos_offset = np.zeros(self.dof)
        with open("motor_config.yaml", "r") as file:
            config = yaml.safe_load(file)
            self.motor_pos_offset = np.array(config["motor_pos_offset"])
            self.set_position_offset(self.motor_pos_offset)

            print("motor_pos_offset:",repr(self.motor_pos_offset))
        # self.motor_pos_offset = np.array([0.06983874, 0.07876512, 0.30102242, 0.1620134 , 0.45463356,0.32062063, 0.21344384, 0.33541981, 0.22198779, 0.12190354])
        
        def signal_handler(sig, frame):
            print("Keyboard Interrupt detected. Attempting to stop background thread...")
            self.set_should_terminate(True)
            exit()

        signal.signal(signal.SIGINT, signal_handler)

    def limit_check(self, dof_pos_target):
        over_max_limit = (self.dof_pos > self.max_limit) & (dof_pos_target > self.max_limit)
        under_min_limit = (self.dof_pos < self.min_limit) & (dof_pos_target<self.min_limit)    
        if any(over_max_limit) or any(under_min_limit):
            t = time.time()
            if t - self.last_limit_check_time>0.5: # only print every 0.5 s
                print(f"\033[93mLimit check: index {np.flatnonzero(under_min_limit)} under min limit, index {np.flatnonzero(over_max_limit)} over max limit!\033[0m")
                self.last_limit_check_time = t
            result = np.where(over_max_limit, self.max_limit, dof_pos_target)
            result = np.where(under_min_limit, self.min_limit, result)
            return result
        else:
            return dof_pos_target  
        
    @property
    def dof_pos(self) -> np.ndarray:
        """returns current dof positions"""
        # return self._motor_value_to_dof_value(np.asarray(self.actual_position)-self.motor_pos_offset)
        return self.dof_position
    
    @property
    def motor_pos_after_offset(self) -> np.ndarray:
        """returns current dof positions"""
        return np.asarray(self.actual_position)-self.motor_pos_offset
    
    @property
    def _dof_vel_raw(self) -> np.ndarray:
        """returns current dof joint velocities"""
        return self._motor_value_to_dof_value(np.asarray(self.actual_velocity))
    
    @property
    def dof_vel(self) -> np.ndarray:
        """returns current dof joint velocities"""
        # return self.dof_velocity
        return self.dof_velocity_filtered
    
    @property
    def dof_current(self) -> np.ndarray:
        return np.asarray(self.actual_torque)

    @property
    def dof_force(self):
        return self.actual_torque*self.rotor_current_to_torque_ratio
        # return np.asarray(self.target_dof_torque_Nm)
    
    def run(self):
        if not self.running:
            self.running = True
            super().run()
            time.sleep(0.05)

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
        self.should_print = False # disable printing
        self.run()
        time.sleep(0.2)
        motor_actual_pos=np.asarray(self.actual_position).copy()
        desire_motor_pos = np.array([0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0])
        # motor_actual_pos - offset = desire_motor_pos
        offset = - desire_motor_pos + motor_actual_pos
        with open("motor_config.yaml", "w") as file:
            yaml.dump({"motor_pos_offset": offset}, file, width=float("inf"))
        print(np.array2string(offset, separator=', ',precision=5,max_line_width=200))

    # return joint position given actual motor position
    def _motor_value_to_dof_value(self, motor_value):
        """convert motor value to joint value, value can be position/velocity"""
        dof_value = np.copy(motor_value)
        dof_value[4] -= motor_value[3]
        dof_value[9] -= motor_value[8]
        return dof_value

    def _dof_value_to_motor_value(self, joint_value):
        """convert joint value to motor value, value can be position/velocity"""
        motor_value = np.copy(joint_value)
        motor_value[4] += motor_value[3] # left ankle
        motor_value[9] += motor_value[8] # right ankle
        return motor_value

    # get motor position given joint positions
    def _dof_pos_to_motor_pos_batch(self, joint_positions):
        motor_positions = np.copy(joint_positions)
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
