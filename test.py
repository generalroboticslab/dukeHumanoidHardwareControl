import numpy as np
import build.simple_test as simple_test
interface_name = "enp3s0"
control_mode = 8
# target_input = np.array([0.5, 0.2])  # Example NumPy array
target_input = 0.5  # Example NumPy array
max_velocity = 0.5
# print(type(interface_name), type(control_mode), type(target_input), type(max_velocity))

# motor=simple_test.Motor(interface_name, control_mode, target_input, max_velocity)

# motor=simple_test.Motor()
# motor.run()

simple_test.do_work_in_thread()

# sudo "$(which python)" test.py