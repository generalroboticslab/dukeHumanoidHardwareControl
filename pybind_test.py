from build import pybind_test
import signal
import time 
import numpy as np


# worker = pybind_test.BackgroundWorker()
worker = pybind_test.BackgroundWorker("enp3s0",8,1.1,2.0)

# exit()
def signal_handler(sig, frame):
    print("Keyboard Interrupt detected. Attempting to stop background thread...")
    worker.set_should_terminate(True)
    exit()
signal.signal(signal.SIGINT, signal_handler)

# # Add values (direct assignment)
# worker.data_vector.append(10)
# worker.data_vector.append(20)
# worker.data_vector[0] = 10
worker.change_data_vector(np.array([11,12,13]))
print(worker.data_vector)  # Output: [11,12,13]
# Start the background thread 
worker.start_background_thread(10) 
# Do other things in Python, but give the background thread time to check the flag
for _ in range(10):
    print(f"Doing other Python tasks:t={worker.t}")
    time.sleep(1)
    print(worker.data_vector)  # Output: [50, 20]


print("End of Python program") 