from build import pybind_test
import time
import sys

worker = pybind_test.Worker()
worker.start_work(30)
print("Work started, doing other things...")
try:
    for i in range(500):
        time.sleep(0.01)
except KeyboardInterrupt:
    print("stopping")
    worker.stop_work()
    exit()
