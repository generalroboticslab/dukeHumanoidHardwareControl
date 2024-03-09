from build import pybind_test

worker = pybind_test.Worker()
worker.start_work(5) 
print("Work started, doing other things...")

# ... later
worker.stop_work() 