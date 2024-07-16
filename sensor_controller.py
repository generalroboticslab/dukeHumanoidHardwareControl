# Author: Michael Scutari
# Description: This script reads Serial data and decodes it using COBS and CRC8. For IMU and load cells.
# © 2024 General Robotics Lab, Thomas Lord Department of Mechanical Engineering, Duke University
# License: MIT License

# Author: Michael Scutari
# Description: This script reads Serial data and decodes it using COBS and CRC8. For IMU and load cells.
# © 2024 General Robotics Lab, Thomas Lord Department of Mechanical Engineering, Duke University
# License: MIT License

import serial  # conda install -c conda-forge pyserial
from cobs import cobs  # pip install cobs
import crc8
import struct
import time
from multiprocessing import Process, Queue
from queue import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np

class SerialDataCollector:
    def __init__(self, port, baudrate, queue, max_queue_size=1):
        self.ser = serial.Serial(port, baudrate)
        self.ser.set_low_latency_mode(True)
        self.sample_count = 0
        self.start_time = time.time()
        self.unpacked = None
        self.running = False
        self.queue = queue
        self.max_queue_size = max_queue_size

        # Initialize data variables
        self.forceSensorData = [0, 0, 0, 0]
        self.accelData = [0, 0, 0]
        self.gyroData = [0, 0, 0]
        self.gravityVector = [0, 0, 0]
        self.quaternionData = [0, 0, 0, 1]
        self.rotationMatrix = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.linearAccel = [0, 0, 0]
        self.filterStatus = 0
        self.filterDynamicsMode = 0
        self.filterStatusFlags = 0
        self.fresh = 0

    def start(self):
        self.running = True
        self.process = Process(target=self.read_loop)
        self.process.start()

    def stop(self):
        self.running = False
        self.process.join()

    def read_loop(self):
        while self.running:
            self.read()

    def read(self):
        received_data = bytearray()
        while True:
            byte = self.ser.read(1)  # Read a single byte
            if byte == b'\x00':  # Check if it's the null byte
                break
            received_data += byte

        try:
            decoded = cobs.decode(received_data)
        except cobs.DecodeError:
            print("COBS decoding failed")
            return

        hash = crc8.crc8()
        hash.update(decoded[1:-1])

        if decoded[-1].to_bytes(1, byteorder='little') != hash.digest():
            print("CRC mismatch")
            return

        try:
            self.unpacked = struct.unpack("@hhhhfffffffffffffffffffffffffhhhh", decoded[1:-1])
            self.sample_count += 1
            # x is forward, y is left, z is up
            # in the imu coordinate x is forward, y is right, z is down
            # so have to negate y and z
            # Update data variables
            self.forceSensorData = [self.unpacked[3], self.unpacked[2], self.unpacked[0], self.unpacked[1]] # 0 is left-back, 1 left-front, 2 right-back, 3 right-front
            self.accelData = [-self.unpacked[4], self.unpacked[5], self.unpacked[6]] # negate x
            self.gyroData = [self.unpacked[7], -self.unpacked[8], -self.unpacked[9]] # negate y and z
            self.gravityVector = [-self.unpacked[10]/9.81, self.unpacked[11]/9.81, self.unpacked[12]/9.81] # gravity vector negate x
            self.rotationMatrix = [self.unpacked[13], self.unpacked[14], self.unpacked[15], self.unpacked[16], self.unpacked[17], self.unpacked[18], self.unpacked[19], self.unpacked[20], self.unpacked[21]]
            self.quaternionData = [self.unpacked[23], -self.unpacked[24], -self.unpacked[25], self.unpacked[22]] # quaternion reordered to xyzw and axes y and z reversed
            self.linearAccel = [self.unpacked[26], -self.unpacked[27], -self.unpacked[28]] # negative y and z
            self.filterStatus = self.unpacked[29]
            self.filterDynamicsMode = self.unpacked[30]
            self.filterStatusFlags = self.unpacked[31]
            self.fresh = [self.unpacked[32]]

            # Send data to the main process
            latest_data = self.get_latest_data()
            if self.queue.qsize() >= self.max_queue_size:
                try:
                    self.queue.get_nowait()  # Remove oldest item
                except Empty:
                    pass
            self.queue.put(latest_data)

        except struct.error as e:
            print("Struct unpacking error:", e)

    def get_latest_data(self):
        return {
            'forceSensorData': self.forceSensorData,
            'accelData': self.accelData,
            'gyroData': self.gyroData,
            'quaternionData': self.quaternionData,
            'rotationMatrix': self.rotationMatrix,
            'gravityVector': self.gravityVector,
            'filterStatus': self.filterStatus,
            'linearAccel': self.linearAccel,
            'filterDynamicsMode': self.filterDynamicsMode,
            'filterStatusFlags': self.filterStatusFlags,
            'fresh': self.fresh,
            'timestamp': time.time(),
            'sps': self.get_samples_per_second()
        }

    def get_samples_per_second(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            return self.sample_count / elapsed_time
        else:
            return 0



class SensorController:
    def __init__(self, port="/dev/cu.usbmodem154158101", baudrate=921600):
        self.queue = Queue(maxsize=1)
        self.data_collector = SerialDataCollector(port, baudrate, self.queue)

    def start(self):
        self.data_collector.start()

    def stop(self):
        self.data_collector.stop()

    def get_samples_per_second(self):
        return self.data_collector.get_samples_per_second()

    def get_latest_data(self):
        try:
            return self.queue.get()
        except Empty:
            return None
        
        
        

def initialize_serial_comm() -> SensorController:
    # Initialize serial communication for sensor data
    for (
        port
    ) in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]:  # try to connect to sensors on every possible port
        try:
            sensors = SensorController(port, baudrate="921600")
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
    
if __name__ == "__main__":


    # Initialize the SerialCommunication instance with the provided port
    ser_comm = SensorController(port="/dev/ttyACM0", baudrate=921600)

    # Start the data collection
    ser_comm.start()

    target_fps = 50
    frame_time = 1 / target_fps

    try:
    
        while True:
            start_time = time.time()
            
            latest_data = ser_comm.get_latest_data()
            if latest_data:
                print(f"{latest_data['sps']:.2f}")
                # print(latest_data['sps'])
            else:
                print(latest_data)
            
            elapsed_time = time.time() - start_time
            sleep_time = frame_time - elapsed_time
            
            if sleep_time > 0:
                time.sleep(sleep_time)

#     # Continuously print out the unpacked data
#     while True:
#         latest_data = ser_comm.get_latest_data()
#         if latest_data:

#             print(f"{latest_data['sps']:.2f}")
#             # print(latest_data['sps'])


#         #Sleep for a short duration to avoid flooding the output
#         time.sleep(0.5)
    except KeyboardInterrupt:
        # Stop the data collection gracefully on interrupt
        ser_comm.stop()
        print("Data collection stopped.")
            