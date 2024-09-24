import serial  # conda install -c conda-forge pyserial
from cobs import cobs  # pip install cobs
import crc8
import struct
import time
from multiprocessing import Process, Queue
from queue import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np
from serial.tools import list_ports
from numpy_ringbuffer import RingArrayBuffer,filterBuffer


def quaternion_to_rotation_matrix(q):
  """Converts a quaternion to a rotation matrix.

  Args:
    q: A numpy array representing a quaternion in the format [x, y, z, w].

  Returns:
    A 3x3 numpy array representing the rotation matrix.
  """

  x, y, z, w = q

  R = np.array([
    [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
  ])

  return R


class SerialDataCollector:

    # Bit masks for fresh data
    FRESH_LOAD_CELL_0 = 0x0001
    FRESH_LOAD_CELL_1 = 0x0002
    FRESH_LOAD_CELL_2 = 0x0004
    FRESH_LOAD_CELL_3 = 0x0008
    FRESH_LOAD_CELL_ALL = 0x000F
    FRESH_FILTER_STATUS = 0x0010
    FRESH_LINEAR_ACCEL = 0x0020
    FRESH_QUATERNION = 0x0040
    FRESH_ROTATION_MATRIX = 0x0080
    FRESH_GRAVITY_VECTOR = 0x0100
    FRESH_GYRO = 0x0200
    FRESH_ACCEL = 0x0400

    def __init__(self, baudrate, queue, max_queue_size=1):
        self.ser = None
        self.baudrate = baudrate
        # Hexadecimal VendorID=0x16c0 & ProductID=0x483
        self.vendor_id = 0x16c0
        self.product_id = 0x483
        self.sample_timestamps = RingArrayBuffer(buffer_len=100, shape=1,dtype=np.float64)  # List to store sample timestamps
        self.sample_timestamps.storage[:] = time.time()
        
        self.unpacked = None
        self.running = False
        self.queue = queue
        self.max_queue_size = max_queue_size

        self.filter_for_ang_vel = filterBuffer(shape=3,fs=500, filter_order=4,cut_off_frequency=20,dtype=np.float64)
        self.filter_for_lin_acc = filterBuffer(shape=3,fs=500, filter_order=4,cut_off_frequency=20,dtype=np.float64)

        self.filter_force_measurement = filterBuffer(shape=4,fs=200, filter_order=4,cut_off_frequency=20,dtype=np.float64)
        # self.filter_force_measurement.buf_filt.storage[:]=2000
        # self.filter_force_measurement.buf_raw.storage[:]=2000

        # Initialize data variables

        self.ang_vel =  np.zeros(3,dtype=np.float64)
        self.lin_acc =  np.zeros(3,dtype=np.float64)

        self.lin_acc_raw = np.zeros(3,dtype=np.float64)

        self.ang_vel_raw =  np.zeros(3,dtype=np.float64)

        self.gravity_vec =  np.zeros(3,dtype=np.float64)
        self.base_quat = np.array([0, 0, 0, 1],dtype=np.float64)
        self.rotationMatrix = np.zeros(9,dtype=np.float64)
        self.ang_vel_raw = np.zeros(3,dtype=np.float64)
        
        self.lin_acc_imu_filtered = np.zeros(3,dtype=np.float64)
        self.ang_vel_imu_filtered = np.zeros(3,dtype=np.float64)
        
        self.force_measurement_raw = np.zeros(4,dtype=np.int32)
        self.force_measurement = np.zeros(4,dtype=np.int32) # filtered
        self.force_measurement_min = np.ones(4,dtype=np.int32)*5000

        #  NOTE: MUST match the actual values of the force sensor at zero contact
        self.force_measurement_min = np.array([730,1020,1000,780])
        # self.force_measurement_raw_buf = RingArrayBuffer(buffer_len=20,shape=(4,),dtype=int)
        self.force_measurement_threshold = 400

        self.contact = np.zeros(2,dtype=bool)
        self.last_contact = np.zeros_like(self.contact)
        self.contact_filt = np.zeros_like(self.contact)
        

        self.filterStatus = 0
        self.filterDynamicsMode = 0
        self.filterStatusFlags = 0
        self.fresh = 0

    def find_port(self):
        ports = list_ports.comports()
        for port in ports:
            if port.vid == self.vendor_id and port.pid == self.product_id:
                return port.device
        raise serial.SerialException("Teensy not connected!")

    def connect(self):
        self.ser = None
        port = self.find_port()
        if port:
            try:
                self.ser = serial.Serial(port, self.baudrate, timeout=1)
                self.ser.set_low_latency_mode(True)
                print(f"Connected to {port}")
            except serial.SerialException as e:
                print(f"Failed to connect to {port}: {e}")
        else:
            print("No suitable port found!")


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
        # # see if there is a connection and retry to connect if not
        if not self.ser:
            self.connect()
            if not self.ser:
                self.queue.put(None)
                return
            
        received_data = bytearray()
        while True:
            try:
                byte = self.ser.read(1)  # Read a single byte
            except serial.SerialException as e:
                self.connect()
                self.queue.put(None)
                return
            
            if byte == b'\x00':  # Check if it's the null byte
                break
            received_data += byte

        try:
            decoded = cobs.decode(received_data)
        except cobs.DecodeError:
            print("bad COBS") 
            # self.queue.put(None)
            # print("CRC mismatch")
            return

        try:
            # self.unpacked = struct.unpack("@hhhhffffffffffffffffffffffffffffhhhh", decoded[1:-1])
            self.unpacked = struct.unpack("@hhhhfffffffffffffffffffhhhh", decoded[1:-1]) # no rotation matrix
            self.sample_timestamps.add(time.time())  # Add timestamp for the new sample

            # x is forward, y is left, z is up
            # in the imu coordinate x is forward, y is right, z is down
            # so have to negate y and z
            # Update data variables
            
            self.fresh = self.unpacked[26] # [35]

            # force_measurement_fresh = bool(self.fresh & self.FRESH_LOAD_CELL_ALL)
            # if force_measurement_fresh:

            self.force_measurement_raw[:] = self.unpacked[0:4] # 0 is left-back, 1 left-front, 2 right-back, 3 right-front
            self.force_measurement[:] = self.filter_force_measurement.add(self.force_measurement_raw)
            self.contact[0] = (self.force_measurement[0] + self.force_measurement[1]-self.force_measurement_min[0]-self.force_measurement_min[1])>self.force_measurement_threshold 
            self.contact[1] = (self.force_measurement[2] + self.force_measurement[3]-self.force_measurement_min[2]-self.force_measurement_min[3])>self.force_measurement_threshold
            
            self.lin_acc_raw[:] = self.unpacked[4:7]
            self.ang_vel_raw[:] = self.unpacked[7:10]
            self.gravity_vec[:] = self.unpacked[10:13]
            # self.rotationMatrix[:] = self.unpacked[13:22]
            # 4
            self.base_quat[:] = self.unpacked[13:17] # [22:26]
            self.lin_acc_imu_filtered[:] = self.unpacked[17:20] # [26:29]
            self.ang_vel_imu_filtered[:] = self.unpacked[20:23] #[29:32]
            self.filterStatus = self.unpacked[23] #[32]
            self.filterDynamicsMode = self.unpacked[24] #[33]
            self.filterStatusFlags = self.unpacked[25] #[34]
            self.sps = 100/(self.sample_timestamps[0][0] - self.sample_timestamps[-1][0]) # sample per second
            
            # # filtered data
            self.ang_vel[:] = self.filter_for_ang_vel.add(self.ang_vel_imu_filtered)
            self.lin_acc[:] = self.filter_for_lin_acc.add(self.lin_acc_imu_filtered)
            
            # self.rot_from_quat = quaternion_to_rotation_matrix(self.base_quat)

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
            'force_measurement': self.force_measurement_raw,
            'self.force_measurement_min': self.force_measurement_min,
            'lin_acc': self.lin_acc,
            'contact': self.contact,
            # 'lin_acc_raw': self.lin_acc_raw,
            # 'lin_acc_filtered': self.lin_acc_imu_filtered,
            'ang_vel': self.ang_vel,
            # 'ang_vel_raw': self.ang_vel_raw, #self.gyroData_filtered, # self.gyroData,
            # 'ang_vel_filtered': self.ang_vel_imu_filtered,
            'base_quat': self.base_quat,
            # 'rot': self.rotationMatrix,
            # 'rot_form_quat': self.rot_from_quat.ravel(),
            # 'rot_debug': self.rotationMatrix.ravel()/self.rot_from_quat.ravel(),
            # 'rotationMatrix': self.rotationMatrix,
            'gravity_vec': self.gravity_vec,
            # 'debug': np.linalg.norm(self.base_quat),
            # 'filterStatus': self.filterStatus,
            # 'filterDynamicsMode': self.filterDynamicsMode,
            # 'filterStatusFlags': self.filterStatusFlags,
            # 'fresh': self.interpret_fresh_bytes(self.fresh),
            # 'timestamp': self.sample_timestamps[0],
            'sps': self.sps
        }
    
    def interpret_fresh_bytes(self):
        fresh_dict = {
            'forceSensorData_0': bool(self.fresh & self.FRESH_LOAD_CELL_0),
            'forceSensorData_1': bool(self.fresh & self.FRESH_LOAD_CELL_1),
            'forceSensorData_2': bool(self.fresh & self.FRESH_LOAD_CELL_2),
            'forceSensorData_3': bool(self.fresh & self.FRESH_LOAD_CELL_3),
            'filterStatus': bool(self.fresh & self.FRESH_FILTER_STATUS),
            'linearAccel': bool(self.fresh & self.FRESH_LINEAR_ACCEL),
            'quaternionData': bool(self.fresh & self.FRESH_QUATERNION),
            # 'rotationMatrix': bool(self.fresh & self.FRESH_ROTATION_MATRIX),
            'gravityVector': bool(self.fresh & self.FRESH_GRAVITY_VECTOR),
            'gyroData': bool(self.fresh & self.FRESH_GYRO),
            'accelData': bool(self.fresh & self.FRESH_ACCEL),
        }
        return fresh_dict


class SensorController:
    def __init__(self, baudrate=1500000):
        self.queue = Queue(maxsize=1)
        self.data_collector = SerialDataCollector(baudrate, self.queue)

    def start(self):
        self.data_collector.start()

    def stop(self):
        self.data_collector.stop()

    def get_samples_per_second(self):
        return self.data_collector.get_samples_per_second()

    def get_latest_data(self):
        try:
            return self.queue.get(timeout=0.1)
        except Empty:
            print("No data received!")
            return None
        
if __name__ == "__main__":
    
    from publisher import DataPublisher
    
    publisher = DataPublisher('udp://localhost:9870',encoding="msgpack",broadcast=False)
    
    # Initialize the SensorController instance
    sensors = SensorController()

    # Start the collection process
    sensors.start()

    target_fps = 500
    frame_time = 1 / target_fps

    try:
        while True:
            start_time = time.time()
            latest_data = sensors.get_latest_data()
            if latest_data:
                # print(f"{latest_data['sps']:.2f}")
                # print(latest_data['sps'])
                # print(latest_data['gyroData'])
                print(latest_data['contact'],latest_data['force_measurement'])

                publisher.publish({"sensors":latest_data})
                pass
            else:
                print("sensorController: No data received!")
            
            elapsed_time = time.time() - start_time
            sleep_time = frame_time - elapsed_time
            
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        # Stop the data collection gracefully on interrupt
        sensors.stop()
        print("Data collection stopped.")
