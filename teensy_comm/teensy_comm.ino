// Author: Michael Scutari
// Description: Read code from LORD Microstrain 3DM-CV7 IMU and FX29 Load Cell. Send data over serial with COBS encoding.
// © 2024 General Robotics Lab, Thomas Lord Department of Mechanical Engineering, Duke University
// License: MIT License

// #define DEBUG
//  modified 8/11
#define PACKETIZER_USE_INDEX_AS_DEFAULT
#define PACKETIZER_USE_CRC_AS_DEFAULT

// must be before the import of Packetizer
#include <Wire.h>       // I2C
#include <TCA9548.h>    // I2C multiplexer
#include <Packetizer.h> // serial with computer
#include <math.h>

// I2C addresses
#define LOADCELL_ADDR 0x28
#define MULTIPLEXER_ADDR 0x70

constexpr float GRAVITY = 9.80665;

// CONSTANTS
// IMU
const int bufferSize = 256;
byte buffer[bufferSize];
int bufferIndex = 0;
bool ackReceived = false; // IMU acknowledgment for commands
unsigned long commandTimeout = 1000;

// Filter and sensor sample rates
const uint16_t sensor_sample_rate = 500; // Hz
const uint16_t filter_sample_rate = 500; // Hz

//*--------------force sensor---------------------------------------------------------
//
bool sampleForceSensors = true;

// multiplexer
PCA9548 multiplexer(MULTIPLEXER_ADDR);

// sensor timing and data collection.
unsigned long previousForceSensorCollectionTime = 0; // ms
const long forceSensorCollectionInterval = 5;        // ms //typical 5 ms
//*---------------------------------------------------------------------------

// Packetizer
// send and recv index
const uint8_t recv_index = 0x12;
const uint8_t send_index = 0x34;

// Finite state machine
enum State
{
  WAIT_FOR_U,
  WAIT_FOR_E,
  WAIT_FOR_DESCRIPTOR,
  WAIT_FOR_LENGTH,
  READ_PACKET
};

union FloatBytes {
    float f;
    byte b[4];
}__attribute__((packed));

// Data stream types
enum DescriptorSet
{
  BASE_COMMAND = 0x01,
  _3DM_COMMAND = 0x0C,
  FILTER_COMMAND = 0x0D,
  SENSOR_DATA = 0x80,
  FILTER_DATA = 0x82,
  SYSTEM_DATA = 0xA0,
  SHARED_DATA = 0xFF
};

enum DataFieldDescriptors
{
  FILTER_FILTER_STATUS = 0x10, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm
  FILTER_ATT_QUATERNION = 0x03, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_attitude_quaternion.htm?Highlight=0x03
  FILTER_ATT_EULER_ANGLES = 0x05, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_euler_angles.htm
  FILTER_ATT_DCM = 0x04, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_attitude_dcm.htm
  FILTER_LINEAR_ACCEL = 0x0D, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_linear_accel.htm
  FILTER_GRAVITY_VECTOR = 0x13, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_gravity_vector.htm
  FILTER_ANGULAR_RATE = 0x0E, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/filter_data/data/mip_field_filter_comp_angular_rate.htm?Highlight=0x0E
  SENSOR_SCALED_ACCEL = 0x04, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/sensor_data/data/mip_field_sensor_scaled_accel.htm
  SENSOR_SCALED_GYRO = 0x05, // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/sensor_data/data/mip_field_sensor_scaled_gyro.htm
};

enum CommandSendDescriptors
{
  COMMAND_PING = 0x01,
  COMMAND_BUILT_IN_TEST = 0x05,
  COMMAND_GET_BASE_DATA_RATE = 0x0E,
  COMMAND_SET_TO_IDLE = 0x02,
  COMMAND_DEVICE_SETTINGS = 0x30,
  COMMAND_DATASTREAM_CONTROL = 0x11,
  COMMAND_DEFAULT_SETTINGS = 0x30,
  COMMAND_MESSAGE_FORMAT = 0x0F,
  COMMAND_RESUME_SAMPLING = 0x06,
  COMMAND_RESET_NAV_FILTER = 0x01,
  COMMAND_SET_HEADING = 0x03,
  COMMAND_AIDING_MEASUREMENT = 0x50,
  COMMAND_VEHICLE_TRANSFORM_EULER = 0x31,
};

// Function Selectors // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/mip_common.html?Highlight=function%20selectors
// Most settings commands support a function selector, which allows the setting to be changed, read back, saved, loaded, or reset to the default. Some commands don't support all 5 options. For most settings, no additional parameters are needed when the function selector is not 0x01. However certain commands need more information (for example, which subset of settings to read back).
// 0x01 - Apply new setting
// 0x02 - Read current setting
// 0x03 - Save current setting to non-volatile memory
// 0x04 - Load current setting from non-volatile memory
// 0x05 - Restore factory default setting
enum FunctionSelectors
{
  SELECT_WRITE = 0x01,
  SELECT_READ = 0x02,
  SELECT_SAVE = 0x03,
  SELECT_LOAD = 0x04,
  SELECT_RESET = 0x05,
};

enum CommandReceiveDescriptors
{
  RECV_BUILT_IN_TEST = 0x83,
  RECV_AIDING_MEASURMENT = 0xD0,
  RECV_GET_BASE_DATA_RATE = 0x8E,
  RECV_DATASTEAM_CONTROL = 0x85,
};

// Structs for storing packet information.
struct PacketField
{
  byte descriptor;
  byte length;
  std::vector<byte> data;
};

struct IMUPacket
{
  byte descriptorSet;
  byte length;
  std::vector<PacketField> fields;
};

// for collecting data in one place and sending
struct CombinedData
{
  // load cell measurements
  int16_t force_measurement[4]; // hhhh

  // accelerometer data
  float lin_acc_raw[3]; // fff

  // gyroscope data
  float ang_vel_raw[3]; // fff

  // gravity vector estimation filter
  float gravity_vector[3]; // fff

  // // attitude rotation matrix
  // float rotation_matrix[9]; // fffffffff

  // quaternion attitude estimation filter
  float quaternion[4]; // ffff

  // gravity-compensated linear acceleration
  float lin_acc_filtered[3]; // fff

  // gravity-compensated angular velocity
  float ang_vel_filtered[3]; // fff

  // filter status, check domumentation
  uint16_t filter_status; // h
  // dynamic mode. check documentation
  uint16_t filter_dynamics_mode; // h
  // status flags. processed in python code.
  uint16_t filter_status_flags; // h
  // check which are fresh.
  uint16_t fresh; // h // 00000 (nothing), 00 (accel, gyro), 0000 (gravity vector, rotation matrix, quaternion, linear accel), 0 (status, dynamics mode, status flag), 0000 (force sensor 3, 2, 1, 0)
} __attribute__((packed));

// Packetizer combined data object
CombinedData combinedData;

// IMU current packet information
State currentState = WAIT_FOR_U;
byte descriptor;
int payload_length;

////////////////////////////////////
// IMU variables (updated by IMU) //
////////////////////////////////////

uint16_t filter_base_rate = 0;
uint16_t sensor_base_rate = 0;

// verifying channels are activate when enabling datastream
bool sensor_channel_active = false;
bool filter_channel_active = false;
bool aiding_measurements_active = true;

// for timing
unsigned long startTime;

void setup()
{
  Serial.begin(1500000);
  Serial1.begin(921600);
  Wire.begin();
  Wire.setClock(921600UL);

  // start all data as not fresh
  combinedData.fresh = 0;

  // Initialize the IMU with default settings.
  if (setToIdle())
  {
    Serial.println("Successfully set to idle.");
  }
  else
  {
    Serial.println("Failed to set to idle.");
  }

  Serial.println();

  if (loadDefaultSettings())
  {
    Serial.println("Successfully loaded default settings.");
  }
  else
  {
    Serial.println("Failed to load default settings.");
  }

  Serial.println();

  // Configure sensor data.
  if (getDataBaseRate(SENSOR_DATA))
  {
    Serial.print("Sensor base data rate: ");
    Serial.println(sensor_base_rate);
  }
  else
  {
    Serial.println("Failed to get base data rate.");
  }

  Serial.println();

  const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

  // Set Message Format
  std::vector<std::pair<byte, uint16_t>> sensorDescriptorsAndDecimators = {
      {SENSOR_SCALED_ACCEL, sensor_decimation}, // Accelerometer
      {SENSOR_SCALED_GYRO, sensor_decimation},  // Gyroscope
                                                // {0x0E, sensor_decimation}  // Magnetometer
  };

  if (setMessageFormat(SENSOR_DATA, sensorDescriptorsAndDecimators))
  {
    Serial.println("Sensor data message format successfully updated.");
  }
  else
  {
    Serial.println("Failed to set message format.");
  }

  Serial.println();

  if (enableDatastream(SENSOR_DATA))
  {
    Serial.println("Successfully enabled sensor data.");
  }
  else
  {
    Serial.println("Failed to enable sensor datastream.");
  }

  // if (disableDatastream(SENSOR_DATA)) // HACK  disble sensor data
  // {
  //   Serial.println("Successfully disabled sensor data.");
  // }
  // else
  // {
  //   Serial.println("Failed to disabled sensor datastream.");
  // }

  Serial.println();

  if (verifyDatastream(SENSOR_DATA))
  {
    Serial.println("Verified: Sensor datastream enabled.");
  }
  else
  {
    Serial.println("Verification failed: Sensor data not available.");
  }

  Serial.println();

  // Configure filter data.

  if (resetNavigationFilter())
  {
    Serial.println("Navigation filter reset successfully.");
  }
  else
  {
    Serial.println("Failed to reset navigation filter.");
  }

  if (setHeadingControl())
  { // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/filter_command/data/mip_filter_set_initial_heading.htm
    Serial.println("Heading control set successfully.");
  }
  else
  {
    Serial.println("Failed to set heading control.");
  }

  if (loadAllAidingMeasurement())
  { // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/filter_command/data/mip_filter_aiding_measurement_enable.htm
    Serial.println("Loaded all aiding mesurements successfully.");
  }
  else
  {
    Serial.println("Failed to load all aiding measurements.");
  }

  Serial.println();

  if (sensorToVehicleTransform(0.0, 0.0, 0.0))
  {
    Serial.println("Sensor to vehicle transform successfully updated.");
  }
  else
  {
    Serial.println("Failed to updated sensor to vehicle transform.");
  }

  Serial.println();

  if (readSensorToVehicleTransform())
  {
    Serial.println("Reading sensor to vehicle transform.");
  }
  else
  {
    Serial.println("Failed to updated sensor to vehicle transform.");
  }

  if (getDataBaseRate(FILTER_DATA))
  {
    Serial.print("Filter base data rate: ");
    Serial.println(filter_base_rate);
  }
  else
  {
    Serial.println("Failed to get base data rate.");
  }

  Serial.println();

  const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

  // Set Message Format
  std::vector<std::pair<byte, uint16_t>> filterDescriptorsAndDecimators = {
      {FILTER_ATT_QUATERNION, filter_decimation}, // Attitude quaternion
      {FILTER_FILTER_STATUS, filter_decimation},  // Filter Status
      {FILTER_GRAVITY_VECTOR, filter_decimation}, // Gravity vector
      // {FILTER_ATT_DCM, filter_decimation},        // Attitude rotation matrix
      {FILTER_LINEAR_ACCEL, filter_decimation},   // Gravity-compensated linear acceleration,
      {FILTER_ANGULAR_RATE, filter_decimation},   // Filter-compensated gyro
  };

  if (setMessageFormat(FILTER_DATA, filterDescriptorsAndDecimators))
  {
    Serial.println("Filter data message format successfully updated.");
  }
  else
  {
    Serial.println("Failed to set message format.");
  }

  Serial.println();

  if (enableDatastream(FILTER_DATA))
  {
    Serial.println("Successfully enabled filter data.");
  }
  else
  {
    Serial.println("Failed to enable filter datastream.");
  }

  Serial.println();

  if (verifyDatastream(FILTER_DATA))
  {
    Serial.println("Verified: Filter datastream enabled.");
  }
  else
  {
    Serial.println("Verification failed: Filter data not available.");
  }

  Serial.println();

  startTime = millis();

  if (resumeSampling())
  {
    Serial.println("IMU sampling resumed.");
  }
  else
  {
    Serial.println("Failed to resume IMU sampling.");
  }
}

void loop()
{
  unsigned long currentMillis = millis();

  // Read force sensor data at regular intervals
  if ((currentMillis - previousForceSensorCollectionTime >= forceSensorCollectionInterval) && sampleForceSensors)
  {
    previousForceSensorCollectionTime = currentMillis;
    for (int i = 0; i < 4; i++)
    {
      readLoadCell(i);
    }
  }

  noInterrupts();  // Disable interrupts
  if (shouldSendPacket()) {

    uint8_t* data = reinterpret_cast<uint8_t*>(&combinedData);
    Packetizer::send(Serial, send_index, data, sizeof(combinedData));
    combinedData.fresh = 0;
  }
  interrupts();  // Re-enable interrupts
}

// Interrupt on receiving data with finite state machine.
void serialEvent1()
{
  while (Serial1.available())
  {
    byte incomingByte = Serial1.read();

    // Serial.println(incomingByte, HEX); // DEBUG

    switch (currentState)
    {
    case WAIT_FOR_U:
      if (incomingByte == 'u')
      {
        bufferIndex = 0;
        buffer[bufferIndex++] = incomingByte;
        currentState = WAIT_FOR_E;
      }
      break;

    case WAIT_FOR_E:
      if (incomingByte == 'e')
      {
        buffer[bufferIndex++] = incomingByte;
        currentState = WAIT_FOR_DESCRIPTOR;
      }
      else
      {
        currentState = WAIT_FOR_U;
        bufferIndex = 0;
      }
      break;

    case WAIT_FOR_DESCRIPTOR:
      buffer[bufferIndex++] = incomingByte;
      descriptor = incomingByte;
      currentState = WAIT_FOR_LENGTH;
      break;

    case WAIT_FOR_LENGTH:
      buffer[bufferIndex++] = incomingByte;
      payload_length = (int)incomingByte;
      if (payload_length > bufferSize - 6)
      { // Check if length is valid
        Serial.println("Invalid packet length");
        currentState = WAIT_FOR_U;
        bufferIndex = 0;
      }
      else
      {
        currentState = READ_PACKET;
      }
      break;

    case READ_PACKET:
      if (bufferIndex < bufferSize)
      {
        buffer[bufferIndex++] = incomingByte;
        if (bufferIndex >= payload_length + 6)
        {
          processPacket(buffer, payload_length + 6); // Process the complete packet
          currentState = WAIT_FOR_U;
        }
      }
      else
      {
        Serial.println("Buffer overflow detected");
        currentState = WAIT_FOR_U;
      }
      break;
    }
  }
}

void processPacket(byte *packet, int packet_length)
{
  if (!verifyChecksum(packet, packet_length))
  {
    Serial.println("Bad checksum.");
    ackReceived = false;
    return;
  }

  IMUPacket response_packet = parsePacket(packet, packet_length);

  // Handle different data and command response packets.
  if (response_packet.descriptorSet == BASE_COMMAND)
  {

    handleBaseCommand(response_packet);
  }
  else if (response_packet.descriptorSet == _3DM_COMMAND)
  {

    handle3DMCommand(response_packet);
  }
  else if (response_packet.descriptorSet == SENSOR_DATA)
  {

    handleSensorData(response_packet);
  }
  else if (response_packet.descriptorSet == FILTER_COMMAND)
  {

    handleFilterCommand(response_packet);
  }
  else if (response_packet.descriptorSet == FILTER_DATA)
  {

    handleFilterData(response_packet);
  }
  else
  {

    Serial.print("ERROR - Unrecognized packet descriptor set: ");
    Serial.println(response_packet.descriptorSet, HEX);
  }
}

void handleBaseCommand(IMUPacket packet)
{
  ackReceived = fieldsContainACK(packet);
  for (const auto &field : packet.fields)
  {
    if (field.descriptor == COMMAND_BUILT_IN_TEST && field.data.size() == 6)
    {
      for (byte data : field.data)
      {
        printBinaryWithLeadingZeros(data);
      }
      Serial.println();
    }
  }
}

void handle3DMCommand(IMUPacket packet)
{
  ackReceived = fieldsContainACK(packet);

  if (!ackReceived)
  {
    return;
  }

  for (const auto &field : packet.fields)
  {

    if (field.descriptor == RECV_GET_BASE_DATA_RATE && field.data.size() == 3)
    { // getBaseDataRate
      if (field.data[0] == FILTER_DATA)
      {
        filter_base_rate = (field.data[1] << 8) | field.data[2];
      }
      else if (field.data[0] == SENSOR_DATA)
      {
        sensor_base_rate = (field.data[1] << 8) | field.data[2];
      }
    }
    else if (field.descriptor == RECV_DATASTEAM_CONTROL)
    { // Datastream control

      if (field.data[0] == SENSOR_DATA && field.data.size() == 2)
      {
        sensor_channel_active = field.data[1];
      }
      else if (field.data[1] == FILTER_DATA && field.data.size() == 2)
      {
        filter_channel_active = field.data[1];
      }
    }
  }
}

void handleFilterCommand(IMUPacket packet)
{
  ackReceived = fieldsContainACK(packet);

  if (!ackReceived)
  {
    return;
  }

  // Serial.println("Received filter command.");

  for (const auto &field : packet.fields)
  {
    if (field.descriptor == RECV_AIDING_MEASURMENT && field.data.size() == 3)
    {
      if (field.data[0] == 0xFF && field.data[1] == 0xFF)
      {
        if (field.data[2] == 0x01)
        {
          aiding_measurements_active = true;
        }
        else
        {
          aiding_measurements_active = false;
        }
      }
    }
  }
}

// inline void convertAndStoreFloats(const std::vector<byte>& data, float* destination, int numFloats) {
//     FloatBytes fb;
//     for (int i = 0; i < numFloats; ++i) {
//         // Copy bytes into the union, assuming big-endian order in data
//         fb.b[0] = data[i * 4 + 3];
//         fb.b[1] = data[i * 4 + 2];
//         fb.b[2] = data[i * 4 + 1];
//         fb.b[3] = data[i * 4];

//         // Access the float value, now in the correct endianness
//         destination[i] = fb.f;
//     }
// }

void handleFilterData(IMUPacket packet)
{
  for (const auto &field : packet.fields)
  { // linear acceleration
    if (field.descriptor == FILTER_LINEAR_ACCEL && field.length == 0x10)
    {

      bool valid = (bool)((field.data[12] << 8) | field.data[13]);
      if (valid)
      {
        // byte x[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
        // byte y[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
        // byte z[] = {field.data[11], field.data[10], field.data[9], field.data[8]};
        // memcpy(&combinedData.lin_acc_filtered[0], x, sizeof(float));
        // memcpy(&combinedData.lin_acc_filtered[1], y, sizeof(float));
        // memcpy(&combinedData.lin_acc_filtered[2], z, sizeof(float));


        // Reinterpret the entire gravity_vector array as an array of FloatBytes
        FloatBytes* fb = reinterpret_cast<FloatBytes*>(combinedData.lin_acc_filtered);
        for (int i = 0; i < 3; ++i) {
            // Copy bytes, assuming big-endian order in field.data
            fb[i].b[0] = field.data[i * 4 + 3];
            fb[i].b[1] = field.data[i * 4 + 2];
            fb[i].b[2] = field.data[i * 4 + 1];
            fb[i].b[3] = field.data[i * 4];
        }
        // negate y and z
        // combinedData.lin_acc_filtered[0] = combinedData.lin_acc_filtered[0];
        combinedData.lin_acc_filtered[1] = -combinedData.lin_acc_filtered[1];
        combinedData.lin_acc_filtered[2] = -combinedData.lin_acc_filtered[2];


        combinedData.fresh |= (1 << 5);
      }
    }
    // else if (field.descriptor == FILTER_ATT_DCM && field.length == 0x28)
    // { 
    //   bool valid = (bool)((field.data[36] << 8) | field.data[37]);
    //   if (valid)
    //   {
    //     // // rotation matrix
    //     FloatBytes fb[9];
    //     for (int i = 0; i < 9; ++i) {
    //         // Copy bytes into the union, assuming big-endian order in field.data
    //         fb[i].b[0] = field.data[i * 4 + 3];
    //         fb[i].b[1] = field.data[i * 4 + 2];
    //         fb[i].b[2] = field.data[i * 4 + 1];
    //         fb[i].b[3] = field.data[i * 4];
    //     }
    //     combinedData.rotation_matrix[0] = fb[0].f;
    //     combinedData.rotation_matrix[1] = -fb[3].f;
    //     combinedData.rotation_matrix[2] = -fb[6].f;
    //     combinedData.rotation_matrix[3] = -fb[1].f;
    //     combinedData.rotation_matrix[4] = fb[4].f;
    //     combinedData.rotation_matrix[5] = fb[7].f;
    //     combinedData.rotation_matrix[6] = -fb[2].f;
    //     combinedData.rotation_matrix[7] = fb[5].f;
    //     combinedData.rotation_matrix[8] = fb[8].f;

    //     combinedData.fresh |= (1 << 7);
    //   }
    // }
    else if (field.descriptor == FILTER_GRAVITY_VECTOR && field.length == 0x10)
    { // gravity vector

      bool valid = (bool)((field.data[12] << 8) | field.data[13]);
      if (valid)
      {
        // byte x[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
        // byte y[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
        // byte z[] = {field.data[11], field.data[10], field.data[9], field.data[8]};
        // memcpy(&combinedData.gravity_vector[0], x, sizeof(float));
        // memcpy(&combinedData.gravity_vector[1], y, sizeof(float));
        // memcpy(&combinedData.gravity_vector[2], z, sizeof(float));

      // FloatBytes fb[3];
      // for (int i = 0; i < 3; ++i) {
      //     // Copy bytes into the union, assuming big-endian order in field.data
      //     fb[i].b[0] = field.data[i * 4 + 3];
      //     fb[i].b[1] = field.data[i * 4 + 2];
      //     fb[i].b[2] = field.data[i * 4 + 1];
      //     fb[i].b[3] = field.data[i * 4];
      //     // Access the float value, now in the correct endianness
      //     combinedData.gravity_vector[i] = fb[i].f/GRAVITY; // normalize to [g]
      // }

      // Reinterpret the entire gravity_vector array as an array of FloatBytes
      FloatBytes* fb = reinterpret_cast<FloatBytes*>(combinedData.gravity_vector);
      for (int i = 0; i < 3; ++i) {
          // Copy bytes, assuming big-endian order in field.data
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
          // Access and normalize the float value in place
          // fb[i].f /= GRAVITY; 
      }
      combinedData.gravity_vector[0] = - combinedData.gravity_vector[0]/GRAVITY; // negate x
      combinedData.gravity_vector[1] = combinedData.gravity_vector[1]/GRAVITY;
      combinedData.gravity_vector[2] = combinedData.gravity_vector[2]/GRAVITY;


        combinedData.fresh |= (1 << 8);
      }
    }
    else if (field.descriptor == FILTER_ATT_QUATERNION && field.length == 0x14)
    { // quaternion

      bool valid = (bool)((field.data[16] << 8) | field.data[17]);

      if (valid)
      {
        // byte w[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
        // byte x[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
        // byte y[] = {field.data[11], field.data[10], field.data[9], field.data[8]};
        // byte z[] = {field.data[15], field.data[14], field.data[13], field.data[12]};

        // memcpy(&combinedData.quaternion[0], w, sizeof(float));
        // memcpy(&combinedData.quaternion[1], x, sizeof(float));
        // memcpy(&combinedData.quaternion[2], y, sizeof(float));
        // memcpy(&combinedData.quaternion[3], z, sizeof(float));

        
        FloatBytes fb[4];
        for (int i = 0; i < 4; ++i) {
            // Copy bytes into the union, assuming big-endian order in field.data
            fb[i].b[0] = field.data[i * 4 + 3];
            fb[i].b[1] = field.data[i * 4 + 2];
            fb[i].b[2] = field.data[i * 4 + 1];
            fb[i].b[3] = field.data[i * 4];
        }
        // fb in wzyz order, need to convert to xyzw and negate y and z
        combinedData.quaternion[0] = fb[1].f;
        combinedData.quaternion[1] = - fb[2].f;
        combinedData.quaternion[2] = - fb[3].f;
        combinedData.quaternion[3] = fb[0].f;


        combinedData.fresh |= (1 << 6);
      }
    }
    // else if (field.descriptor == FILTER_ATT_EULER_ANGLES && field.data.size() == 14) {  // euler angles

    //   byte x[] = { field.data[3], field.data[2], field.data[1], field.data[0] };
    //   byte y[] = { field.data[7], field.data[6], field.data[5], field.data[4] };
    //   byte z[] = { field.data[11], field.data[10], field.data[9], field.data[8] };

    //   bool valid = (bool)((field.data[12] << 8) | field.data[13]);

    //   if (valid) {
    //     Serial.println("ERROR: Operating in quaternion mode.");
    //   }

    // }
    else if (field.descriptor == FILTER_FILTER_STATUS && field.data.size() == 6)
    { // filter status

      combinedData.filter_status = (field.data[0] << 8) | field.data[1];
      combinedData.filter_dynamics_mode = (field.data[2] << 8) | field.data[3];
      combinedData.filter_status_flags = (field.data[4] << 8) | field.data[5];

      combinedData.fresh |= (1 << 4);
    }
    else if (field.descriptor == FILTER_ANGULAR_RATE && field.length == 0x10)
    { // scaled accelerometer values

      // byte x[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
      // byte y[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
      // byte z[] = {field.data[11], field.data[10], field.data[9], field.data[8]};

      // memcpy(&combinedData.ang_vel_filtered[0], x, sizeof(float));
      // memcpy(&combinedData.ang_vel_filtered[1], y, sizeof(float));
      // memcpy(&combinedData.ang_vel_filtered[2], z, sizeof(float));


      // Reinterpret the entire gravity_vector array as an array of FloatBytes
      FloatBytes* fb = reinterpret_cast<FloatBytes*>(combinedData.ang_vel_filtered);
      for (int i = 0; i < 3; ++i) {
          // Copy bytes, assuming big-endian order in field.data
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
      }
      // negate y and z
      // combinedData.ang_vel_filtered[0] = combinedData.ang_vel_filtered[0];
      combinedData.ang_vel_filtered[1] = -combinedData.ang_vel_filtered[1];
      combinedData.ang_vel_filtered[2] = -combinedData.ang_vel_filtered[2];


      combinedData.fresh |= (1 << 10);
    }

    else
    {
      Serial.print("Unrecognized data type! ");
      Serial.print("Descriptor: 0x");
      Serial.print(field.descriptor, HEX);
      Serial.print(", Length: 0x");
      Serial.println(field.length, HEX);
    }
  }
}

void handleSensorData(IMUPacket packet)
{
  for (const auto &field : packet.fields)
  {
    if (field.descriptor == SENSOR_SCALED_ACCEL && field.data.size() == 12)
    { // scaled accelerometer values

      // byte x[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
      // byte y[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
      // byte z[] = {field.data[11], field.data[10], field.data[9], field.data[8]};
      // memcpy(&combinedData.lin_acc_raw[0], x, sizeof(float));
      // memcpy(&combinedData.lin_acc_raw[1], y, sizeof(float));
      // memcpy(&combinedData.lin_acc_raw[2], z, sizeof(float));
      // // convert unit from [g] to [m/s^2]
      // combinedData.lin_acc_raw[0]*=GRAVITY;
      // combinedData.lin_acc_raw[1]*=GRAVITY;
      // combinedData.lin_acc_raw[2]*=GRAVITY;

      // FloatBytes fb;
      // for (int i = 0; i < 3; ++i) {
      //     // Copy bytes into the union, assuming big-endian order in field.data
      //     fb.b[0] = field.data[i * 4 + 3];
      //     fb.b[1] = field.data[i * 4 + 2];
      //     fb.b[2] = field.data[i * 4 + 1];
      //     fb.b[3] = field.data[i * 4];
      //     // Access the float value, now in the correct endianness
      //     combinedData.lin_acc_raw[i] = fb.f*GRAVITY;
      // }

      // Reinterpret the entire gravity_vector array as an array of FloatBytes
      FloatBytes* fb = reinterpret_cast<FloatBytes*>(combinedData.lin_acc_raw);
      for (int i = 0; i < 3; ++i) {
          // Copy bytes, assuming big-endian order in field.data
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
          // Access and normalize the float value in place
          // fb[i].f /= GRAVITY; 
      }
      // negate y and z
      combinedData.lin_acc_raw[0] = combinedData.lin_acc_raw[0]*GRAVITY;
      combinedData.lin_acc_raw[1] = -combinedData.lin_acc_raw[1]*GRAVITY;
      combinedData.lin_acc_raw[2] = -combinedData.lin_acc_raw[2]*GRAVITY;
      

      combinedData.fresh |= (1 << 10);
    }
    else if (field.descriptor == SENSOR_SCALED_GYRO && field.data.size() == 12)
    { // gyroscope

      // byte x[] = {field.data[3], field.data[2], field.data[1], field.data[0]};
      // byte y[] = {field.data[7], field.data[6], field.data[5], field.data[4]};
      // byte z[] = {field.data[11], field.data[10], field.data[9], field.data[8]};

      // memcpy(&combinedData.ang_vel_raw[0], x, sizeof(float));
      // memcpy(&combinedData.ang_vel_raw[1], y, sizeof(float));
      // memcpy(&combinedData.ang_vel_raw[2], z, sizeof(float));

      // FloatBytes fb;
      // for (int i = 0; i < 3; ++i) {
      //     // Copy bytes into the union, assuming big-endian order in field.data
      //     fb.b[0] = field.data[i * 4 + 3];
      //     fb.b[1] = field.data[i * 4 + 2];
      //     fb.b[2] = field.data[i * 4 + 1];
      //     fb.b[3] = field.data[i * 4];
      //     // Access the float value, now in the correct endianness
      //     combinedData.ang_vel_raw[i] = fb.f;
      // }

      // Reinterpret the entire gravity_vector array as an array of FloatBytes
      FloatBytes* fb = reinterpret_cast<FloatBytes*>(combinedData.ang_vel_raw);
      for (int i = 0; i < 3; ++i) {
          // Copy bytes, assuming big-endian order in field.data
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
      }
      // negate y and z
      // combinedData.ang_vel_raw[0] = combinedData.ang_vel_raw[0];
      combinedData.ang_vel_raw[1] = -combinedData.ang_vel_raw[1];
      combinedData.ang_vel_raw[2] = -combinedData.ang_vel_raw[2];



      // convertAndStoreFloats(field.data, combinedData.ang_vel_raw, 3);

      combinedData.fresh |= (1 << 9);
    }
    else
    {
      Serial.println("Unrecognized data type.");
    }
  }
}

// Code for parsing buffer into IMUPacket struct.
IMUPacket parsePacket(byte *buffer, int length)
{
  IMUPacket packet;
  packet.descriptorSet = buffer[2];
  packet.length = buffer[3];
  int index = 4;

#ifdef DEBUG
  Serial.println();
  for (int i = 0; i < length; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("Parsing Packet:");
  Serial.print("Descriptor Set: ");
  Serial.print(packet.descriptorSet, HEX);
  Serial.print(", Length: ");
  Serial.println(packet.length, HEX);
#endif

  while (index < length - 2)
  { // Subtract 2 for checksum
    PacketField field;
    field.length = buffer[index++];
    field.descriptor = buffer[index++];
    field.data.assign(buffer + index, buffer + index + field.length - 2);
    index += field.length - 2;
    packet.fields.push_back(field);

#ifdef DEBUG
    Serial.print("Length: ");
    Serial.print(field.length, HEX);
    Serial.print(", Field Descriptor: ");
    Serial.print(field.descriptor, HEX);
    Serial.print(", Data: ");
    for (byte b : field.data)
    {
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif
  }
  return packet;
}

// Verify if fletcher checksum is received.
bool verifyChecksum(byte *packet, int packet_length)
{
  uint16_t packet_checksum = (packet[packet_length - 2] << 8) | packet[packet_length - 1];
  uint16_t calculated_checksum = fletcher_checksum(packet, packet_length);
  return packet_checksum == calculated_checksum;
}

uint16_t fletcher_checksum(const uint8_t *packet, int packet_length)
{
  const int checksum_length = packet_length - 2;
  uint8_t checksum_MSB = 0;
  uint8_t checksum_LSB = 0;
  for (int i = 0; i < checksum_length; i++)
  {
    checksum_MSB += packet[i];
    checksum_LSB += checksum_MSB;
  }
  return ((uint16_t)checksum_MSB << 8) | (uint16_t)checksum_LSB;
}

bool fieldsContainACK(IMUPacket packet)
{
  for (const auto &field : packet.fields)
  {
    if (field.descriptor == 0xF1)
    { // Example: ACK bits position
      if (field.data[1] == 0x00)
      {
        return true;
      }
      else if (field.data[1] == 0x03)
      {
        Serial.println("Invalid parameter!");
      }
    }
  }
  return false;
}

bool checkACK()
{
  unsigned long startTime = millis();
  while (millis() - startTime < commandTimeout)
  {
    noInterrupts(); // Disable interrupts
    if (ackReceived)
    {
      ackReceived = false;
      interrupts(); // Re-enable interrupts
      return true;  // Acknowledgment received
    }
    interrupts(); // Re-enable interrupts
    yield();      // Yield to other processes
  }

  Serial.println("Timeout occurred!");
  return false; // Timeout occurred
}

// ADD Checksum to last two bytes of command.
void addChecksum(byte *command, int command_length)
{
  uint16_t checksum = fletcher_checksum(command, command_length);
  command[command_length - 2] = (byte)(checksum >> 8);   // MSB of checksum
  command[command_length - 1] = (byte)(checksum & 0xFF); // LSB of checksum
}

// LIST OF POTENTIAL COMMANDS
bool ping()
{
  int command_length = 8;
  byte command[command_length] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_PING, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool setToIdle()
{
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_SET_TO_IDLE, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool loadDefaultSettings()
{
  int command_length = 9;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x03, 0x03, COMMAND_DEFAULT_SETTINGS, SELECT_LOAD, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool getDataBaseRate(byte dataDescriptorSet)
{
  int command_length = 9;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x03, 0x03, COMMAND_GET_BASE_DATA_RATE, dataDescriptorSet, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}



// https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/3dm_command/data/mip_cmd_3dm_message_format.htm
bool setMessageFormat(byte descriptorSet, const std::vector<std::pair<byte, uint16_t>> &descriptorsAndDecimators)
{
  int numDescriptors = descriptorsAndDecimators.size();
  int command_length = 11 + numDescriptors * 3;

  std::vector<byte> command(command_length, 0);
  command[0] = 0x75;
  command[1] = 0x65;
  command[2] = _3DM_COMMAND;
  command[3] = 5 + numDescriptors * 3;
  command[4] = 5 + numDescriptors * 3;
  command[5] = COMMAND_MESSAGE_FORMAT;
  command[6] = 0x01;
  command[7] = descriptorSet;
  command[8] = numDescriptors;

  for (int i = 0; i < numDescriptors; ++i)
  {

    command[9 + i * 3] = descriptorsAndDecimators[i].first;
    command[10 + i * 3] = (byte)(descriptorsAndDecimators[i].second >> 8);
    command[11 + i * 3] = (byte)(descriptorsAndDecimators[i].second & 0xFF);
  }

  command[12 + numDescriptors * 3] = 0x00;
  command[13 + numDescriptors * 3] = 0x00;

  addChecksum(command.data(), command_length);
  Serial1.write(command.data(), command_length);

  return checkACK();
}

bool verifyDatastream(byte dataDescriptorSet)
{ // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/3dm_command/data/mip_cmd_3dm_datastream_control.htm
  int command_length = 10;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x04, 0x04, COMMAND_DATASTREAM_CONTROL, SELECT_READ, dataDescriptorSet, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool enableDatastream(byte dataDescriptorSet)
{
  int command_length = 11;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x05, 0x05, COMMAND_DATASTREAM_CONTROL, SELECT_WRITE, dataDescriptorSet, 0x01, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  if (checkACK())
  {
    if (dataDescriptorSet == SENSOR_DATA)
    {
      sensor_channel_active = true;
      return true;
    }
    else if (dataDescriptorSet == FILTER_DATA)
    {
      filter_channel_active = true;
      return true;
    }
    else
    {
      Serial.println("Unrecognized data descriptor set.");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool disableDatastream(byte dataDescriptorSet)
{
  int command_length = 11;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x05, 0x05, COMMAND_DATASTREAM_CONTROL, SELECT_WRITE, dataDescriptorSet, 0x00, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  if (checkACK())
  {
    if (dataDescriptorSet == SENSOR_DATA)
    {
      sensor_channel_active = false;
      return true;
    }
    else if (dataDescriptorSet == FILTER_DATA)
    {
      filter_channel_active = false;
      return true;
    }
    else
    {
      Serial.println("Unrecognized data descriptor set.");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool resumeSampling()
{
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_RESUME_SAMPLING, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool resetNavigationFilter()
{
  // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/filter_command/data/mip_filter_reset_filter.htm
  int command_length = 8;
  byte command[] = {0x75, 0x65, FILTER_COMMAND, 0x02, 0x02, COMMAND_RESET_NAV_FILTER, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool setHeadingControl()
{
  int command_length = 12;
  byte command[] = {0x75, 0x65, FILTER_COMMAND, 0x06, 0x06, COMMAND_SET_HEADING, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // set heading reference to 0

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool loadAllAidingMeasurement()
{ // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Commands/filter_command/data/mip_filter_aiding_measurement_enable.htm
  int command_length = 12;
  byte command[] = {0x75, 0x65, FILTER_COMMAND, 0x06, 0x06, COMMAND_AIDING_MEASUREMENT, SELECT_LOAD, 0xFF, 0xFF, 0x01, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

bool sensorToVehicleTransform(float x, float y, float z)
{ // Sensor to Vehicle Frame Transformation Euler

  // Split float32_ts into bytes and store them in big-endian order
  byte *xBytes = (byte *)&x;
  byte *yBytes = (byte *)&y;
  byte *zBytes = (byte *)&z;

  int command_length = 21;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x0F, 0x0F, COMMAND_VEHICLE_TRANSFORM_EULER, SELECT_WRITE, xBytes[3], xBytes[2], xBytes[1], xBytes[0], yBytes[3], yBytes[2], yBytes[1], yBytes[0], zBytes[3], zBytes[2], zBytes[1], zBytes[0], 0x00, 0x00};

  addChecksum(command, command_length); // replaces zeroes at end of command with checksum
  Serial1.write(command, command_length);

  return checkACK();
}

bool readSensorToVehicleTransform()
{ // Sensor to Vehicle Frame Transformation Euler

  int command_length = 9;
  byte command[] = {0x75, 0x65, _3DM_COMMAND, 0x03, 0x03, COMMAND_VEHICLE_TRANSFORM_EULER, SELECT_READ, 0x00, 0x00};

  addChecksum(command, command_length); // replaces zeroes at end of command with checksum
  Serial1.write(command, command_length);

  return checkACK();
}

bool runBuiltInTest()
{
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_BUILT_IN_TEST, 0x00, 0x00};

  addChecksum(command, command_length);
  Serial1.write(command, command_length);

  return checkACK();
}

///////////////////////
// LOAD CELL FUNCTIONS
///////////////////////

void readLoadCell(int index)
{
  multiplexer.selectChannel(index);
  Wire.requestFrom(LOADCELL_ADDR, 2); // Request 2 bytes of data
  if (Wire.available() >= 2)
  {
    uint8_t data[2];
    data[0] = Wire.read();                                                          // First byte of data
    data[1] = Wire.read();                                                          // Second byte of data
    uint8_t bridge_status = data[0] >> 6;                                           // Extract status bits, 2=old data, 0=new data

    combinedData.force_measurement[index] = (int16_t)(((data[0] & 0x3F) << 8) | data[1]); // Extract bridge data
    if (bridge_status == 2)
    {
      // 8/20: do nothing
      // // LOADCELL data is stale
      // combinedData.fresh &= ~(1 << index);
    }
    else
    {
      // LOADCELL data is fresh
      combinedData.fresh |= (1 << index);
    }
  }
  else
  {
    // DEBUG
    Serial.print("Loadcell no data: ");
    Serial.println(index);
  }
}

///////////////////////
// Other Functions
///////////////////////

bool shouldSendPacket()
{
  // if load cell data fresh
  uint16_t load_cell_mask = 0b0000000000001111;
  if ((load_cell_mask & combinedData.fresh) == load_cell_mask)
    return true;

  // h // 00000 (nothing), 00 (accel, gyro), 0000 (gravity vector, rotation matrix, quaternion, linear accel), 0 (status, dynamics mode, status flag), 0000 (force sensor 3, 2, 1, 0)

  // if sensor and filter are data fresh
  // uint16_t filter_and_sensor_mask = 0b0000011111110000;
  uint16_t filter_and_sensor_mask = 0b0000011101110000; // no rotation matrix

  if ((filter_and_sensor_mask & combinedData.fresh) == filter_and_sensor_mask)
    return true;

  // // if sensor data fresh
  // uint16_t sensor_mask = 0b0000011000000000;
  // if ((sensor_mask & combinedData.fresh) == sensor_mask) return true;

  // // if sensor data fresh
  // uint16_t filter_mask = 0b0000000111110000;
  // if ((filter_mask & combinedData.fresh) == filter_mask) return true;

  return false;
}

void printBinaryWithLeadingZeros(byte value)
{
  for (int i = 15; i >= 0; i--)
  {
    Serial.print(bitRead(value, i));
  }
  Serial.println();
}
