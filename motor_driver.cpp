/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "include/motor_driver.h"

VisionMotorDriver::VisionMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID),
  wheel1_id_(DXL_F_LEFT_ID), wheel2_id_(DXL_F_RIGHT_ID), wheel3_id_(DXL_M_LEFT_ID), wheel4_id_(DXL_M_RIGHT_ID), wheel5_id_(DXL_R_LEFT_ID), wheel6_id_(DXL_R_RIGHT_ID)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

VisionMotorDriver::~VisionMotorDriver()
{
  close();
}

bool VisionMotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

bool VisionMotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_F_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_F_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_M_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_M_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_R_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_R_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0) {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

bool VisionMotorDriver::getTorque()
{
  return torque_;
}

void VisionMotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

bool VisionMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

bool VisionMotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_data_byte[4] = {0, };
  uint8_t right_data_byte[4] = {0, };


  left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t*)&right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool VisionMotorDriver::writePosition(int64_t left_value, int64_t right_value) {
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_data_byte[4] = {0, };
  uint8_t right_data_byte[4] = {0, };
  uint8_t zero_data_byte[4] = {0, };

  int64_t zero_value = 2048;
  zero_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(zero_value));
  zero_data_byte[1] = DXL_LOBYTE(DXL_LOWORD(zero_value));
  zero_data_byte[2] = DXL_LOBYTE(DXL_LOWORD(zero_value));
  zero_data_byte[3] = DXL_LOBYTE(DXL_LOWORD(zero_value));

  left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  dxl_addparam_result = groupSyncWritePosition_->addParam(wheel1_id_, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(wheel2_id_, (uint8_t*)&right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(wheel3_id_, (uint8_t*)&zero_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(wheel4_id_, (uint8_t*)&zero_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(wheel5_id_, (uint8_t*)&zero_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(wheel6_id_, (uint8_t*)&zero_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool VisionMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;

  float wheel_velocity_cmd[2];

  float lin_vel = value[LINEAR];
  float ang_vel = value[ANGULAR];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}

bool VisionMotorDriver::controlMotor_ackermann(float* value)
{
  bool dxl_comm_result = false;

  float wheel_position_cmd[2];

  float lin_vel = value[LINEAR];
  float ang_vel = value[ANGULAR];

  if (ang_vel < 0) {
    double radius = lin_vel / ang_vel;
    wheel_position_cmd[LEFT]   = 2048 - (std::atan2(WHEEL_BASE, radius + (AXLE_WIDTH / 2)) * 2048 / PI);
    wheel_position_cmd[RIGHT]  = 2048 - (std::atan2(WHEEL_BASE, radius - (AXLE_WIDTH / 2)) * 2048 / PI);
  }
  else if (ang_vel > 0) {
    double radius = lin_vel / ang_vel;
    wheel_position_cmd[LEFT]   = 2048 + (std::atan2(WHEEL_BASE, radius - (AXLE_WIDTH / 2)) * 2048 / PI);
    wheel_position_cmd[RIGHT]  = 2048 + (std::atan2(WHEEL_BASE, radius + (AXLE_WIDTH / 2)) * 2048 / PI);
  }
  else {
    wheel_position_cmd[LEFT]   = 2048;
    wheel_position_cmd[RIGHT]  = 2048;
  }

  wheel_position_cmd[LEFT]   = constrain(wheel_position_cmd[LEFT], 512, 3583);
  wheel_position_cmd[RIGHT]  = constrain(wheel_position_cmd[RIGHT], 512, 3583);

  dxl_comm_result = writePosition((int64_t)wheel_position_cmd[LEFT], (int64_t)wheel_position_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}