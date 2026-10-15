// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_multiple_positions.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
// #define ADDR_GOAL_VELOCITY 104
#define ADDR_PROFILE_VELOCITY 108
#define ADDR_PROFILE_ACCELERATION 112

#define MIN_POSITION 1024
#define MAX_POSITION 3072


// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

int degreeToPosition(int motor_id, float degree)
{
    if (degree < 0.0) degree = 0.0;
    if (degree > 180.0) degree = 180.0;

    if (motor_id == 1) {
        // Mapping for Motor ID 1 (180 deg -> 1587, 0 deg -> 3784)
        return static_cast<int>(1587 + ((180.0 - degree) / 180.0) * (3784 - 1587));
    } 
    else if (motor_id == 2) {
        // Mapping for Motor ID 2 (0 deg -> 1845, 180 deg -> 4030)
        return static_cast<int>(1845 + (degree / 180.0) * (4030 - 1845));
    } 

    return -1; // Invalid motor ID
}


ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
    RCLCPP_INFO(this->get_logger(), "Run read write node");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // Ensure the subscriber is using the correct message type (SetMultiplePositions)
set_position_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::SetMultiplePositions>(
    "set_position",
    QOS_RKL10V,
    [this](const dynamixel_sdk_custom_interfaces::msg::SetMultiplePositions::SharedPtr msg) -> void
    {
        for (size_t i = 0; i < msg->ids.size(); ++i) {
            int motor_id = msg->ids[i];
            float target_degree = msg->position;  // Target degree should be the same for both motors

            int goal_position = degreeToPosition(motor_id, target_degree);
            if (goal_position == -1) {
                RCLCPP_WARN(this->get_logger(), "Invalid motor ID: %d. Skipping...", motor_id);
                continue;
            }

            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                static_cast<uint8_t>(motor_id),
                ADDR_GOAL_POSITION,
                static_cast<uint32_t>(goal_position),
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d (Degrees: %.2f)]", motor_id, goal_position, target_degree);
            }
        }
    }
);

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "This is the modified version of the node.");


  uint8_t dxl_error = 0;

  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,  // Position Control Mode
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Set Default Profile Velocity
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_PROFILE_VELOCITY,
    500,  // Set your constant velocity here
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Profile Velocity.");
  } else if (dxl_error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "%s", packetHandler->getRxPacketError(dxl_error));
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Profile Velocity to 5000.");
  }

  // Set Default Profile Acceleration
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_PROFILE_ACCELERATION,
    700,  // Set your constant acceleration here
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Profile Acceleration.");
  } else if (dxl_error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "%s", packetHandler->getRxPacketError(dxl_error));
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Profile Acceleration to 7000.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,  // Enable Torque
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}


int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
