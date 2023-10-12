// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xFF;
  uint8_t detect_color;  // 0-red 1-blue
  uint8_t reset_tracker;
  uint8_t reserved;
  
  float roll;
  float pitch;
  float yaw;
  float speed;
  uint8_t checksum = 0xFE;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xFF;
  uint8_t tracking;
  // uint8_t id;          // 0-outpost 6-guard 7-base
  // uint8_t armors_num;  // 2-balance 3-outpost 4-normal
  uint8_t reserved;
  uint8_t fire_control = 1;
  float yaw;
  float pitch;
  uint8_t checksum = 0xFE;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
