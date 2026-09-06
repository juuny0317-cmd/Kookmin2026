// Copyright 2026 Xytron
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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vesc_driver/vesc_packet.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"

namespace
{

using vesc_driver::Buffer;
using vesc_driver::VescFrame;
using vesc_driver::VescPacketFWVersion;
using vesc_driver::VescPacketSetServoPos;
using vesc_driver::VescPacketValues;

Buffer makeFrame(const Buffer & payload)
{
  EXPECT_LT(payload.size(), 256u);
  Buffer frame(payload.size() + VescFrame::VESC_MIN_FRAME_SIZE, 0);
  frame[0] = VescFrame::VESC_SOF_VAL_SMALL_FRAME;
  frame[1] = static_cast<uint8_t>(payload.size());
  std::copy(payload.begin(), payload.end(), frame.begin() + 2);
  const uint16_t crc = CRC::Calculate(payload.data(), payload.size(), VescFrame::CRC_TYPE);
  frame[frame.size() - 3] = static_cast<uint8_t>(crc >> 8);
  frame[frame.size() - 2] = static_cast<uint8_t>(crc & 0xFF);
  frame[frame.size() - 1] = VescFrame::VESC_EOF_VAL;
  return frame;
}

vesc_driver::VescPacketPtr parsePayload(const Buffer & payload)
{
  Buffer frame = makeFrame(payload);
  int bytes_needed = 0;
  std::string error;
  auto packet = vesc_driver::VescPacketFactory::createPacket(
    frame.cbegin(), frame.cend(), &bytes_needed, &error);
  EXPECT_TRUE(packet) << error;
  EXPECT_EQ(bytes_needed, 0);
  return packet;
}

void putInt16(Buffer & payload, std::size_t offset, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  payload.at(offset) = static_cast<uint8_t>(raw >> 8);
  payload.at(offset + 1) = static_cast<uint8_t>(raw & 0xFF);
}

void putInt32(Buffer & payload, std::size_t offset, int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  payload.at(offset) = static_cast<uint8_t>(raw >> 24);
  payload.at(offset + 1) = static_cast<uint8_t>((raw >> 16) & 0xFF);
  payload.at(offset + 2) = static_cast<uint8_t>((raw >> 8) & 0xFF);
  payload.at(offset + 3) = static_cast<uint8_t>(raw & 0xFF);
}

TEST(VescFirmwareProtocol, ParsesLegacyVersionWithoutReadingExtendedFields)
{
  const Buffer payload{0, 2, 18};
  auto packet = std::dynamic_pointer_cast<VescPacketFWVersion>(parsePayload(payload));
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->fwMajor(), 2);
  EXPECT_EQ(packet->fwMinor(), 18);
  EXPECT_FALSE(packet->hasExtendedInfo());
  EXPECT_TRUE(packet->hwname().empty());
  for (std::size_t index = 0; index < 12; ++index) {
    EXPECT_EQ(packet->uuid()[index], 0);
  }
}

TEST(VescFirmwareProtocol, ParsesModernVersionAndMetadata)
{
  Buffer payload{0, 5, 3, 'V', 'E', 'S', 'C', 0};
  for (uint8_t value = 1; value <= 12; ++value) {
    payload.push_back(value);
  }
  payload.push_back(1);  // paired
  payload.push_back(0);  // compatibility byte
  payload.push_back(7);  // hardware type/version

  auto packet = std::dynamic_pointer_cast<VescPacketFWVersion>(parsePayload(payload));
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->fwMajor(), 5);
  EXPECT_EQ(packet->fwMinor(), 3);
  EXPECT_TRUE(packet->hasExtendedInfo());
  EXPECT_EQ(packet->hwname(), "VESC");
  EXPECT_TRUE(packet->paired());
  EXPECT_EQ(packet->devVersion(), 7);
  for (std::size_t index = 0; index < 12; ++index) {
    EXPECT_EQ(packet->uuid()[index], index + 1);
  }
}

TEST(VescFirmwareProtocol, ParsesLegacy218ValuesLayout)
{
  Buffer payload(56, 0);
  payload[0] = 4;
  putInt16(payload, 1, 450);
  putInt16(payload, 3, 460);
  putInt16(payload, 5, 470);
  putInt16(payload, 13, 480);
  putInt32(payload, 15, 1234);
  putInt32(payload, 19, -250);
  putInt16(payload, 23, 500);
  putInt32(payload, 25, 9000);
  putInt16(payload, 29, 120);
  putInt32(payload, 31, 12345);
  putInt32(payload, 47, -123);
  putInt32(payload, 51, 456);
  payload[55] = 2;

  auto packet = std::dynamic_pointer_cast<VescPacketValues>(parsePayload(payload));
  ASSERT_TRUE(packet);
  EXPECT_TRUE(packet->isLegacyLayout());
  EXPECT_EQ(packet->payloadSize(), 56u);
  EXPECT_DOUBLE_EQ(packet->temp_fet(), 48.0);
  EXPECT_DOUBLE_EQ(packet->temp_mos1(), 45.0);
  EXPECT_DOUBLE_EQ(packet->temp_mos2(), 46.0);
  EXPECT_DOUBLE_EQ(packet->temp_mos3(), 47.0);
  EXPECT_DOUBLE_EQ(packet->avg_motor_current(), 12.34);
  EXPECT_DOUBLE_EQ(packet->avg_input_current(), -2.5);
  EXPECT_DOUBLE_EQ(packet->duty_cycle_now(), 0.5);
  EXPECT_DOUBLE_EQ(packet->rpm(), 9000.0);
  EXPECT_DOUBLE_EQ(packet->v_in(), 12.0);
  EXPECT_DOUBLE_EQ(packet->amp_hours(), 1.2345);
  EXPECT_EQ(packet->tachometer(), -123);
  EXPECT_EQ(packet->tachometer_abs(), 456);
  EXPECT_EQ(packet->fault_code(), 2);
  EXPECT_DOUBLE_EQ(packet->avg_id(), 0.0);
  EXPECT_DOUBLE_EQ(packet->avg_iq(), 0.0);
}

TEST(VescFirmwareProtocol, ParsesModern5xValuesLayout)
{
  Buffer payload(73, 0);
  payload[0] = 4;
  putInt16(payload, 1, 500);
  putInt16(payload, 3, 400);
  putInt32(payload, 5, 1500);
  putInt32(payload, 9, 250);
  putInt32(payload, 13, 100);
  putInt32(payload, 17, 200);
  putInt16(payload, 21, 250);
  putInt32(payload, 23, 8000);
  putInt16(payload, 27, 118);
  putInt32(payload, 29, 20000);
  payload[53] = 3;
  putInt32(payload, 54, 500000);
  payload[58] = 9;
  putInt16(payload, 59, 510);
  putInt16(payload, 61, 520);
  putInt16(payload, 63, 530);
  putInt32(payload, 65, 1200);
  putInt32(payload, 69, -1400);

  auto packet = std::dynamic_pointer_cast<VescPacketValues>(parsePayload(payload));
  ASSERT_TRUE(packet);
  EXPECT_FALSE(packet->isLegacyLayout());
  EXPECT_DOUBLE_EQ(packet->temp_fet(), 50.0);
  EXPECT_DOUBLE_EQ(packet->temp_motor(), 40.0);
  EXPECT_DOUBLE_EQ(packet->avg_motor_current(), 15.0);
  EXPECT_DOUBLE_EQ(packet->avg_input_current(), 2.5);
  EXPECT_DOUBLE_EQ(packet->avg_id(), 1.0);
  EXPECT_DOUBLE_EQ(packet->avg_iq(), 2.0);
  EXPECT_DOUBLE_EQ(packet->duty_cycle_now(), 0.25);
  EXPECT_DOUBLE_EQ(packet->rpm(), 8000.0);
  EXPECT_DOUBLE_EQ(packet->v_in(), 11.8);
  EXPECT_DOUBLE_EQ(packet->amp_hours(), 2.0);
  EXPECT_EQ(packet->fault_code(), 3);
  EXPECT_DOUBLE_EQ(packet->pid_pos_now(), 0.5);
  EXPECT_EQ(packet->controller_id(), 9);
  EXPECT_DOUBLE_EQ(packet->temp_mos1(), 51.0);
  EXPECT_DOUBLE_EQ(packet->temp_mos2(), 52.0);
  EXPECT_DOUBLE_EQ(packet->temp_mos3(), 53.0);
  EXPECT_DOUBLE_EQ(packet->avg_vd(), 1.2);
  EXPECT_DOUBLE_EQ(packet->avg_vq(), -1.4);
}

TEST(VescFirmwareProtocol, UsesFirmwareSpecificServoCommandIds)
{
  const VescPacketSetServoPos legacy(0.5, true);
  const VescPacketSetServoPos modern(0.5, false);
  ASSERT_GT(legacy.frame().size(), 2u);
  ASSERT_GT(modern.frame().size(), 2u);
  EXPECT_EQ(legacy.frame()[2], 11);
  EXPECT_EQ(modern.frame()[2], 12);
}

}  // namespace
