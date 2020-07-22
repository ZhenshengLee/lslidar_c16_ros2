// Copyright 2020, Andreas Lebherz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_DRIVER_H_
#define TEST_DRIVER_H_

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

#include "common/types.hpp"
#include "ouster_driver/os1_translator.hpp"
#include "gtest/gtest.h"

#include "os1_packet.h"

/// Test little Endian uint16_t conversion
TEST(helpers, uint32)
{
  const uint8_t first = 0x12;
  const uint8_t second = 0x34;
  const uint8_t third = 0x56;
  const uint8_t fourth = 0x00;

  EXPECT_EQ(autoware::drivers::ouster_driver::le_to_uint32(first, second), 13330U);
  EXPECT_EQ(autoware::drivers::ouster_driver::le_to_uint32(first, second, third, fourth), 5649426U);
}

using autoware::common::types::float32_t;
using autoware::drivers::ouster_driver::OS1Translator;
using autoware::drivers::ouster_driver::make_point;

class ouster_driver : public::testing::Test
{
public:
  ouster_driver()
  {
    (void)memcpy(&pkt, static_packet, sizeof(pkt));
    out.reserve(OS1Translator::POINT_BLOCK_CAPACITY);
  }

protected:
  OS1Translator::Packet pkt;
  std::vector<autoware::common::types::PointXYZIF> out;
};  // class ouster_driver

// for testing
static constexpr uint16_t NUM_BYTES_PER_PIXEL = sizeof(OS1Translator::DataChannel);
static constexpr uint16_t NUM_HEADER_BYTES = 16U;
static constexpr uint16_t NUM_TRAILER_BYTES = 4U;

TEST_F(ouster_driver, basic)
{

  const geometry_msgs::msg::Point32 offset_m;
  const geometry_msgs::msg::Point32 rotation_rad;
  const OS1Translator::Config cfg{"1024x10"};
  OS1Translator driver(cfg);
  driver.convert(pkt, out);
  EXPECT_LE(out.size(),
  NUM_HEADER_BYTES +
  OS1Translator::NUM_POINTS_PER_BLOCK * OS1Translator::NUM_BLOCKS_PER_PACKET * NUM_BYTES_PER_PIXEL +
  NUM_TRAILER_BYTES);

  // Mostly just a sanity check: All points should fall in a pie slice
  float32_t min_r = std::numeric_limits<float32_t>::max();
  float32_t max_r = 0.0F;
  float32_t min_th = std::numeric_limits<float32_t>::max();
  float32_t max_th = -std::numeric_limits<float32_t>::max();
  float32_t min_phi = std::numeric_limits<float32_t>::max();
  float32_t max_phi = -std::numeric_limits<float32_t>::max();
  uint32_t last_id = 0U;
  for (uint32_t idx = 0U; idx < out.size(); ++idx) {
    autoware::common::types::PointXYZIF & pt = out[idx];
    if (0U == idx) {
      last_id = pt.id;
    }
    const float32_t th = atan2f(pt.y, pt.x);
    const float32_t r_xy = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    const float32_t phi = atan2f(pt.z, r_xy);
    const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
   // Update min/max
    min_r = std::min(min_r, r);
    max_r = std::max(max_r, r);
    if (fabsf(th) > 0.00000001F) {
      // missing returns are (0, 0)
      min_th = std::min(min_th, th);
      max_th = std::max(max_th, th);
    }
    min_phi = std::min(min_phi, phi);
    max_phi = std::max(max_phi, phi);
    // Sanity check on intensity
    EXPECT_LE(pt.intensity, 255.0F);
    EXPECT_GE(pt.intensity, 0.0F);
    // IDs are contiguous
    EXPECT_TRUE((pt.id == last_id) || (pt.id == (last_id + 1U)));
    last_id = pt.id;
  }
  // values from hardware spec OS1-0; 80% reflectivity = 55.0m
  EXPECT_GE(min_r, 0.0F);
  EXPECT_LE(max_r, 55.1F) << max_r;
  // compute angle differences
  const float32_t dth = max_th - min_th;
  const float32_t th_diff = fabsf(atan2f(sinf(dth), cosf(dth)));
  // max difference in elevation angles should be no more than 20 degrees
  const float32_t dphi = max_phi - min_phi;
  const float32_t phi_diff = fabsf(atan2f(sinf(dphi), cosf(dphi)));
  EXPECT_LE(phi_diff, (20.0F * 3.14159F / 180.0F) + 0.001F);
}

// figure out what the runtime of convert() is, locally
TEST_F(ouster_driver, benchmark)
{
  const OS1Translator::Config cfg{"1024x10"};
  autoware::drivers::ouster_driver::OS1Translator driver(cfg);
  const uint32_t num_runs = 200U;
  auto time_begin = std::chrono::steady_clock::now();
  for (uint32_t idx = 0U; idx < num_runs; ++idx) {
    driver.convert(pkt, out);
  }
  auto time_end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin);
  std::cerr << "convert() average runtime: " << duration.count() / num_runs << " µs\n";
}
#endif  // TEST_DRIVER_H_
