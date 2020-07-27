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

#include <common/types.hpp>
#include <cstring>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <string>
#include "ouster_driver/os1_translator.hpp"

#include <iostream>

namespace autoware
{
namespace drivers
{
namespace ouster_driver
{

using autoware::common::types::float32_t;
using autoware::common::types::PointXYZIF;

////////////////////////////////////////////////////////////////////////////////
OS1Translator::OS1Translator(const Config & config)
: m_fire_id(0U)
{
  // technically don't need these, but pclint would yell at you
  (void)memset(static_cast<void *>(m_altitude_ind), 0, sizeof(m_altitude_ind));
  (void)memset(static_cast<void *>(m_azimuth_ind), 0, sizeof(m_azimuth_ind));
  (void)memset(static_cast<void *>(m_origin_to_beam), 0, sizeof(m_origin_to_beam));
  // do precomputation
  init_tables();

  m_lidar_resolution = config.get_lidar_resolution();
}

void OS1Translator::init_tables()
{
  init_trig_tables();
  init_intensity_table();
}

////////////////////////////////////////////////////////////////////////////////
void OS1Translator::convert(const Packet & pkt, std::vector<PointXYZIF> & output)
{
  output.clear();
  for (uint16_t idx = 0U; idx < NUM_BLOCKS_PER_PACKET; ++idx) {
    const DataBlock & block = pkt.blocks[idx];
    // packet header: status_ok = 0xffffffff; status_error = 0x00
    if ((block.azimuth_status[0U] == static_cast<uint8_t>(0xFF)) &&
      (block.azimuth_status[1U] == static_cast<uint8_t>(0xFF)) &&
      (block.azimuth_status[2U] == static_cast<uint8_t>(0xFF)) &&
      (block.azimuth_status[3U] == static_cast<uint8_t>(0xFF)))
    {
      uint32_t encoder = le_to_uint32(
        block.encoder_count[0],
        block.encoder_count[1],
        block.encoder_count[2],
        block.encoder_count[3]);
      // get distance/intensity for each point in block, convert to cartesian
      uint16_t size_column = static_cast<uint16_t>(m_pixels_per_column[0]);

      for (uint16_t jdx = 0U; jdx < size_column; ++jdx) {
        float32_t azimuth = m_azimuth_ind[jdx];
        float32_t altitude = m_altitude_ind[jdx];

        const DataChannel & channel = block.channels[jdx];

        const float32_t r = compute_distance_mm(
          channel.data[0U],
          channel.data[1U],
          channel.data[2U],
          channel.data[3U]);

        // new condition introduced to avoid weird behaviour when points are
        // reflected from the roof of the vehicle
        if(r == MAX_RANGE) {
          break;
        }

        // theta, phi in radiant
        const float32_t phi = (TAU / 360.0f) * altitude;
        const float32_t th_int = TAU * (azimuth / 360.0f);
        const float32_t th_enc = TAU * (static_cast<float32_t>(encoder) /
          AZIMUTH_ROTATION_RESOLUTION);
        const float32_t th = th_enc + th_int;

        // theta, phi as indices in sin/cos tables have to be in range 0:TAU
        const uint32_t th_ind = static_cast<uint32_t>((th + static_cast<float32_t>(TAU)) *
          RAD2IDX) % AZIMUTH_ROTATION_RESOLUTION;
        const uint32_t th_enc_ind = static_cast<uint32_t>((th_enc + static_cast<float32_t>(TAU)) *
          RAD2IDX) % AZIMUTH_ROTATION_RESOLUTION;
        const uint32_t phi_ind = static_cast<uint32_t>((phi + static_cast<float32_t>(TAU)) *
          RAD2IDX) % AZIMUTH_ROTATION_RESOLUTION;

        PointXYZIF pt;
        // calculates and sets x/y/z coordinates of pt in m
        polar_to_xyz(pt, r, th_ind, phi_ind, th_enc_ind);

        // intensity values are 2 Bytes
        pt.intensity = m_intensity_table[le_to_uint32(channel.data[4], channel.data[5])];
        pt.id = m_fire_id;
        output.push_back(pt);
      }
      // prevent overflow
      m_fire_id = static_cast<uint16_t>(m_fire_id + 1U);
      // check fire_id against count, push end of scan message
      if (m_fire_id >= m_lidar_resolution) {
        PointXYZIF pt;
        pt.id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
        output.push_back(pt);

        m_fire_id = static_cast<uint16_t>(0U);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
void OS1Translator::parse_json_string(std::string json_substring, T data_array, size_t data_size)
{
  // single value
  if (data_size == 1) {
    std::stringstream iss(json_substring);

    std::string::size_type dot = json_substring.find(":");
    json_substring = json_substring.substr(dot + 1, LENGTH_FLOAT_PARAMETER + 1);
    *data_array = stof(json_substring, 0);
    return;
  }
  // multiple values
  // Remove all occurences of "," to ease parsing
  std::string del = ",";
  std::string::size_type n = del.length();
  for (std::string::size_type i = json_substring.find(del);
    i != std::string::npos;
    i = json_substring.find(del))
  {
    json_substring.erase(i, n);
  }

  std::string::size_type ob = json_substring.find("[");
  std::string::size_type cb = json_substring.find("]");

  if (ob != std::string::npos && cb != std::string::npos) {
    // get everything between brackets "[", "]"
    json_substring = json_substring.substr(ob + 1, cb - 1);

    std::stringstream iss(json_substring);

    // extract float values
    auto number = *data_array;
    size_t i = 0;
    // avoid overflow
    while (iss >> number && i < data_size) {
      data_array[i] = number;
      ++i;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void OS1Translator::init_beam_intrinsics(std::string beam_intrinsics, std::string generation)
{
  std::string origin_sub_str = "";
  std::string alt_sub_str = beam_intrinsics.substr(
    beam_intrinsics.find("beam_altitude_angles"), beam_intrinsics.size());
  std::string azi_sub_str = beam_intrinsics.substr(
    beam_intrinsics.find("beam_azimuth_angles"), beam_intrinsics.size());

  uint16_t size_tables = static_cast<uint16_t>(m_pixels_per_column[0]);

  memset(m_azimuth_ind, 0.0, sizeof(m_azimuth_ind));
  memset(m_altitude_ind, 0.0, sizeof(m_altitude_ind));

  // pixels_per_column is polled value from sensor
  parse_json_string(azi_sub_str, m_azimuth_ind, size_tables);
  parse_json_string(alt_sub_str, m_altitude_ind, size_tables);

  if (generation == "Gen2") {
    origin_sub_str = beam_intrinsics.substr(
      beam_intrinsics.find("lidar_origin_to_beam_origin_mm"), beam_intrinsics.size());
    parse_json_string(origin_sub_str, m_origin_to_beam, LENGTH_SINGLE_ENTRY);
  } else {
    // in case of older generation ignore offset
    m_origin_to_beam[0] = 0.0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void OS1Translator::init_data_format(std::string data_format)
{
  std::string origin_sub_str = "";
  std::string pixels_per_column_sub_str = data_format.substr(
    data_format.find("pixels_per_column"), data_format.size());

  parse_json_string(pixels_per_column_sub_str, m_pixels_per_column, LENGTH_SINGLE_ENTRY);
}

////////////////////////////////////////////////////////////////////////////////
void OS1Translator::init_trig_tables()
{
  const float32_t IDX2RAD = TAU / static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION);
  for (uint64_t idx = 0U; idx < AZIMUTH_ROTATION_RESOLUTION; ++idx) {
    // angles range from -Pi to Pi
    m_cos_table[idx] = cosf(static_cast<float32_t>(idx) * IDX2RAD);
    m_sin_table[idx] = sinf((static_cast<float32_t>(idx)) * IDX2RAD);
  }
}

////////////////////////////////////////////////////////////////////////////////
void OS1Translator::init_intensity_table()
{
  for (uint64_t idx = 0U; idx < NUM_INTENSITY_VALUES; ++idx) {
    m_intensity_table[idx] = static_cast<float32_t>(idx);
  }
}

}  // namespace ouster_driver
}  // namespace drivers
}  // namespace autoware
