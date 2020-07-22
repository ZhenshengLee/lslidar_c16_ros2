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

#ifndef OUSTER_DRIVER__OS1_TRANSLATOR_HPP_
#define OUSTER_DRIVER__OS1_TRANSLATOR_HPP_

#include <ouster_driver/visibility_control.hpp>
#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include "geometry_msgs/msg/point32.hpp"
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
/// \brief Classes, types, and definitions specifically relating to Ouster LiDARs.
namespace ouster_driver
{
/// \brief Converts a little endian two byte representation into a 32bit unsigned integer
/// \param[in] lsb Least significant bit
/// \param[in] msb Most significant bit
/// \return The concatenated 32bit unsigned bit value
inline uint32_t le_to_uint32(const uint8_t lsb, const uint8_t msb)
{
  const uint32_t ret = static_cast<uint32_t>(msb) << 8U;
  return ret + static_cast<uint32_t>(lsb);
}
/// \brief Converts a little endian four byte representation into a 32bit unsigned integer
/// \param[in] first First byte of a little endian word
/// \param[in] second Second byte of a little endian word
/// \param[in] third Third byte of a little endian word
/// \param[in] fourth Fourth byte of a little endian word
/// \return The concatenated 32bit unsigned bit value
inline uint32_t le_to_uint32(
  const uint8_t first,
  const uint8_t second,
  const uint8_t third,
  const uint8_t fourth)
{
  uint32_t lsb = le_to_uint32(first, second);
  uint32_t msb = le_to_uint32(third, fourth);
  return lsb + (msb << 16);
}
/// \brief Transforms cartesian coordinates in geometry_msgs::msg::Point32
/// \param[in] x x coordinated of the point
/// \param[in] y y coordinated of the point
/// \param[in] z z coordinated of the point
/// \return The point with given coordinates
inline geometry_msgs::msg::Point32
make_point(const float32_t x, const float32_t y, const float32_t z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

/// \brief This class handles converting packets from a Ouster OS1 LiDAR into cartesian points
class OUSTER_DRIVER_PUBLIC OS1Translator
{
public:
  /// Tau = 2 * pi
  static constexpr float32_t TAU = 6.283185307179586476925286766559F;
  // SENSOR SPECIFIC CONSTANTS
  /// Resolution of the azimuth angle for a full turn
  static constexpr uint32_t AZIMUTH_ROTATION_RESOLUTION = 90112U;
  /// Resolution of the altitude angle for one firing
  static constexpr uint32_t ALTITUDE_RESOLUTION = 16U;
  /// Number of quantization steps for the intensity
  static constexpr uint32_t NUM_INTENSITY_VALUES = 65536U;
  /// Conversion of angle in radiant to index for lookup tables
  static constexpr float32_t RAD2IDX = static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION) / TAU;

  // All of these hardcoded values should remain fixed unless the OS1 packet spec changes
  /// Number of data blocks per data packet
  static constexpr uint16_t NUM_BLOCKS_PER_PACKET = 16U;
  /// Number of points stored in a data block
  static constexpr uint16_t NUM_POINTS_PER_BLOCK = 64U;
  /// Length of an array of the sensor response on "get_beam_intrinsics"
  static constexpr uint16_t LENGTH_BEAM_INTRINSICS = 64U;
  /// Length of a single entry of the sensor response on "get_beam_intrinsics"
  static constexpr uint16_t LENGTH_SINGLE_BEAM_INTRINSICS = 1U;
  /// Number of status bytes for single block
  static constexpr uint32_t AZIMUTH_STATUS_BYTES = 4U;
  /// Number of bytes of a single entry of the sensor response on "get_beam_intrinsics"
  static constexpr uint16_t LENGTH_FLOAT_PARAMETER = 5U;
  /// Minimal rotation rate in Hz
  static constexpr uint16_t MIN_RATE = 10U;
  /// Maximum rotation rate in Hz
  static constexpr uint16_t MAX_RATE = 20U;
  /// Maximum range in mm
  static constexpr uint32_t MAX_RANGE = 55000U;
  /// Point capacity of a single packet: 1 pkt has 16 blocks,
  /// each block has 128 points which results in 2048 points per pkt
  static constexpr uint16_t POINT_BLOCK_CAPACITY = 4096U;

  /// \brief Stores basic configuration information, does some simple validity checking
  class Config
  {
public:
    /// \brief Constructor
    /// \param[in] lidar_mode Describes azimuth resolution and rotational speed
    explicit Config(const std::string lidar_mode);

    /// \brief Gets lidar mode
    /// \return lidar_mode Configuration entry lidar_mode
    std::string get_lidar_mode() const;
    /// \brief Gets lidar rotation rate
    /// \return lidar_rate Rotational rate
    uint16_t get_lidar_rate() const;
    /// \brief Gets lidar resolution
    /// \return lidar_resolution Rotational resolution
    uint16_t get_lidar_resolution() const;

private:
    std::string m_lidar_mode;
    uint16_t m_lidar_rate;
    uint16_t m_lidar_resolution;

    /// \brief Sets lidar rate
    void set_lidar_rate();
    /// \brief Sets lidar resolution
    void set_lidar_resolution();
  };

  /// \brief Corresponds to an individual laser's firing
  struct DataChannel
  {
    uint8_t data[12U];
  };

  /// \brief Corresponds to a OS1 data block
  struct DataBlock
  {
    uint8_t timestamp[8U];
    uint8_t measurement_id[2U];
    uint8_t frame_id[2U];
    uint8_t encoder_count[4U];
    DataChannel channels[NUM_POINTS_PER_BLOCK];
    uint8_t azimuth_status[4U];
  };

  /// \brief Stores an Ouster data packet
  struct Packet
  {
    DataBlock blocks[NUM_BLOCKS_PER_PACKET];
  };

  /// \brief Default constructor
  /// \param[in] config Struct with set lidar_mode
  /// \throw std::runtime_error if pruning parameters are inconsistent
  explicit OS1Translator(const Config & config);

  /// \brief Convert a packet into a block of cartesian points
  /// \param[in] pkt A packet from a OS1 sensor for conversion
  /// \param[out] output Gets filled with cartesian points and any additional flags
  void convert(const Packet & pkt, std::vector<autoware::common::types::PointXYZIF> & output);

  /// \brief Initializes altitude and azimuth lookup tables
  void init_beam_intrinsics(std::string beam_intrinsics, std::string generation);

private:
  static_assert(sizeof(DataChannel) == 12U, "Error OS1 data channel size is incorrect");
  static_assert(sizeof(DataBlock) == 788U, "Error OS1 data block size is incorrect");
  // cannot assert packet size since it depends on sensor type
  // static_assert(sizeof(Packet) == 12608U, "Error OS1 packet size is incorrect");
  // Ensure that a full packet will fit into a point block
  static_assert(static_cast<uint32_t>(POINT_BLOCK_CAPACITY) >=
    ((NUM_POINTS_PER_BLOCK * NUM_BLOCKS_PER_PACKET) + 1U),
    "Number of points from one OS1 packet cannot fit into a point block");

  /// \brief Converts polar coordinates into cartesian (xyz), not threadsafe: modifies a
  ///        preallocated workspace member variable (m_point)
  /// \param[out] pt Gets filled with the requisite cartesian information
  /// \param[in] r_mm The radius in millimeters
  /// \param[in] th_ind The index of the azimuth in lookup tables (angle about the z-axis)
  /// \param[in] phi_ind The altitude angle index from lookup tables(angle orthogonal to z-axis)
  /// \param[in] th_enc_ind The index of the azimuth loopup table (encoder angle about the z-axis)
  /// \return none
  void polar_to_xyz(
    autoware::common::types::PointXYZIF & pt,
    const float32_t r_mm,
    const uint32_t th_ind,
    const uint32_t phi_ind,
    const uint32_t th_enc_ind) const
  {
    // conversion constant for mm to m
    float32_t mm_to_m = 0.001F;
    // distance to origin is saved as array with length 1
    float32_t origin_to_beam = m_origin_to_beam[0];
    pt.x = ((r_mm - origin_to_beam) * m_cos_table[phi_ind] * m_cos_table[th_ind] +
      origin_to_beam * m_cos_table[th_enc_ind]) * mm_to_m;
    pt.y = ((origin_to_beam - r_mm) * m_cos_table[phi_ind] * m_sin_table[th_ind] +
      origin_to_beam * m_sin_table[th_enc_ind]) * mm_to_m;
    pt.z = ((r_mm - origin_to_beam) * m_sin_table[phi_ind]) * mm_to_m;
  }

  /// \brief Converts the four byte representation of distance into meters
  /// \param[in] first First byte of a little endian word
  /// \param[in] second Second byte of a little endian word
  /// \param[in] third Third byte of a little endian word
  /// \param[in] fourth Fourth byte of a little endian word
  /// \return The radial distance in millimeters
  inline float32_t compute_distance_mm(
    const uint8_t first,
    const uint8_t second,
    const uint8_t third,
    const uint8_t fourth) const
  {
    const uint32_t dist_mm = le_to_uint32(first, second, third, fourth);
    return static_cast<float32_t>(std::min(MAX_RANGE, dist_mm));
  }

  /// \brief Clamps value with upper and lower bounding
  /// \param[in] val Value to clamp
  /// \param[in] min Upper bounding
  /// \param[in] max Lower bounding
  /// \return The clamped value
  template<typename T>
  inline T clamp(const T val, const T min, const T max)
  {
    return (val < min) ? min : ((val > max) ? max : val);
  }

  /// \brief Parses OS1 sensor response and extracts information of json formatted string
  /// \param[in] json_substring Substring of sensor response in json format
  /// \param[in] data_array Predefined placeholder for extracted data
  /// \param[in] data_size Length of data_array
  template<typename T>
  void parse_json_string(std::string json_substring, T data_array, size_t data_size);

  // initialization/precomputation functions
  /// \brief Runs all the table initialization functions in the right order
  OUSTER_DRIVER_LOCAL void init_tables();

  /// \brief Initializes sin and cosine lookup tables
  OUSTER_DRIVER_LOCAL void init_trig_tables();

  /// \brief Initializes intensity lookup table
  OUSTER_DRIVER_LOCAL void init_intensity_table();

  /// Lookup table for azimuth offset for each firing in a block
  float32_t m_azimuth_ind[NUM_POINTS_PER_BLOCK];
  /// Lookup table for altitude angle for each firing in a fire sequence (2 per block)
  float32_t m_altitude_ind[NUM_POINTS_PER_BLOCK];
  /// Offset of beam from origin of coordinate frame
  float32_t m_origin_to_beam[LENGTH_SINGLE_BEAM_INTRINSICS];
  /// Lookup table for sin calculations
  float32_t m_sin_table[AZIMUTH_ROTATION_RESOLUTION];
  /// Lookup table for cos calculations
  float32_t m_cos_table[AZIMUTH_ROTATION_RESOLUTION];
  /// Lookup table for intensity values
  float32_t m_intensity_table[NUM_INTENSITY_VALUES];
  /// Id for firing sequence
  uint16_t m_fire_id;
  /// Configurated radial resolution
  uint32_t m_lidar_resolution;
  /// Configurated radial rate
  uint32_t m_lidar_rate;
};  // class Driver

}  // namespace ouster_driver
}  // namespace drivers
}  // namespace autoware

#endif  // OUSTER_DRIVER__OS1_TRANSLATOR_HPP_
