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
#include <limits>
#include <utility>
#include <iostream>
#include <string>

#include "ouster_driver/os1_translator.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace drivers
{
namespace ouster_driver
{
////////////////////////////////////////////////////////////////////////////////
OS1Translator::Config::Config(const std::string lidar_mode)
: m_lidar_mode(lidar_mode)
{
  set_lidar_rate();
  set_lidar_resolution();
}
////////////////////////////////////////////////////////////////////////////////
std::string OS1Translator::Config::get_lidar_mode() const
{
  return m_lidar_mode;
}
////////////////////////////////////////////////////////////////////////////////
uint16_t OS1Translator::Config::get_lidar_rate() const
{
  return m_lidar_rate;
}
////////////////////////////////////////////////////////////////////////////////
uint16_t OS1Translator::Config::get_lidar_resolution() const
{
  return m_lidar_resolution;
}
////////////////////////////////////////////////////////////////////////////////
void OS1Translator::Config::set_lidar_resolution()
{
  uint16_t resolution;
  if (m_lidar_mode == "512x10" || m_lidar_mode == "512x20") {
    resolution = 512U;
  } else if (m_lidar_mode == "1024x10" || m_lidar_mode == "1024x20") {
    resolution = 1024U;
  } else if (m_lidar_mode == "2048x10") {
    resolution = 2048U;
  } else {
    std::cout << "Error: Could not determine lidar mode! default resolution: 512" << std::endl;
    resolution = 512U;
  }
  m_lidar_resolution = resolution;
}
////////////////////////////////////////////////////////////////////////////////
void OS1Translator::Config::set_lidar_rate()
{
  uint16_t rate;
  if (m_lidar_mode == "512x10" || m_lidar_mode == "1024x10" || m_lidar_mode == "2048x10") {
    rate = 10U;
  } else if (m_lidar_mode == "512x20" || m_lidar_mode == "1024x20") {
    rate = 20U;
  } else {
    std::cout << "Error: Could not determine lidar mode! default rate: 10" << std::endl;
    rate = 10U;
  }
  m_lidar_rate = rate;
}

}  // namespace ouster_driver
}  // namespace drivers
}  // namespace autoware
