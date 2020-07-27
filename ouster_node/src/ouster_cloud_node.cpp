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

#include <string>
#include <chrono>
#include <vector>
#include <iostream>
#include <memory>

#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "ouster_node/ouster_cloud_node.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace drivers
{
namespace ouster_node
{
OusterCloudNode::OusterCloudNode(
  const std::string & node_name,
  const std::string & topic,
  const std::string & ip,
  const uint16_t port,
  const std::string & frame_id,
  const std::size_t cloud_size,
  const ouster_driver::OS1Translator::Config & config)
: UdpDriverNode<ouster_driver::OS1Translator::Packet, sensor_msgs::msg::PointCloud2>(
    node_name,
    topic,
    UdpConfig{ip, port}),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(frame_id),
  m_cloud_size(cloud_size),
  m_translator(config)
{
  m_point_block.reserve(autoware::drivers::ouster_driver::OS1Translator::POINT_BLOCK_CAPACITY);
  // If your preallocated cloud size is too small, the node really won't operate well at all
  if (static_cast<uint32_t>(m_point_block.capacity()) >= cloud_size) {
    throw std::runtime_error("OusterCloudNode: cloud_size must be > PointBlock::CAPACITY");
  }
  // check if default configuration should be used
  bool8_t default_config = static_cast<bool8_t>(
    declare_parameter("default_config").get<bool8_t>());

  if(!default_config) {
    // do TCP initialization of client
    init_client();
  } else {
    init_default_client();
  }

  std::string data_format = get_parameter("lidar_data_format").as_string();
  m_translator.init_data_format(data_format);

  // set beam_intrinsics in translator
  std::string beam = get_parameter("beam_intrinsics").as_string();
  std::string generation = get_parameter("generation").as_string();
  m_translator.init_beam_intrinsics(beam, generation);
}

OusterCloudNode::OusterCloudNode(
  const std::string & node_name,
  const std::string & node_namespace)
: UdpDriverNode(node_name, node_namespace),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(declare_parameter("frame_id").get<std::string>().c_str()),
  m_cloud_size(static_cast<std::size_t>(declare_parameter("cloud_size").get<std::size_t>())),
  m_generation(static_cast<std::string>(declare_parameter("generation").get<std::string>())),
  m_translator(ouster_driver::OS1Translator::Config{
        static_cast<std::string>(declare_parameter("lidar_mode").get<std::string>())})
{
  m_point_block.reserve(autoware::drivers::ouster_driver::OS1Translator::POINT_BLOCK_CAPACITY);
  // if your preallocated cloud size is too small, the node really won't operate well at all
  if (static_cast<uint32_t>(m_point_block.capacity()) >= m_cloud_size) {
    throw std::runtime_error("OusterCloudNode: cloud_size must be > PointBlock::CAPACITY");
  }
  // check if default configuration should be used
  bool8_t default_config = static_cast<bool8_t>(
    declare_parameter("default_config").get<bool8_t>());
  if(!default_config) {
    // do TCP initialization of client
    init_client();
  } else {
    init_default_client();
  }

  std::string data_format = get_parameter("lidar_data_format").as_string();
  m_translator.init_data_format(data_format);

  // pass beam_intrinsics and sensor-generation to translator
  std::string beam = get_parameter("beam_intrinsics").as_string();
  std::string generation = get_parameter("generation").as_string();
  m_translator.init_beam_intrinsics(beam, generation);
}

////////////////////////////////////////////////////////////////////////////////
void OusterCloudNode::init_output(sensor_msgs::msg::PointCloud2 & output)
{
  autoware::common::lidar_utils::init_pcl_msg(output, m_frame_id.c_str(), m_cloud_size);
  m_point_cloud_its.reset(output, m_point_cloud_idx);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t OusterCloudNode::do_tcp_cmd(
  int sock_fd,
  const std::vector<std::string> & cmd_tokens,
  std::string & res)
{
  const size_t max_res_len = 16 * 1024;
  auto read_buf = std::unique_ptr<char[]>{new char[max_res_len + 1]};

  std::stringstream ss;
  for (const auto & token : cmd_tokens) {
    ss << token << " ";
  }
  ss << "\n";
  std::string cmd = ss.str();

  ssize_t len = write(sock_fd, cmd.c_str(), cmd.length());
  if (len != static_cast<ssize_t>(cmd.length())) {
    std::cout << "len != cmd.length " << len << " != " <<
      static_cast<ssize_t>(cmd.length()) << std::endl;
    return false;
  }

  // need to synchronize with server by reading response
  std::stringstream read_ss;
  do {
    len = read(sock_fd, read_buf.get(), max_res_len);
    if (len < 0) {
      std::cout << "len < 0" << std::endl;
      return false;
    }
    read_buf.get()[len] = '\0';
    read_ss << read_buf.get();
  } while (len > 0 && read_buf.get()[len - 1] != '\n');

  res = read_ss.str();
  res.erase(res.find_last_not_of(" \r\n\t") + 1);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
uint16_t OusterCloudNode::cfg_socket(const char * addr, const char * udp_port)
{
  struct addrinfo hints, * info_start, * ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  int ret = getaddrinfo(addr, udp_port, &hints, &info_start);

  if (ret != 0) {
    std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
    return -1;
  }
  if (info_start == NULL) {
    std::cerr << "getaddrinfo: empty result" << std::endl;
    return -1;
  }
  int sock_fd;
  for (ai = info_start; ai != NULL; ai = ai->ai_next) {
    sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock_fd < 0) {
      std::cerr << "socket: " << std::strerror(errno) << std::endl;
      continue;
    }
    if (connect(sock_fd, ai->ai_addr, ai->ai_addrlen) == -1) {
      close(sock_fd);
      continue;
    }
    break;
  }

  freeaddrinfo(info_start);
  if (ai == NULL) {
    return -1;
  }
  return sock_fd;
}

////////////////////////////////////////////////////////////////////////////////
void OusterCloudNode::init_client()
{
  std::cout << "Configuration via TCP startet." << std::endl;

  std::string udp_sensor_ip = static_cast<std::string>(
    declare_parameter("udp_sensor_ip").get<std::string>());
  std::string udp_computer_ip = static_cast<std::string>(
    declare_parameter("udp_computer_ip").get<std::string>());
  std::string udp_port_lidar = static_cast<std::string>(
    declare_parameter("udp_port_lidar").get<std::string>());
  std::string udp_port_imu = static_cast<std::string>(
    declare_parameter("udp_port_imu").get<std::string>());

  const char * udp_config_port = "7501";
  uint16_t sock_fd = cfg_socket(udp_sensor_ip.c_str(), udp_config_port);

  bool success = true;
  std::string res;

  success &= do_tcp_cmd(sock_fd, {"set_config_param", "udp_ip",
        udp_computer_ip}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(sock_fd, {"set_config_param", "udp_port_lidar",
        udp_port_lidar}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(sock_fd, {"set_config_param", "udp_port_imu",
        udp_port_imu}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(
    sock_fd, {"set_config_param", "lidar_mode",
      get_parameter("lidar_mode").as_string()}, res);
  success &= res == "set_config_param";

  // get necessary data from sensor (only beam_intrinsics and data_format used)
  success &= do_tcp_cmd(sock_fd, {"get_beam_intrinsics"}, res);
  declare_parameter("beam_intrinsics", res);

  success &= do_tcp_cmd(sock_fd, {"get_imu_intrinsics"}, res);
  declare_parameter("imu_intrinsics", res);

  success &= do_tcp_cmd(sock_fd, {"get_lidar_intrinsics"}, res);
  declare_parameter("lidar_instrinsics", res);

  success &= do_tcp_cmd(sock_fd, {"get_lidar_data_format​"}, res);
  declare_parameter("lidar_data_format", res);

  success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
  success &= res == "reinitialize";

  if(success) {
    std::cout << "Configuration finished successful." << std::endl;
  }

  close(sock_fd);
}

////////////////////////////////////////////////////////////////////////////////
void OusterCloudNode::init_default_client()
{
  std::cout << "IN DEFAULT CONFIG!" << std::endl;
  // Values from parameter file
  declare_parameter("udp_sensor_ip");
  declare_parameter("udp_computer_ip");
  declare_parameter("udp_port_lidar");
  declare_parameter("udp_port_imu");

  // beam_intrinsics standard config
  std::string beam_intrinsics = "{\"beam_altitude_angles\": [45.82, 44.11, 42.85, 41.17, 39.93, 38.24, 36.99, 35.32, 34.06, 32.42, 31.15, 29.54, 28.26, 26.67, 25.38, 23.81, 22.51, 20.97, 19.66, 18.15, 16.82, 15.33, 13.99, 12.53, 11.18, 9.73, 8.369999999999999, 6.94, 5.56, 4.16, 2.77, 1.37, -0.04, -1.41, -2.84, -4.19, -5.64, -6.99, -8.449999999999999, -9.789999999999999, -11.28, -12.59, -14.11, -15.41, -16.95, -18.25, -19.8, -21.08, -22.67, -23.94, -25.55, -26.81, -28.44, -29.7, -31.35, -32.6, -34.28, -35.53, -37.24, -38.48, -40.22, -41.46, -43.23, -44.47], \"beam_azimuth_angles\": [10.85, -3.53, 10.46, -3.4, 10.11, -3.28, 9.81, -3.19, 9.56, -3.11, 9.33, -3.04, 9.130000000000001, -2.97, 8.960000000000001, -2.92, 8.82, -2.87, 8.69, -2.84, 8.59, -2.81, 8.51, -2.79, 8.44, -2.77, 8.4, -2.76, 8.359999999999999, -2.75, 8.33, -2.75, 8.32, -2.76, 8.34, -2.76, 8.359999999999999, -2.78, 8.390000000000001, -2.8, 8.44, -2.82, 8.51, -2.86, 8.59, -2.89, 8.69, -2.93, 8.81, -2.99, 8.94, -3.04, 9.1, -3.11, 9.300000000000001, -3.19, 9.52, -3.28, 9.779999999999999, -3.39, 10.09, -3.51, 10.45, -3.66], \"lidar_origin_to_beam_origin_mm\": 27.67}";
  declare_parameter("beam_intrinsics", beam_intrinsics);

  std::cout << "Default configuration loaded: OS2-64" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t OusterCloudNode::convert(
  const ouster_driver::OS1Translator::Packet & pkt,
  sensor_msgs::msg::PointCloud2 & output)
{
  // This handles the case when the below loop exited due to containing extra points
  if (m_published_cloud) {
    // reset the pointcloud
    autoware::common::lidar_utils::reset_pcl_msg(output, m_cloud_size, m_point_cloud_idx);
    m_point_cloud_its.reset(output, m_point_cloud_idx);

    // deserialize remainder into pointcloud
    m_published_cloud = false;

    for (uint32_t idx = m_remainder_start_idx; idx < m_point_block.size(); ++idx) {
      const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
      (void)add_point_to_cloud(m_point_cloud_its, pt, m_point_cloud_idx);
      // Here I am ignoring the return value, because this operation should never fail.
      // In the constructor I ensure that cloud_size > PointBlock::CAPACITY. This means
      // I am guaranteed to fit at least one whole PointBlock into my PointCloud2.
      // Because just above this for loop, I reset the capacity of the pcl message,
      // I am guaranteed to have capacity for the remainder of a point block.
    }
  }
  m_translator.convert(pkt, m_point_block);

  for (uint32_t idx = 0U; idx < m_point_block.size(); ++idx) {
    const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
    if (static_cast<uint16_t>(autoware::common::types::PointXYZIF::END_OF_SCAN_ID) != pt.id) {
      if (!add_point_to_cloud(m_point_cloud_its, pt, m_point_cloud_idx)) {
        m_published_cloud = true;
        m_remainder_start_idx = idx;
      }
    } else {
      m_published_cloud = true;
      m_remainder_start_idx = idx;
      break;
    }
  }
  if (m_published_cloud) {
    // resize pointcloud down to its actual size
    autoware::common::lidar_utils::resize_pcl_msg(output, m_point_cloud_idx);
    output.header.stamp = this->now();
    m_point_cloud_its.reset(output, m_point_cloud_idx);
  }
  return m_published_cloud;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t OusterCloudNode::get_output_remainder(sensor_msgs::msg::PointCloud2 & output)
{
  (void)output;
  return false;
}

}  // namespace ouster_node
}  // namespace drivers
}  // namespace autoware
