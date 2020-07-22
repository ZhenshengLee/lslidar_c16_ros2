// Copyright 2020, Andreas Lebherz
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

#ifndef OUSTER_NODE__OUSTER_CLOUD_NODE_HPP_
#define OUSTER_NODE__OUSTER_CLOUD_NODE_HPP_

#include <string>
#include <vector>
#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "udp_driver/udp_driver_node.hpp"
#include "ouster_driver/os1_translator.hpp"
#include "ouster_node/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `ouster_driver`
namespace ouster_node
{
class OUSTER_NODE_PUBLIC OusterCloudNode
  : public udp_driver::UdpDriverNode<
    ouster_driver::OS1Translator::Packet,
    sensor_msgs::msg::PointCloud2>
{
public:
  /// \brief Default constructor, starts driver
  /// \param[in] node_name Name of the node for rclcpp internals
  /// \param[in] topic Name of the topic to publish output on
  /// \param[in] ip Expected IP of UDP packets
  /// \param[in] port Port that this driver listens to (i.e. sensor device at ip writes to port)
  /// \param[in] frame_id Frame id for the published point cloud messages
  /// \param[in] cloud_size Preallocated capacity (in number of points) for the point cloud messages
  ///                       must be greater than PointBlock::CAPACITY
  /// \param[in] config Config struct with lidar_mode parameter
  /// \throw std::runtime_error If cloud_size is not sufficiently large
  OusterCloudNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & ip,
    const uint16_t port,
    const std::string & frame_id,
    const std::size_t cloud_size,
    const ouster_driver::OS1Translator::Config & config);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] node_namespace Namespace for this node
  OusterCloudNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

protected:
  // Override of virtual functions
  void init_output(sensor_msgs::msg::PointCloud2 & output) override;
  bool8_t convert(
    const ouster_driver::OS1Translator::Packet & pkt,
    sensor_msgs::msg::PointCloud2 & output) override;
  bool8_t get_output_remainder(sensor_msgs::msg::PointCloud2 & output) override;

private:
  /// \brief Initializer of the ouster client via tcp commands
  /// Opens a socket and sends tcp commands with parameters from yaml file and sensor response
  /// closes socket after initialization was successful
  void init_client();
  /// \brief Defualt initializer of the ouster client via tcp commands
  /// Uses values from parameter file and standard sensor configuration in order to use
  /// this driver without necessary needing a connected sensor
  void init_default_client();
  /// \brief TCP command sender
  /// \param[in] sock_fd TCP socket fd
  /// \param[in] cmd_tokens Command to send via TCP
  /// \param[in] res Returns the answer of the client sensor
  /// \param[out] bool States correctness of transmission
  bool8_t do_tcp_cmd(int sock_fd, const std::vector<std::string> & cmd_tokens, std::string & res);
  /// \brief Socket configurator for TCP traffic
  /// \param[in] addr Ipv4 address for TCP communicaiton
  /// \param[in] udp_port Port for TCP communicatoin
  /// \param[out] Opened socket fd
  /// \throw std::strerror if opening socket failed
  uint16_t cfg_socket(const char * addr, const char * udp_port);

  // These next two variables are a minor hack to maintain stateful information across convert()
  // calls. Specifically, it signals to reset any stateful information on the data vector at the
  // top of the convert function
  /// Cloud publish status
  bool8_t m_published_cloud;
  // Keeps track of where you left off on the converted point block in case you needed to publish
  // a point cloud in the middle of processing it
  /// Start index of remainder in case message does not fit in cloud
  uint32_t m_remainder_start_idx;
  /// keeps track of the constructed point cloud to continue growing it with new data
  uint32_t m_point_cloud_idx;
  /// Lidar pointcloud
  autoware::common::lidar_utils::PointCloudIts m_point_cloud_its;
  /// Current id
  const std::string m_frame_id;
  /// Cloud size
  const std::size_t m_cloud_size;
  /// Sensor generation e.g. OS1-64
  const std::string m_generation;
  /// Instance of translator to pass and parameter from yaml file
  ouster_driver::OS1Translator m_translator;
  /// Set of points to convert
  std::vector<autoware::common::types::PointXYZIF> m_point_block;
};  // class OusterCloudNode

}  // namespace ouster_node
}  // namespace drivers
}  // namespace autoware

#endif  // OUSTER_NODE__OUSTER_CLOUD_NODE_HPP_
