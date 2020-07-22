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
#include <gtest/gtest.h>
#include <ouster_node/ouster_cloud_node.hpp>
#include <lidar_integration/lidar_integration.hpp>
#include <lidar_integration/udp_sender.hpp>
#include <memory>
#include <thread>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

// This test is just to make sure the alternative constructor doesn't die
TEST(ouster_node, constructor)
{
  rclcpp::init(0, nullptr);

  const auto name = "test_node";
  const auto ip = "127.0.0.1";
  const auto port = 9999U;
  const auto frame_id = "base_link";
  const auto cloud_size = 10000U;
  const auto sensor_id = 0U;
  using autoware::drivers::ouster_driver::OS1Translator;
  const auto config = OS1Translator::Config{"1024x10"};

  using autoware::drivers::ouster_node::OusterCloudNode;

  EXPECT_NO_THROW(
    OusterCloudNode(
      name,
      "ouster_test_topic_1",
      ip,
      port,
      frame_id,
      cloud_size,
      config)
  );
  // Fail because cloud size too small
  EXPECT_THROW(
    OusterCloudNode(
      name,
      "ouster_test_topic_2",
      ip,
      port,
      frame_id,
      500U,
      config),
    std::runtime_error
  );

  rclcpp::shutdown();
}

struct OusterNodeTestParam
{
  uint32_t reserved_size;
  uint32_t expected_size;
  float32_t expected_period_ms;
  bool8_t is_cloud;
};  // OusterNodeTestParam

class ouster_node_integration : public ::testing::TestWithParam<OusterNodeTestParam>
{
public:
protected:
};  // class ouster_node_integration

TEST_P(ouster_node_integration, DISABLED_test)
{
  rclcpp::init(0, nullptr);

  const auto param = GetParam();
  // Configuration
  const auto cloud_size = param.reserved_size;
  const auto name = "test_node";
  const auto ip = "127.0.0.1";
  const auto port = 3555U;
  const auto frame_id = "base_link";
  const auto sensor_id = 0U;
  const auto runtime = std::chrono::seconds(10);
  std::string topic = "ouster_test_topic_cloud";
  using autoware::drivers::ouster_driver::OS1Translator;
  const auto config = OS1Translator::Config{"1024x10"};

  // Node
  using autoware::drivers::ouster_node::OusterCloudNode;
  std::shared_ptr<OusterCloudNode> nd_ptr = std::make_shared<OusterCloudNode>(
    name,
    topic,
    ip,
    port,
    frame_id,
    cloud_size,
    config);
  std::thread ouster_node_thread;

  // Listener
  using lidar_integration::LidarIntegrationListener;
  std::shared_ptr<LidarIntegrationListener> listen_ptr;
  using lidar_integration::LidarIntegrationPclListener;
  listen_ptr = std::make_shared<LidarIntegrationPclListener>(
    topic,
    param.expected_period_ms,
    param.expected_size,
    0.7,  // period tolerance
    0.1F  // size tolerance
  );

  rclcpp::shutdown();
}
