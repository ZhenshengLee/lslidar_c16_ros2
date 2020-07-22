# Copyright 2020, Andreas Lebherz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Demonstration of Ouster Driver integration. """

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os

context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Ouster Driver integration.
    """
    ouster_demo_pkg_prefix = get_package_share_directory('ouster_full_pipe_demo')
    os1_top_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_top.param.yaml')
    os1_middle_left_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_middle_left.param.yaml')
    os1_middle_right_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_middle_right.param.yaml')
    voxel_grid_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_voxel_grid.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_ray_ground.param.yaml')
    euclidean_cluster_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_euclidean_cluster.param.yaml')
    pc_filter_transform_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/os1_point_cloud_transform.param.yaml')

    rviz_cfg_path = os.path.join(ouster_demo_pkg_prefix, 'config/ouster_demo.rviz')

    # Argumentsavp_
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    os1_top_param = DeclareLaunchArgument(
        'os1_top_param_file',
        default_value=os1_top_param_file,
        description='Path to config file for Ouster OS1 Top'
    )
    os1_middle_left_param = DeclareLaunchArgument(
        'os1_middle_left_param_file',
        default_value=os1_middle_left_param_file,
        description='Path to config file for Ouster OS1 Middle Left'
    )
    os1_middle_right_param = DeclareLaunchArgument(
        'os1_middle_right_param_file',
        default_value=os1_middle_right_param_file,
        description='Path to config file for Ouster OS1 Middle Right'
    )
    voxel_grid_param = DeclareLaunchArgument(
        'voxel_grid_param_file',
        default_value=voxel_grid_param_file,
        description='Path to config file for Voxel Grid Downsample'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )

    # Nodes
    static_transform = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        arguments=['1', '0', '1', '0', '0', '0', 'base_link', 'lidar_top']
    )
    filter_transform_lidar_top = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_lidar_top',
        node_namespace='lidar_top',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[('points_in', '/lidar_top/points')],
    )
    os1_top = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='lidar_top',
        parameters=[LaunchConfiguration('os1_top_param_file')]
    )
    os1_middle_left = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='middle_left',
        parameters=[LaunchConfiguration('os1_middle_left_param_file')]
    )
    os1_middle_right = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='lidar_middle_right',
        parameters=[LaunchConfiguration('os1_middle_right_param_file')]
    )
    voxel_grid = Node(
        package='voxel_grid_nodes',
        node_executable='voxel_grid_cloud_node_exe',
        parameters=[LaunchConfiguration('voxel_grid_param_file')],
        remappings=[('points_in', 'lidar_top/points')]
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_cloud_node_exe',
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[('points_in', '/points_downsampled')],
    )
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        node_executable='euclidean_cluster_exe',
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[('points_in', '/nonground_points')],
    )
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )
    return LaunchDescription([
        static_transform,
        pc_filter_transform_param,
        os1_top_param,
        # os1_middle_left_param,
        # os1_middle_right_param,
        voxel_grid_param,
        ray_ground_classifier_param,
        euclidean_cluster_param,
        with_rviz_param,
        filter_transform_lidar_top,
        os1_top,
        # os1_middle_left,
        # os1_middle_right,
        voxel_grid,
        ray_ground_classifier,
        euclidean_clustering,
        rviz2
    ])
