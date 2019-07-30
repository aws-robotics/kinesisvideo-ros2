# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import re

# Argument names
NODE_NAME = "node_name"
CONFIG = "config"

def update_logger_config_path():
  logger_config_path = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'kvs_log_configuration')
  default_config = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'sample_config.yaml')
  with open(default_config, 'r') as f:
      config_text = f.read()
  updated_config_text = re.sub('log4cplus_config:(.*)', 'log4cplus_config: ' + logger_config_path, config_text)
  with open(default_config, 'w') as f:
      f.write(updated_config_text)

def generate_launch_description():
  update_logger_config_path()
  # Default to included config file
  default_config = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'sample_config.yaml')

  ld = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      NODE_NAME,
      default_value="kinesis_video_streamer",
    ),
    launch.actions.DeclareLaunchArgument(
      CONFIG,
      default_value=default_config
    )
   ])
  encoder_node = launch_ros.actions.Node(
    package="kinesis_video_streamer",
    node_executable="kinesis_video_streamer",
    node_name=launch.substitutions.LaunchConfiguration(NODE_NAME),
    parameters=[launch.substitutions.LaunchConfiguration(CONFIG)]
  )

  ld.add_action(encoder_node)

  return ld


if __name__ == "__main__":
  generate_launch_description()
