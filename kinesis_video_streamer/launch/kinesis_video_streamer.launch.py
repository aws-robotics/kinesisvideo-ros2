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
import yaml
import shutil

# Argument names
NODE_NAME = "node_name"
CONFIG = "config"
MAX_STRING_LEN = 128

def get_logger_config_path():
  logger_config_path = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'kvs_log_configuration')
  default_config = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'sample_config.yaml')

  with open(default_config, 'r') as f:
      config_text = f.read()
  config_yaml = yaml.safe_load(config_text)

  fallback_path = '/tmp/kvs_log_configuration'
  if fallback_path == config_yaml['kinesis_video_streamer']['ros__parameters']['kinesis_video']['log4cplus_config']:
    if len(logger_config_path) <= MAX_STRING_LEN:
      # Override /tmp/... to the actual path
      return logger_config_path
    else:
      # Path too long and no alternative was specified - copy to /tmp/ fallback path (platform-dependent)
      shutil.copy(logger_config_path, fallback_path)
      return fallback_path
  else:
    # The config file was modified by the user - do not override
    return None

def generate_launch_description():
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

  logger_config_path = get_logger_config_path()
  if logger_config_path is None:
    node_parameters = [launch.substitutions.LaunchConfiguration(CONFIG)]
  else:
    node_parameters = [launch.substitutions.LaunchConfiguration(CONFIG), {'log4cplus_config': logger_config_path}]
  streamer_node = launch_ros.actions.Node(
    package="kinesis_video_streamer",
    node_executable="kinesis_video_streamer",
    node_name=launch.substitutions.LaunchConfiguration(NODE_NAME),
    parameters=node_parameters
  )

  ld.add_action(streamer_node)

  return ld


if __name__ == "__main__":
  generate_launch_description()
