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
AWS_REGION = "aws_region"
STREAM_NAME = "stream_name"
REKOGNITION_DATA_STREAM = "rekognition_data_stream"

MAX_STRING_LEN = 128

def get_logger_config_path(config_yaml):
  logger_config_path = os.path.join(get_package_share_directory('kinesis_video_streamer'),
    'config', 'kvs_log_configuration')
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
  default_config = os.path.join(get_package_share_directory('kinesis_video_streamer'), 'config', 'sample_config.yaml')
  with open(default_config, 'r') as f:
    default_config_text = f.read()
  default_config_yaml = yaml.safe_load(default_config_text)

  default_aws_region = default_config_yaml['kinesis_video_streamer']['ros__parameters']['aws_client_configuration']['region']
  default_stream_name = default_config_yaml['kinesis_video_streamer']['ros__parameters']['kinesis_video']['stream0']['stream_name']
  default_rekognition = default_config_yaml['kinesis_video_streamer']['ros__parameters']['kinesis_video']['stream0']['rekognition_data_stream']

  ld = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      NODE_NAME,
      default_value="kinesis_video_streamer",
    ),
    launch.actions.DeclareLaunchArgument(
      CONFIG,
      default_value=default_config
    ),
    launch.actions.DeclareLaunchArgument(
      AWS_REGION,
      default_value=default_aws_region
    ),
    launch.actions.DeclareLaunchArgument(
      STREAM_NAME,
      default_value=default_stream_name
    ),
    launch.actions.DeclareLaunchArgument(
      REKOGNITION_DATA_STREAM,
      default_value=default_rekognition
    )
  ])

  node_parameters = [launch.substitutions.LaunchConfiguration(CONFIG)]
  node_parameters.append({
    'aws_client_configuration': {
      'region': launch.substitutions.LaunchConfiguration(AWS_REGION)
    }
  })
  node_parameters.append({
    'kinesis_video': {
      'stream0': {
        'stream_name': launch.substitutions.LaunchConfiguration(STREAM_NAME)
      }
    }
  })
  node_parameters.append({
    'kinesis_video': {
      'stream0': {
        'rekognition_data_stream': launch.substitutions.LaunchConfiguration(REKOGNITION_DATA_STREAM)
      }
    }
  })
  logger_config_path = get_logger_config_path(default_config_yaml)
  if logger_config_path is not None:
    node_parameters.append({
      'kinesis_video': {
        'log4cplus_config': logger_config_path
      }
    })

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
