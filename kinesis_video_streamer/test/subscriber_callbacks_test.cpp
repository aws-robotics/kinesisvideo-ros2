/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_definition_provider.h>
#include "kinesis_video_msgs/msg/kinesis_video_frame.hpp"
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <queue>
#include <gmock/gmock.h>

#include "kinesis_video_streamer_test_utils.h"

using namespace std;
using namespace Aws::Client;
using namespace Aws::Kinesis;
using namespace Aws::Utils::Logging;

TestData * kTestData = nullptr;
AWSROSLogger * kLogger;

class MockKinesisStreamManager : public KinesisStreamManagerInterface
{
public:
  MockKinesisStreamManager(const Aws::Client::ParameterReaderInterface * parameter_reader,
                                const StreamDefinitionProvider * stream_definition_provider,
                                StreamSubscriptionInstaller * subscription_installer) : KinesisStreamManagerInterface(parameter_reader,
 stream_definition_provider, subscription_installer) {}

  MOCK_METHOD2(ProcessCodecPrivateDataForStream, KinesisManagerStatus(const std::string & stream_name, std::vector<uint8_t> codec_private_data));
  MOCK_CONST_METHOD2(PutFrame, KinesisManagerStatus(std::string stream_name, Frame & frame));
  MOCK_CONST_METHOD2(PutMetadata, KinesisManagerStatus(std::string stream_name, const std::string & name, const std::string & value));
};


class SubscriberCallbacksTestBase : public ::testing::Test
{
protected:
    void SetUp() override
    {
        handle = rclcpp::Node::make_shared("test_node");
        kTestData = &test_data;
        stream_definition_provider = new MockStreamDefinitionProvider(&test_data);
        subscription_installer =
                new MockStreamSubscriptionInstaller(&test_data, *stream_manager, handle);
        stream_manager = new MockKinesisStreamManager(&test_data, &parameter_reader,
                                                      stream_definition_provider, subscription_installer);
        subscription_installer->set_stream_manager(stream_manager);
    }

    void TearDown() override
    {
        delete stream_manager;
        delete stream_definition_provider;
        delete subscription_installer;
    }

    std::shared_ptr<rclcpp::Node> handle;
    TestParameterReader parameter_reader;
    TestData test_data;
    MockKinesisStreamManager * stream_manager;
    StreamDefinitionProvider real_stream_definition_provider;
    MockStreamDefinitionProvider * stream_definition_provider;
    MockStreamSubscriptionInstaller * subscription_installer;
};

/**
 * Tests success scenario.
 */
TEST_F(SubscriberCallbacksTestBase, KinesisVideoFrameTransportCallbackTest)
{
    kinesis_video_msgs::msg::KinesisVideoFrame message;
    KinesisVideoFrameTransportCallback(stream_manager, "stream_name", message);
    /* Shouldn't process codec data as the message is empty */
    EXPECT_CALL(stream_manager, ProcessCodecPrivateDataForStream()).Times(0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
//    auto node = rclcpp::Node::make_shared("test_kinesis_video_streamer");
//    AWSROSLogger logger(LogLevel::Trace, node);
//    kLogger = &logger;
    int ret = RUN_ALL_TESTS();
    return ret;
}
