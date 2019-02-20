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
#pragma once

namespace Aws {
namespace Kinesis {
/**
 * By default, we use one thread to handle incoming messages. You may specify a different setting
 * via the "spinner_thread_count" parameter.
 * @note There are no ordering guarantees; assumptions can be made when dealing with real-time video
 * streaming: Since frame duration is 1/fps seconds - ample time to process a message - misordering
 * of frames is highly unlikely (but still possible). Finally, message timestamps can be used to
 * validate (or fix) the order by the receiving application.
 */
constexpr uint32_t kDefaultNumberOfSpinnerThreads = 1;
const char * kSpinnerThreadCountOverrideParameter = "spinner_thread_count";

} // namespace Kinesis
} // namespace Aws