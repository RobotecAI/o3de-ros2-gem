/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */

#include "Utilities/ROS2Names.h"
#include <vector>
#include <string>
#include <AzTest/AzTest.h>
#include <AzCore/Memory/SystemAllocator.h>

namespace ROS2
{
    class ROS2NamesTest : public ::testing::Test
    {
    public:
        std::vector<AZStd::string> validTopicNames =
        {
            "/chatter",
            "chatter",
            "namespace/chatter",
            "/namespace/chatter",
            "chatter42",
            "point_cloud"
        };

        std::vector<AZStd::string> invalidTopicNames =
        {
            "",
            "/",
            "invalidCharacter!",
            "5tartsWithNumber"
        };
    };

    TEST_F(ROS2NamesTest, ValidTopicsAreAccepted)
    {
        for (const auto& topic : validTopicNames)
        {
            EXPECT_TRUE(ROS2Names::ValidateTopic(topic).IsSuccess());
        }
    }

    TEST_F(ROS2NamesTest, InvalidTopicsAreRejected)
    {
        for (const auto& topic : invalidTopicNames)
        {
            EXPECT_FALSE(ROS2Names::ValidateTopic(topic).IsSuccess());
        }
    }
}
