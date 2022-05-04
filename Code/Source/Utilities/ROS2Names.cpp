/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Names.h"
#include <AzCore/std/string/regex.h>
#include <rcl/validate_topic_name.h>

namespace ROS2
{
    AZStd::string ROS2Names::GetNamespacedName(const AZStd::string& ns, const AZStd::string& name)
    {
        if (ns.empty())
        {
            return name;
        }

        return AZStd::string::format("%s/%s", ns.c_str(), name.c_str());;
    }

    AZStd::string ROS2Names::RosifyName(const AZStd::string& input)
    {
        // TODO - add unit tests
        // TODO - implement stricter guidelines and differentiate: https://design.ros2.org/articles/topic_and_service_names.html
        // TODO - add check whether it begins with a number (and if so, prepend underscore)
        const AZStd::regex ros2Disallowedlist("[^0-9|a-z|A-Z|_]");
        return AZStd::regex_replace(input, ros2Disallowedlist, "_");
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateTopic(const AZStd::string& topic)
    {
        int validationResult;
        size_t invalidIndex;
        if ( rcl_validate_topic_name(topic.c_str(), &validationResult, &invalidIndex) != RCL_RET_OK)
        {
            AZ_Error("ValidateTopic", false, "Call to rcl validation for topic failed");
            return AZ::Failure(AZStd::string("Unable to validate topic due to rcl error"));
        }

        if (RCL_TOPIC_NAME_VALID == validationResult)
        {
            return AZ::Success();
        }

        return AZ::Failure(AZStd::string(rcl_topic_name_validation_result_string(validationResult)));
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateTopicField(void* newValue, [[maybe_unused]] const AZ::Uuid& valueType)
    {
        AZStd::string topic(static_cast<const char*>(newValue));
        return ValidateTopic(topic);
    }
}  // namespace ROS2