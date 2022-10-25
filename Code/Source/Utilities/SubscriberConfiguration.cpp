/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SubscriberConfiguration.h"
#include "ROS2/Utilities/ROS2Names.h"
#include <AzCore/Serialization/EditContext.h>

namespace ROS2
{
    void SubscriberConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SubscriberConfiguration>()
                ->Version(1)
                ->Field("Topic", &SubscriberConfiguration::m_topic)
                ->Field("QoS", &SubscriberConfiguration::m_qos);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SubscriberConfiguration>("Subscriber configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SubscriberConfiguration::m_topic, "Topic", "Topic with no namespace")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateTopicField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SubscriberConfiguration::m_qos, "QoS", "Quality of Service");
            }
        }
    };
} // namespace ROS2
