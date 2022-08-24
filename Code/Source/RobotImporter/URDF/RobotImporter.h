/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
#include <functional>

namespace ROS2
{
    struct RobotImporterConfig
    {
        AZStd::string urdfFilePath;
        AZStd::string prefabFilePath;
    };

    namespace RobotImporter {
        void Import(const RobotImporterConfig& config,
                    std::function<void(const AZStd::string&)> infoLogger,
                    std::function<void(const AZStd::string&)> errorLogger);
    }

} // namespace ROS2
