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
#include <atomic>

#include "RobotImporter/URDF/URDFPrefabMaker.h"

namespace ROS2
{
    struct RobotImporterConfig
    {
        AZStd::string urdfFilePath;
        AZStd::string prefabFilePath;
    };

    class RobotImporter
    {
    public:
        RobotImporter(std::function<void(const AZStd::string&)> infoLogger,
                      std::function<void(const AZStd::string&)> errorLogger);
        void Import(const RobotImporterConfig& config);
        void Update(std::function<void()> importFinishedCb);

    private:
        std::atomic_bool m_isProcessingAssets;
        std::optional<URDFPrefabMaker> m_prefabMaker;
        std::function<void(const AZStd::string&)> m_infoLogger;
        std::function<void(const AZStd::string&)> m_errorLogger;
    };

} // namespace ROS2
