/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>

namespace ROS2
{
    class RobotImporterInputInterface
    {
    public:
        enum ExistingPrefabAction
        {
            Overwrite,
            NewName,
            Cancel
        };
        virtual AZStd::string GetURDFPath() = 0;
        virtual void ReportError(AZStd::string errorMessage) = 0;
        virtual void ReportWarning(AZStd::string warningMessage) = 0;
        virtual void ReportInfo(AZStd::string infoMessage) = 0;
        virtual ExistingPrefabAction GetExistingPrefabAction() = 0;
        virtual AZStd::string GetNewPrefabPath() = 0;
    };

} // namespace ROS2
