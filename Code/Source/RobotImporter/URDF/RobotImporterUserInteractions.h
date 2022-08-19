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
    class RobotImporterUserInteractions
    {
    public:
        //! Report an error to the user.
        virtual void ReportError(const AZStd::string& errorMessage) = 0;

        //! Report an information to the user.
        virtual void ReportInfo(const AZStd::string& infoMessage) = 0;

        //! Get valid path to the existing URDF file from the user
        //! @return valid path to the existing URDF file or empty optional if the user canceled the operation
        virtual AZStd::optional<AZStd::string> GetURDFPath() = 0;

        //! Validate whether path exists. If yes, ask user to take a proper action to provide correct path.
        //! @param path - path to validate
        //! @return Valid path or empty optional if it was not possible or user cancelled.
        virtual AZStd::optional<AZStd::string> ValidatePrefabPathExistenceAndGetNewIfNecessary(const AZStd::string& path) = 0;
    };

} // namespace ROS2
