/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <QWidget>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/optional.h>

namespace ROS2::RobotImporterWidgetUtils
{
    enum class ExistingPrefabAction
    {
        Overwrite,
        CreateWithNewName,
        Cancel
    };

    //! Get valid path to the existing URDF file from the user
    //! @return valid path to the existing URDF file or empty optional if the user canceled the operation
    //! @param parent - parent widget for the widgets used inside this function
    AZStd::optional<AZStd::string> QueryUserForURDFPath(QWidget* parent = nullptr);

} // namespace ROS2::RobotImporterWidgetUtils
