/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/URDF/RobotImporterUserInteractions.h"
#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include <memory>


namespace ROS2
{
    class RobotImporterWidget;

    class RobotImporter
    {
    public:
        explicit RobotImporter(RobotImporterWidget& interactionInterface);
        void Import();

    private:
        // Has to be a reference since Qt is not very kin of smart pointers. It should be fixed
        // For now, programmer must guarantee that this reference is valid for as long as the importer lives
        RobotImporterWidget& m_robotImporterWidget;
    };

} // namespace ROS2
