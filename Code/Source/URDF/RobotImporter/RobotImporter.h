/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "URDF/RobotImporter/RobotImporterInputInterface.h"
#include "URDF/RobotImporter/URDFPrefabMaker.h"
#include "URDF/UrdfParser.h"
#include <memory>

namespace ROS2
{

    class RobotImporter
    {
    public:
        explicit RobotImporter(RobotImporterInputInterface& interactionInterface);
        void Import();

    private:
        // Has to be a reference since Qt is not very kin of smart pointers. It should be fixed
        // For now, programmer must guarantee that this reference is valid for as long as the importer lives
        RobotImporterInputInterface& m_interactionInterface;
        std::optional<URDFPrefabMaker> m_prefabMaker;
    };

} // namespace ROS2
