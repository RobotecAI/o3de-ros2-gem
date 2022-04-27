/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <any>
#include <string>

#include "UrdfParser.h"
#include "Fbx.h"

namespace ROS2
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! Class for conversion from URDF to Filmbox (.fbx) files
    class UrdfToFbxConverter
    {
    public:
        std::string ConvertUrdfToFbx(const std::string & urdfString);

    private:

    };

} // namespace ROS2