/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/ComponentBus.h"
#include "AzCore/std/string/string.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/function/function_template.h>

namespace ROS2
{
    namespace
    {
        const AZStd::function<bool(const AZStd::string&)> fileExistsCall = [](const AZStd::string& fn) -> bool
        {
            return AZ::IO::SystemFile::Exists(fn.c_str());
        };

    }
    namespace Utils
    {

        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        /// Goes through URDF and finds world to entity transformation for us
        AZ::Transform getWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t = AZ::Transform::Identity());

        /// Retrieve all links in urdf file
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> getAllLinks(const std::vector<urdf::LinkSharedPtr>& links);

        /// Retrieve all joints in urdf file
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> getAllJoints(const std::vector<urdf::LinkSharedPtr>& links);

        /// Retireve all meshes as URDF paths
        /// @param visual - find for visual
        /// @param colliders - find for colliders
        /// @returns set of meshes
        AZStd::unordered_set<AZStd::string> getMeshesFilenames(const urdf::LinkConstSharedPtr& root_link, bool visual, bool colliders);

        /// Finds global path from URDF path
        AZStd::string resolveURDFPath(
            AZStd::string path,
            const AZStd::string& urdf_path,
            const AZStd::function<bool(const AZStd::string&)>& fileExists = fileExistsCall);

    } // namespace Utils
} // namespace ROS2