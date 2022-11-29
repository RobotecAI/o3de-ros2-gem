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
        const AZStd::function<bool(const AZStd::string&)> fileExistsCall = [](const AZStd::string& filename) -> bool
        {
            return AZ::IO::SystemFile::Exists(filename.c_str());
        };
    }

    namespace Utils
    {
        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        /// Goes through URDF and finds world to entity transformation for us.
        AZ::Transform GetWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t = AZ::Transform::Identity());

        /// Retrieve all childLinks in urdf file.
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> GetAllLinks(const std::vector<urdf::LinkSharedPtr>& childLinks);

        /// Retrieve all joints in urdf file.
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> GetAllJoints(const std::vector<urdf::LinkSharedPtr>& childLinks);

        /// Retrieve all meshes as URDF paths.
        /// Function traverse URDF in recursive manner.
        /// It obtains referenced meshes' filenames.
        /// Note that returned filenames are unresolved URDF patches.
        /// @param visual - find for visual.
        /// @param colliders - find for colliders.
        /// @returns set of meshes' filenames.
        AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const urdf::LinkConstSharedPtr& rootLink, bool visual, bool colliders);

        /// Finds global path from URDF path.
        /// @param unresolvedPath - unresolved URDF path, example : `package://meshes/foo.dae`.
        /// @param urdfFilePath - absolute path of URDF file which contains path that are to be resolved.
        /// @param fileExists - functor to check if given file exists. Exposed for unit test, default one should be used.
        AZStd::string ResolveURDFPath(
            AZStd::string unresolvedPath,
            const AZStd::string& urdfFilePath,
            const AZStd::function<bool(const AZStd::string&)>& fileExists = fileExistsCall);

    } // namespace Utils
} // namespace ROS2