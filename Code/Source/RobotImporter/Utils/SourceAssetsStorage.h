/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>

namespace ROS2::Utils
{
    /// Structure contains essential information about source and product assets in o3de.
    /// It designed to provide necessary information for other classes in URDF converter, e.g. ColliderMaker or Visual maker.
    /// @note ColliderMaker, @note VisualMaker
    struct AvailableAsset
    {
        /// relative path to source asset e.g. `Assets/foo_robot/meshes/bar_link.dae`.
        AZStd::string m_sourceAssetRelativePath;

        /// relative path to source asset e.g. `/home/user/project/Assets/foo_robot/meshes/bar_link.dae`.
        /// It is a location of file in Asset folder - the location in which User has put asset.
        AZStd::string m_sourceAssetGlobalPath;

        /// relative path to source asset e.g. `foo_robot/meshes/bar_link.azmodel`.
        AZStd::string m_productAssetRelativePath;

        /// Product asset ID @see AZ::Data::AssetInfo.
        AZ::Data::AssetId m_assetId;
    };

    /// Structure contains mapping between URDF's path to O3De asset information.
    struct UrdfAsset
    {
        /// unresolved URDF path to mesh, e.g. `package://meshes/bar_link.dae`.
        AZStd::string m_urdfPath;

        /// Resolved path, points to valid mesh in filestystem, e.g. `/home/user/ros_ws/src/foo_robot/meshes/bar_link.dae'
        AZStd::string m_resolvedUrdfPath;

        /// Checksum of the file located in path `m_resolvedUrdfPath`
        AZ::Crc32 m_urdfFileCRC;

        /// Found o3de asset
        AvailableAsset m_availableAssetInfo;
    };

    /// Type that hold result of mapping from URDF path to asset info
    using UrdfAssetMap = AZStd::unordered_map<AZStd::string, Utils::UrdfAsset>;

    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename);

    /// Function takes assets catalog and compute CRC for every source mesh.
    /// ToDo consider limit scope to only sub-directory.
    /// ToDo consider use filesystem instead of AssetCatalogRequestBus::EnumerateAssets.
    /// @returns map where key is crc of source file and value is AvailableAsset.
    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC();

    /// Function is to discover meshes in URDF as o3de assets.
    /// The @param meshesFilenames contains the list of unresolved URDF filenames that are to be found as assets.
    /// Steps:
    /// - Functions resolves URDF filenames with `ResolveURDFPath`.
    /// - Files pointed by resolved URDF patches have their checksum computed `GetFileCRC`.
    /// - Function scans all available o3de assets by calling `GetInterestingSourceAssetsCRC`.
    /// - Suitable mapping to the o3de asset is found by comparing the checksum of the file pointed by the URDF path and source asset.
    /// @param meshesFilenames - list of the unresolved path from the URDF file
    /// @param urdFilename - filename of URDF file, used for resolvement
    /// @returns map where key is unresolved urdf path to AvailableAsset
    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdFilename);

} // namespace ROS2::Utils
