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

    struct available_asset
    {
        AZStd::string m_sourceAssetRelativePath;
        AZStd::string m_sourceAssetGlobalPath;
        AZStd::string m_productAssetRelativePath;
        AZ::Data::AssetId assetId;
    };

    struct urdf_asset
    {
        AZStd::string m_urdfPath;
        AZStd::string m_resolvedUrdfPath;
        AZ::Crc32 m_urdfFileCRC;
        available_asset m_availableAssetInfo;
    };

    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename);

    /// Function takes assets catalog and compute CRC for every source mesh
    /// ToDo consider lazy initialization / future-promise threading
    /// ToDo consider limit scope to only sub-directory
    /// ToDo consider use filesystem instead of AssetCatalogRequestBus::EnumerateAssets
    /// @returns map where key is crc of source file and value is a azmodel relative path
    AZStd::unordered_map<AZ::Crc32, available_asset> GetInterestingSourceAssetsCRC();

    AZStd::unordered_map<AZStd::string, urdf_asset> FindAssetsForUrdf(
        const AZStd::unordered_set<AZStd::string>& meshes_filenames, const AZStd::string& urdf_filename);

} // namespace ROS2::Utils
