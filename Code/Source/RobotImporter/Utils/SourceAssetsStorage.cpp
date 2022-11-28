/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "SourceAssetsStorage.h"
#include "RobotImporter/Utils/RobotImporterUtils.h"

namespace ROS2::Utils
{

    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename)
    {
        auto fileSize = AZ::IO::SystemFile::Length(filename.c_str());
        fileSize = AZStd::min(fileSize, 1024ull); // limit crc computation to first kilobyte
        if (fileSize == 0)
        {
            return AZ::Crc32();
        }
        AZStd::vector<char> buffer(fileSize + 1);
        buffer[fileSize] = '\0';
        if (!AZ::IO::SystemFile::Read(filename.c_str(), buffer.data(), fileSize))
        {
            return AZ::Crc32();
        }
        AZ::Crc32 r;
        r.Add(buffer.data(), fileSize);
        return r;
    }

    AZStd::unordered_map<AZ::Crc32, available_asset> GetInterestingSourceAssetsCRC()
    {
        const AZStd::unordered_set<AZStd::string> kInterestingExtensions{ ".dae", ".stl", ".obj" };
        AZStd::unordered_map<AZ::Crc32, available_asset> availableAssets;

        // take all meshes in catalog
        AZ::Data::AssetCatalogRequests::AssetEnumerationCB collectAssetsCb =
            [&](const AZ::Data::AssetId id, const AZ::Data::AssetInfo& info)
        {
            if (AZ::Data::AssetManager::Instance().GetHandler(info.m_assetType))
            {
                if (!info.m_relativePath.ends_with(".azmodel"))
                {
                    return;
                }
                using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
                bool pathFound{ false };
                AZStd::string fullSourcePathStr;

                AssetSysReqBus::BroadcastResult(
                    pathFound, &AssetSysReqBus::Events::GetFullSourcePathFromRelativeProductPath, info.m_relativePath, fullSourcePathStr);
                if (!pathFound)
                {
                    return;
                }
                const AZ::IO::Path fullSourcePath(fullSourcePathStr);
                const AZStd::string extension = fullSourcePath.Extension().Native();

                if (!kInterestingExtensions.contains(extension))
                {
                    return;
                }

                AZ::Crc32 crc = Utils::GetFileCRC(fullSourcePathStr);
                AZ_Printf(
                    "RobotImporterWidget",
                    "m_relativePath %s %s : %s %llu \n",
                    info.m_relativePath.c_str(),
                    info.m_assetId.ToString<AZStd::string>().c_str(),
                    fullSourcePath.c_str(),
                    crc);

                available_asset t;
                t.m_sourceAssetRelativePath = info.m_relativePath;
                t.assetId = info.m_assetId;
                t.m_sourceAssetGlobalPath = fullSourcePathStr;
                t.m_productAssetRelativePath = info.m_relativePath;
                availableAssets[crc] = t;
            }
        };
        AZ::Data::AssetCatalogRequestBus::Broadcast(
            &AZ::Data::AssetCatalogRequestBus::Events::EnumerateAssets, nullptr, collectAssetsCb, nullptr);

        return availableAssets;
    }

    AZStd::unordered_map<AZStd::string, Utils::urdf_asset> FindAssetsForUrdf(
        const AZStd::unordered_set<AZStd::string>& meshes_filenames, const AZStd::string& urdf_filename)
    {
        AZStd::unordered_map<AZStd::string, Utils::urdf_asset> urdf_to_asset;
        for (const auto& t : meshes_filenames)
        {
            Utils::urdf_asset asset;
            asset.m_urdfPath = t;
            asset.m_resolvedUrdfPath = Utils::ResolveURDFPath(asset.m_urdfPath, urdf_filename);
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);
            urdf_to_asset.emplace(t, AZStd::move(asset));
        }
        const AZStd::unordered_map<AZ::Crc32, available_asset> available_assets = Utils::GetInterestingSourceAssetsCRC();
        for (auto it = urdf_to_asset.begin(); it != urdf_to_asset.end(); it++)
        {
            Utils::urdf_asset& asset = it->second;
            auto found_source_asset = available_assets.find(asset.m_urdfFileCRC);
            if (found_source_asset != available_assets.end())
            {
                asset.m_availableAssetInfo = found_source_asset->second;
            }
        }
        return urdf_to_asset;
    }

} // namespace ROS2::Utils
