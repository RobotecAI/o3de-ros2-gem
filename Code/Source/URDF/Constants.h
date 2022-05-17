/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>

namespace ROS2
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! The struct contains all constant parameters used in FBX files.
    //! TODO: Could be moved to configuration file
    namespace Fbx
    {
        struct Constants
        {
            struct FbxHeader
            {
                inline static const int headerVersion = 1003;
                inline static const int fileVersion = 7500;
                inline static const std::string creatorName = "O3DE ROS2 Gem";
                inline static const int timeStampVersion = 1000;
                inline static const int metaDataVersion = 100;
                inline static const std::string metaDataTitle = "";
                inline static const int sceneInfoVersion = 100;
                inline static const std::string applicationName = "O3DE";
                inline static const std::string applicationVersion = "2022";
                inline static const std::string documentActiveAnimStackName = "Take 001";
                inline static const std::string dummyPath = "/dummy_path.fbx";
                inline static const std::string fileCreationDate = "01/01/2022 00:00:00.000";
            };

            struct GlobalSettings
            {
                inline static const int version = 1000;
                inline static const int defaultTimeSpan = 1924423250;
                inline static const std::string defaultCamera = "Producer Perspective";
                inline static const int timeMode = 11;
                inline static const int timeProtocol = 2;
                inline static const int snapOnFrameMode = 0;
                inline static const int customFrameRate = -1;
                inline static const int currentTimeMarker = -1;
            };

            struct Material
            {
                inline static const int defaultVersion = 102;
                inline static const std::string defaultShadingModel = "phong";
                inline static const float defaultDiffuseFactor = 0.9;
                inline static const float defaultOpacity = 1.0;
                inline static const float defaultReflectivity = 0.0;
            };

            struct Object
            {
                inline static const int modelVersion = 232;
                inline static const int geometryVersion = 102;
                inline static const int layerElementNormalVersion = 102;
                inline static const int layerElementUvVersion = 101;
            };

            // Other
            inline static const int definitionsVersion = 100;
            inline static const std::string defaultConnectionType = "OO";
        };
    } // namespace Fbx
} // namespace ROS2