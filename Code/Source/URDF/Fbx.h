/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>
#include <vector>

#include "FbxNode.h"

namespace ROS2
{
    namespace Fbx
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //! The class represents the FBX file structure and operations on that files.
        //!
        //! FBX file has a tree based structure.
        //! Top structure of the FBX file consists of the following sections:
        //!   - FBXHeaderExtension (mandatory) - metadata of file.
        //!   - GlobalSettings (mandatory) - general data properties.
        //!   - Documents (optional) - ?
        //!   - References (optional) - ?
        //!   - Definitions (optional) - ?
        //!   - Objects (optional) - static data storage like objects, geometries, textures and materials.
        //!   - Connections (optional) - defines connections between data defined in Objects.
        //!                              For example which object uses specific material.
        //!   - Takes (optional) - animations definitions.
        //!
        //! Additional documentation about FBX structure:
        //! https://web.archive.org/web/20160605023014/https://wiki.blender.org/index.php/User:Mont29/Foundation/FBX_File_Structure
        //! https://banexdevblog.wordpress.com/2014/06/23/a-quick-tutorial-about-the-fbx-ascii-format/
        //!
        //! Example FBX file
        //! https://www.ics.uci.edu/~djp3/classes/2014_03_ICS163/tasks/arMarker/Unity/arMarker/Assets/CactusPack/Meshes/Sprites/Rock_Medium_SPR.fbx
        class Fbx
        {
        public:
            enum class FileType { Text, Binary };

            //! Save the current FBX structure to file.
            //! Note: only ASCII version is supported
            void SaveToFile(const std::string & filePath, FileType type = FileType::Text);

            //! Return the string with ASCII version of current FBX structure.
            std::string GetFbxString();

            ///! Reset the internal data used for FBX file structure creation.
            void Reset();

        private:
            struct Connection {
                Connection(int parent, int child, std::string connectionType)
                    : parentId(parent), childId(child), type(connectionType) {}

                int parentId = -1;
                int childId = -1;
                std::string type = "OO";
            };

            //! Generate the fbx file structure.
            void GenerateFbxStructure();

            // Default FBX file header
            Node GetFbxHeaderExtension() const;
            Node GetTimeStamp() const;
            Node GetSceneInfo() const;
            Node GetMetaData() const;

            // Default global settings
            Node GetGlobalSettings() const;

            Node GetDocuments() const;
            Node GetDefinitions() const;

            // Objects creation
            Node GetObjects();
            Node CreateGeometryCube(int id, double size = 1.0) const;

            Node GetConnections() const;

            std::vector<Node> m_basicNodes;
            bool m_nodesUpdated = false;

            std::vector<Connection> m_connections;
        };
    } // namespace Fbx
} // namespace ROS2