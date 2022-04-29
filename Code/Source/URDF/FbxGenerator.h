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
#include "UniqueIdGenerator.h"

namespace ROS2
{
    namespace Fbx
    {
        //! Define type of the FBX file.
        enum class FileType { Text, Binary };

        //! RGB color
        struct Color
        {
            float r = 0;
            float g = 0;
            float b = 0;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //! The class represents the FBX file structure generator and operations on that files.
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
        class FbxGenerator
        {
        public:
            //! Save the current FBX structure to file.
            //! Note: only ASCII version is supported
            void SaveToFile(const std::string & filePath, FileType type = FileType::Text);

            //! Return the string with ASCII version of current FBX structure.
            std::string GetFbxString();

            //! Reset the internal data used for FBX file structure creation.
            void Reset();

            //! Add cube object
            //! Returns id of created object
            //! Notice: First added object is attached to the root node.
            //! TODO: generalization for other types of objects e.g. cuboid, cylinder
            //! TODO: handle textures
            Id AddCubeObject(const std::string & objectName, double size, Id materialId);

            //! Add default material and returns its id
            //! TODO: add more material parameters
            Id AddMaterial(const std::string & materialName, const Color & color);

            ///! Create relation between objects
            void SetRelationBetweenObjects(Id parentId, Id childId);

        private:
            struct Connection
            {
                Connection(Id parent, Id child, std::string connectionType)
                    : parentId(parent), childId(child), type(connectionType) {}

                Id parentId = -1;
                Id childId = -1;
                std::string type = "OO";
            };

            //! Generate the FBX file structure.
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
            NodeWithId CreateModel(const std::string & modelName) const;
            NodeWithId CreateMaterial(const std::string & name, const Color & color) const;
            NodeWithId CreateGeometryCube(double size = 1.0) const;

            // Generate connections based on the m_connections.
            Node GenerateConnections() const;

            static constexpr Id rootId = 0;

            std::vector<Node> m_basicNodes;
            bool m_nodesUpdated = false;
            std::vector<Connection> m_connections;

            std::shared_ptr<Node> m_objects = std::make_shared<Node>("Objects");
            bool m_first_object = true;
        };
    } // namespace Fbx
} // namespace ROS2