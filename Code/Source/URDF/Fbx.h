/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>
#include <any>

namespace ROS2
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
        using Property = std::any;
        using Properties = std::vector<Property>;
        enum class FileType { Text, Binary };

        //! A node in FBX file tree structure.
        //! Each named node could contain children nodes (subnodes) and properties.
        class Node
        {
            public:
                Node(const std::string & name, const Properties & properties = {});

                std::string GetName() const;
                std::vector<Node> GetChildren() const;
                Properties GetProperties() const;
                bool HasChildren() const;
                bool HasProperties() const;

                void AddProperty(const Property & property);
                void AddChildNode(const Node & child);
                void AddChildNode(const std::string & name, const Property & property);
                void AddChildNode(const Node && child);

                std::string ToString(int nodeDepth = 0)  const;

            private:
                std::string m_name;
                std::vector<Node> m_children;
                Properties m_properties;
        };

        //! Save the current FBX structure to file.
        //! Note: only ASCII version is supported
        void SaveToFile(const std::string & filePath, FileType type = FileType::Text);

        //! Return the string with ASCII version of current FBX structure.
        std::string GetFbxString();

    private:
        //! Generate the fbx file structure.
        void GenerateFbxStructure();
        // Default FBX file header
        Node GetFbxHeaderExtension() const;
        // Default global settings
        Node GetGlobalSettings() const;
        Node GetTimeStamp() const;
        Node GetSceneInfo() const;
        Node GetMetaData() const;

        Node GetDocuments() const;
        Node GetDefinitions() const;
        Node GetObjects() const;
        Node GetConnections() const;

        std::vector<Node> basicNodes;
        bool nodesUpdated = false;
    };

} // namespace ROS2