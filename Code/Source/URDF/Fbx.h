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
    //! The class represents the Fbx file structure and operations on that files.
    //!
    //! Fbx file has a tree based structure.
    //! https://web.archive.org/web/20160605023014/https://wiki.blender.org/index.php/User:Mont29/Foundation/FBX_File_Structure
    class Fbx
    {
    public:
        using Property = std::any;
        using Properties = std::vector<Property>;
        enum class FileType { Text, Binary };

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

        void SaveToFile(const std::string & filePath, FileType type = FileType::Text);

        std::string GetFbxString() const;

        void CreateFileStructure();

    private:
        Node GetFbxHeaderExtension() const;
        Node GetTimeStamp() const;
        Node GetSceneInfo() const;
        Node GetMetaData() const;

        std::vector<Node> basicNodes;
    };

} // namespace ROS2