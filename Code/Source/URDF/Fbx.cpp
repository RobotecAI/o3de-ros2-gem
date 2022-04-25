/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Fbx.h"

#include <fstream>
#include <string>

#include <AzCore/Debug/Trace.h>

namespace ROS2
{
    using Node = Fbx::Node;
    using Property = Fbx::Property;
    using FileType = Fbx::FileType;

    std::string AnyToString(const std::any & a)
    {
        if (a.type() == typeid(int))
        {
            return std::to_string(std::any_cast<int>(a));
        }
        else if (a.type() == typeid(unsigned))
        {
            return std::to_string(std::any_cast<unsigned>(a));
        }
        else if (a.type() == typeid(float))
        {
            return std::to_string(std::any_cast<float>(a));
        }
        else if (a.type() == typeid(const char*))
        {
            return std::string("\"") + std::any_cast<const char*>(a) + std::string("\"");
        }
        else if (a.type() == typeid(std::string))
        {
            return std::string("\"") + std::any_cast<std::string>(a) + std::string("\"");
        }

        return "";
        // throw std::runtime_error("Unsupported type!");
    }

    Fbx::Node::Node(const std::string & name, const Properties & properties)
        : m_name(name)
        , m_properties(properties)
    {}

    std::string Fbx::Node::GetName() const
    {
        return m_name;
    }

    std::vector<Node> Fbx::Node::GetChildren() const
    {
        return m_children;
    }

    Fbx::Properties Fbx::Node::GetProperties() const
    {
        return m_properties;
    }

    bool Fbx::Node::HasChildren() const
    {
        return !m_children.empty();
    }

    bool Fbx::Node::HasProperties() const
    {
        return !m_properties.empty();
    }

    void Node::AddProperty(const Property & property)
    {
        m_properties.push_back(property);
    }

    void Node::AddChildNode(const Node & child)
    {
        m_children.push_back(child);
    }

    void Node::AddChildNode(const std::string & name, const Property & property)
    {
        m_children.push_back(Node(name, { property }));
    }

    void Node::AddChildNode(const Node && child)
    {
        m_children.push_back(child);
    }

    std::string Fbx::Node::ToString(int nodeDepth) const
    {
        std::stringstream ss;
        std::string offset;

        // Calculate offset
        for (int i = 0; i < nodeDepth; ++i)
            offset += "  ";

        // Write name
        ss << offset << m_name << ": ";

        if (!HasProperties() && !HasChildren())
        {
            ss << " {\n";
            ss << offset + "}";
        }

        // Write properties
        if (HasProperties())
        {
            bool hasPrevious = false;
            for(const auto & property : m_properties)
            {
                if(hasPrevious)
                {
                    ss << ", ";
                }
                (void)property;
                ss << AnyToString(property);
                hasPrevious = true;
            }
        }

        // Write child nodes
        if (HasChildren())
        {
            ss << " {\n";

            if(m_children.size() > 0)
            {
                for(auto node : m_children)
                {
                    ss << node.ToString(nodeDepth + 1);
                }
            }
            ss << offset + "}\n";
        }
        else
        {
            ss << "\n";
        }

        return ss.str();
    }

    void Fbx::SaveToFile(const std::string & filePath, FileType type)
    {
        // TODO: add support for binary files
        if (type != FileType::Text)
        {
            std::runtime_error(std::string(__func__) + ": Only text file type is supported!");
        }

        std::ofstream file(filePath);
        if (!file.is_open())
        {
            std::runtime_error(std::string(__func__) + ": Unable to open file: " + filePath);
        }

        file << GetFbxString();
        AZ_Printf("Fbx", "Data structure saved to file: %s", filePath.c_str());
    }

    std::string Fbx::GetFbxString() const
    {
        std::string data;
        // Traverse tree in depth-first order and generate string
        for (const auto & n : basicNodes)
        {
            data += n.ToString(0);
        }

        return data;
    }

    void Fbx::CreateFileStructure()
    {
        basicNodes.clear();

        basicNodes.push_back(GetFbxHeaderExtension());
        basicNodes.push_back(Node("GlobalSettings"));
        basicNodes.push_back(Node("Documents"));
        basicNodes.push_back(Node("References"));
        basicNodes.push_back(Node("Definitions"));
        basicNodes.push_back(Node("Objects"));
        basicNodes.push_back(Node("Connections"));
        basicNodes.push_back(Node("Takes"));
    }

    Node Fbx::GetFbxHeaderExtension() const
    {
        Node fbxHeader("FBXHeaderExtension");
        fbxHeader.AddChildNode("FBXHeaderVersion", 1003);
        fbxHeader.AddChildNode("FBXVersion", 7500);
        fbxHeader.AddChildNode(GetTimeStamp());
        fbxHeader.AddChildNode("Creator", "O3DE URDF->FBX Converter");
        fbxHeader.AddChildNode(GetSceneInfo());

        return fbxHeader;
    }

    Node Fbx::GetTimeStamp() const
    {
        // TODO: get proper time stamp
        Node timeStamp("CreationTimeStamp");
        timeStamp.AddChildNode("Version", 1000);
        timeStamp.AddChildNode("Year", 2022);
        timeStamp.AddChildNode("Month", 01);
        timeStamp.AddChildNode("Day", 01);
        timeStamp.AddChildNode("Hour", 0);
        timeStamp.AddChildNode("Minute", 0);
        timeStamp.AddChildNode("Second", 0);
        timeStamp.AddChildNode("Millisecond", 0);

        return timeStamp;
    }

    Node Fbx::GetSceneInfo() const
    {
        Node sceneInfo("SceneInfo");
        sceneInfo.AddProperty("SceneInfo::GlobalInfo");
        sceneInfo.AddProperty("UserData");
        sceneInfo.AddChildNode("Type", "UserData");
        sceneInfo.AddChildNode("Version", 100);
        sceneInfo.AddChildNode(GetMetaData());

        Node properties("Properties70");
        properties.AddChildNode(
            Node("P", {"DocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
        properties.AddChildNode(
            Node("P", {"SrcDocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
        properties.AddChildNode(
            Node("P", {"Original", "Compound", "", ""}));
        properties.AddChildNode(
            Node("P", {"Original|ApplicationVendor", "KString", "", "", "O3DE"}));
        properties.AddChildNode(
            Node("P", {"Original|ApplicationName", "KString", "", "", "O3DE"}));
        properties.AddChildNode(
            Node("P", {"Original|ApplicationVersion", "KString", "", "", "2022"}));
        properties.AddChildNode(
            Node("P", {"Original|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
        properties.AddChildNode(
            Node("P", {"Original|FileName", "KString", "", "", "/dummy_path.fbx"}));
        properties.AddChildNode(
            Node("P", {"LastSaved", "Compound", "", ""}));
        properties.AddChildNode(
            Node("P", {"LastSaved|ApplicationVendor", "KString", "", "", "O3DE"}));
        properties.AddChildNode(
            Node("P", {"LastSaved|ApplicationName", "KString", "", "", "O3DE"}));
        properties.AddChildNode(
            Node("P", {"LastSaved|ApplicationVersion", "KString", "", "", "2022"}));
        properties.AddChildNode(
            Node("P", {"LastSaved|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
        properties.AddChildNode(
            Node("P", {"Original|ApplicationActiveProject", "KString", "", "", "/dummy_path.fbx"}));
        properties.AddChildNode(
            Node("P", {"Original|ApplicationNativeFile", "KString", "", "", "/dummy_path.fbx"}));

        sceneInfo.AddChildNode(std::move(properties));

        return sceneInfo;
    }

    Node Fbx::GetMetaData() const
    {
        Node metaData("MetaData");
        metaData.AddChildNode("Version", 100);
        metaData.AddChildNode("Title", "");
        metaData.AddChildNode("Subject", "");
        metaData.AddChildNode("Author", "");
        metaData.AddChildNode("Keywords", "");
        metaData.AddChildNode("Revision", "");
        metaData.AddChildNode("Comment", "");

        return metaData;
    }

} // namespace ROS2