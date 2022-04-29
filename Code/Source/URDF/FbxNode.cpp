/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FbxNode.h"

#include <fstream>
#include <string>

#include <AzCore/Debug/Trace.h>

namespace ROS2
{
    namespace Fbx
    {
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
            else if (a.type() == typeid(double))
            {
                return std::to_string(std::any_cast<double>(a));
            }
            else if (a.type() == typeid(const char*))
            {
                return std::string("\"") + std::any_cast<const char*>(a) + std::string("\"");
            }
            else if (a.type() == typeid(std::string))
            {
                return std::string("\"") + std::any_cast<std::string>(a) + std::string("\"");
            }
            else if (a.type() == typeid(RawString))
            {
                return std::any_cast<RawString>(a).data;
            }

            return "";
        }

        Node::Node(const std::string & name, const Properties & properties)
            : m_name(name)
            , m_properties(properties)
        {}

        std::string Node::GetName() const
        {
            return m_name;
        }

        std::vector<Node> Node::GetChildren() const
        {
            return m_children;
        }

        Properties Node::GetProperties() const
        {
            return m_properties;
        }

        bool Node::HasChildren() const
        {
            return !m_children.empty();
        }

        bool Node::HasProperties() const
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

        std::string Node::ToString(int nodeDepth) const
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

    } // namespace Fbx
} // namespace ROS2